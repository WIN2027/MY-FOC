#include "Protocol_HAL.h"
#include <string.h>

/* 关键断言：DMA 单批缓冲不能超过协议层内部接收缓冲，
 * 否则 Protocol_Parse 会整批拒绝（RX_OVERFLOW）导致“完全收不到” */
PROTOCOL_SA(PROTOCOL_HAL_RX_BUF_SIZE > 0 &&
            PROTOCOL_HAL_RX_BUF_SIZE <= 0xFFFF,
            "PROTOCOL_HAL_RX_BUF_SIZE must be in (0, 65535]");
PROTOCOL_SA(PROTOCOL_HAL_RX_BUF_SIZE <= PROTOCOL_RX_BUFFER_SIZE,
            "PROTOCOL_HAL_RX_BUF_SIZE must be <= PROTOCOL_RX_BUFFER_SIZE: "
            "Protocol_Parse rejects any batch larger than its internal buffer");
PROTOCOL_SA(PROTOCOL_HAL_MAX_INSTANCES >= 1 && PROTOCOL_HAL_MAX_INSTANCES <= 255,
            "PROTOCOL_HAL_MAX_INSTANCES must be in [1, 255]");
PROTOCOL_SA(PROTOCOL_HAL_TX_QUEUE_DEPTH >= 2,
            "PROTOCOL_HAL_TX_QUEUE_DEPTH must be >= 2 (ring keeps one empty slot)");
PROTOCOL_SA(PROTOCOL_HAL_TX_QUEUE_DEPTH <= 64,
            "PROTOCOL_HAL_TX_QUEUE_DEPTH is unreasonably large");
PROTOCOL_SA(sizeof(((Protocol_HAL_Context_t *)0)->tx_buf) >= PROTOCOL_HAL_FRAME_MAX,
            "tx_buf too small for a max frame");

/* ==================== 实例表（huart -> HAL 上下文映射） ====================
 * HAL 回调只有 huart 参数，通过本表查回到具体的 HAL 上下文。
 * 写入仅发生在 Protocol_HAL_Init/Deinit（启动/停止阶段），读取发生在中断中；
 * 若运行期动态 Init/Deinit，请自行在工程层加临界区。
 */
static Protocol_HAL_Context_t *g_hal_table[PROTOCOL_HAL_MAX_INSTANCES];

static bool hal_register(Protocol_HAL_Context_t *h) {
    for (int i = 0; i < PROTOCOL_HAL_MAX_INSTANCES; i++) {
        if (g_hal_table[i] == h) return true;          /* 已注册 */
        if (g_hal_table[i] == NULL) {
            g_hal_table[i] = h;
            return true;
        }
    }
    return false;                                       /* 表满 */
}

static void hal_unregister(Protocol_HAL_Context_t *h) {
    for (int i = 0; i < PROTOCOL_HAL_MAX_INSTANCES; i++) {
        if (g_hal_table[i] == h) g_hal_table[i] = NULL;
    }
}

static Protocol_HAL_Context_t *hal_find_by_huart(UART_HandleTypeDef *huart) {
    for (int i = 0; i < PROTOCOL_HAL_MAX_INSTANCES; i++) {
        if (g_hal_table[i] && g_hal_table[i]->huart == huart) {
            return g_hal_table[i];
        }
    }
    return NULL;
}

/* ==================== 内部：启动/重挂接收 ==================== */
static bool hal_start_rx(Protocol_HAL_Context_t *h) {
    if (h == NULL || h->huart == NULL) return false;

    if (HAL_UARTEx_ReceiveToIdle_DMA(h->huart, h->rx_dma_buf,
                                     sizeof(h->rx_dma_buf)) != HAL_OK) {
        /* 重挂失败：置标志，由主循环 Protocol_HAL_Process 重新 Abort 后重试 */
        h->rx_active = false;
        h->rx_need_restart = true;
        return false;
    }
    if (h->huart->hdmarx) {
        /* HAL 在 XferHalfCpltCallback 非空时会打开 HT；本模块只需要 IDLE/TC，
         * 关闭 HT 避免半批回调（若 HAL 版本仍回调 HT，RxEventCallback 会按
         * 增量数据解析且不重启接收）。 */
        __HAL_DMA_DISABLE_IT(h->huart->hdmarx, DMA_IT_HT);
    }
    h->rx_active = true;
    h->rx_need_restart = false;
    return true;
}

/* ==================== 内部：发送环泵送 ====================
 * 调用者无需关中断；函数内部只在队列/状态操作时短临界区，
 * HAL_UART_Transmit_DMA 在临界区外执行（降低对 FOC ADC 中断的延迟）。
 */
static void hal_tx_pump(Protocol_HAL_Context_t *h) {
    uint16_t slot;
    uint16_t len;
    uint32_t primask;

    if (h == NULL || h->huart == NULL) return;

    primask = __get_PRIMASK();
    __disable_irq();

    if (h->tx_busy || (h->tx_head == h->tx_tail)) {
        __set_PRIMASK(primask);
        return;
    }

    slot = h->tx_tail;
    memcpy(h->tx_buf, h->tx_queue[slot], sizeof(h->tx_buf));

    /* 防御：队列内容被破坏时不要越过 tx_buf 发送 */
    {
        const uint8_t payload_len = h->tx_buf[2];
        if (payload_len > PROTOCOL_MAX_PAYLOAD) {
            h->tx_tail = slot;
            h->tx_error_count++;
            __set_PRIMASK(primask);
            return;
        }
        len = (uint16_t)payload_len + PROTOCOL_FRAME_OVERHEAD;
    }

    h->tx_tail = (uint16_t)((slot + 1U) % PROTOCOL_HAL_TX_QUEUE_DEPTH);
    h->tx_busy = true;
    h->tx_starting = true;
    h->tx_start_tick = HAL_GetTick();

    __set_PRIMASK(primask);

    if (HAL_UART_Transmit_DMA(h->huart, h->tx_buf, len) != HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        h->tx_busy = false;
        h->tx_starting = false;
        h->tx_tail = slot;             /* 发送未启动，帧退回队列 */
        h->tx_error_count++;
        __set_PRIMASK(primask);
    } else {
        primask = __get_PRIMASK();
        __disable_irq();
        h->tx_starting = false;
        __set_PRIMASK(primask);
    }
}

/* ==================== 发送回调（由协议层调用，user = HAL 上下文） ====================
 * 发送策略：所有帧先入环（拷贝），再由 hal_tx_pump 启动 DMA。
 * 这样同一批到达的多条命令的应答不会因为 tx_busy 被丢弃。
 */
static bool Protocol_HAL_Send(void *user, const uint8_t *data, uint16_t len) {
    Protocol_HAL_Context_t *h = (Protocol_HAL_Context_t *)user;
    uint32_t primask;
    bool accepted = false;

    if (h == NULL || h->huart == NULL || data == NULL || len == 0U) return false;
    if (len > sizeof(h->tx_buf) || len < PROTOCOL_FRAME_OVERHEAD) return false;
    if ((uint16_t)data[2] + PROTOCOL_FRAME_OVERHEAD != len) return false;

    primask = __get_PRIMASK();
    __disable_irq();

    {
        const uint16_t next = (uint16_t)((h->tx_head + 1U) % PROTOCOL_HAL_TX_QUEUE_DEPTH);
        if (next != h->tx_tail) {
            memcpy(h->tx_queue[h->tx_head], data, len);
            h->tx_head = next;
            accepted = true;
        } else {
            h->tx_drop_count++;        /* 队列满：丢弃并计数 */
        }
    }

    __set_PRIMASK(primask);

    if (accepted) hal_tx_pump(h);
    return accepted;
}

/* ==================== 初始化 ==================== */
bool Protocol_HAL_Init(Protocol_HAL_Context_t *h,
                       Protocol_Context_t *proto,
                       UART_HandleTypeDef *huart) {
    Protocol_HAL_Context_t *owner;

    if (h == NULL || proto == NULL || huart == NULL) return false;

    /* 仅支持 Normal DMA；TX 也必须是 DMA（发送环依赖 HAL_UART_Transmit_DMA） */
    if (huart->hdmarx != NULL && huart->hdmarx->Init.Mode != DMA_NORMAL) {
        return false;
    }
    if (huart->hdmatx == NULL) {
        return false;
    }
    if (huart->hdmatx->Init.Mode != DMA_NORMAL) {
        return false;
    }

    /* 先校验目标 huart 是否被其它上下文占用，避免拆掉旧绑定后才失败 */
    owner = hal_find_by_huart(huart);
    if (owner != NULL && owner != h) {
        return false;
    }

    /* 清理同一 h 的旧绑定（重入/换串口场景） */
    if (h->huart != NULL) {
        hal_unregister(h);
        if (h->huart->RxState != HAL_UART_STATE_READY) {
            (void)HAL_UART_AbortReceive(h->huart);
        }
        if ((h->huart->gState == HAL_UART_STATE_BUSY_TX) ||
            (h->huart->gState == HAL_UART_STATE_BUSY_TX_RX)) {
            (void)HAL_UART_AbortTransmit(h->huart);
        }
        if ((h->proto != NULL) && (h->proto != proto)) {
            Protocol_RegisterSender(h->proto, NULL, NULL);
        }
    }

    h->huart           = huart;
    h->proto           = proto;
    h->tx_busy         = false;
    h->tx_starting     = false;
    h->rx_active       = false;
    h->rx_need_restart = false;
    h->tx_head         = 0U;
    h->tx_tail         = 0U;
    h->tx_drop_count   = 0U;
    h->tx_error_count  = 0U;
    h->tx_start_tick   = 0U;

    if (!hal_register(h)) {
        h->huart = NULL;
        h->proto = NULL;
        return false;
    }

    /* 顺序：先初始化协议层 + 注册发送器，再启动 DMA 接收。
     * 若反过来，StartRx 成功后到 Init 之间到达的数据会被 Init 的 rx_len=0 清空，
     * 且该窗口内 send_func 为 NULL，Auto-Reply 也会失败。 */
    Protocol_Init(proto);
    Protocol_RegisterSender(proto, Protocol_HAL_Send, h);

    if (!hal_start_rx(h)) {
        hal_unregister(h);
        h->huart = NULL;
        h->proto = NULL;
        return false;
    }
    return true;
}

/* ==================== 接收事件 ====================
 * v1.7 顺序：先把本批数据拷到 rx_pending → 立刻重启 RX → 再解析/回调。
 * 这样 handler 执行期间 UART 仍在接收，避免默认（handler 在 ISR）模式下的丢字节。
 * 回调之间不会重入（同一 UART 中断不会抢占自身），rx_pending 可安全复用。
 */
void Protocol_HAL_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    Protocol_HAL_Context_t *h = hal_find_by_huart(huart);
    bool half_transfer;

    if (h == NULL) return;

    half_transfer = (huart->RxEventType == HAL_UART_RXEVENT_HT);
    if ((size > 0U) && ((size_t)size > sizeof(h->rx_dma_buf))) {
        size = 0U;   /* 防御：HAL 不应给出超长 size */
    }

    if (size > 0U) {
        memcpy(h->rx_pending, h->rx_dma_buf, size);
    }

    if (!half_transfer) {
        h->rx_active = false;
        (void)hal_start_rx(h);
    }

    if ((size > 0U) && (h->proto != NULL)) {
        Protocol_Parse(h->proto, h->rx_pending, size);
    }
}

/* ==================== 发送完成 ==================== */
void Protocol_HAL_TxCpltCallback(UART_HandleTypeDef *huart) {
    Protocol_HAL_Context_t *h = hal_find_by_huart(huart);
    uint32_t primask;

    if (h == NULL) return;

    primask = __get_PRIMASK();
    __disable_irq();
    h->tx_busy = false;
    h->tx_starting = false;
    __set_PRIMASK(primask);

    hal_tx_pump(h);
}

/* ==================== 错误回调 ====================
 * 区分 RX 行错误 / DMA 错误，避免“TX 错误顺手把 RX 也重启”。
 *   - RX 行错误（PE/NE/FE/ORE）：HAL 已结束接收，只置 RX 重启标志，不动 TX；
 *   - DMA 错误：HAL 可能同时结束 TX 与 RX，分别恢复；
 *   - TX 错误：清 busy、计数、泵送下一帧。
 */
void Protocol_HAL_ErrorCallback(UART_HandleTypeDef *huart) {
    Protocol_HAL_Context_t *h = hal_find_by_huart(huart);
    const uint32_t err = huart->ErrorCode;
    const bool dma_err = ((err & HAL_UART_ERROR_DMA) != 0U);
    const bool line_err = ((err & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                                   HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE)) != 0U);
    uint32_t primask;

    if (h == NULL) return;

    if (dma_err) {
        if (h->tx_busy && !h->tx_starting) {
            (void)HAL_UART_AbortTransmit(huart);
            primask = __get_PRIMASK();
            __disable_irq();
            h->tx_busy = false;
            h->tx_starting = false;
            h->tx_error_count++;
            __set_PRIMASK(primask);
            hal_tx_pump(h);
        }
        if (h->rx_active) {
            h->rx_active = false;
            h->rx_need_restart = true;
        }
    } else if (line_err) {
        h->rx_active = false;
        h->rx_need_restart = true;
    }

#if PROTOCOL_HAL_AUTO_RESTART_RX
    if (h->rx_need_restart) {
        (void)Protocol_HAL_RestartRx(h);
    }
#endif
}

/* ==================== 主循环：延迟恢复 ==================== */
#if PROTOCOL_HAL_TX_TIMEOUT_MS > 0
static uint32_t hal_tx_timeout_ms(const Protocol_HAL_Context_t *h) {
    uint32_t baud = 0U;
    uint32_t expected_ms;
    uint32_t timeout;

    if ((h != NULL) && (h->huart != NULL)) {
        baud = (uint32_t)h->huart->Init.BaudRate;
    }

    /* 按最长帧估算：每字节 10 bit（8N1） */
    expected_ms = (baud > 0U)
                ? ((uint32_t)PROTOCOL_HAL_FRAME_MAX * 10U * 1000U / baud)
                : 0U;
    timeout = expected_ms + 20U;

    if (timeout < (uint32_t)PROTOCOL_HAL_TX_TIMEOUT_MS) {
        timeout = (uint32_t)PROTOCOL_HAL_TX_TIMEOUT_MS;
    }
    return timeout;
}
#endif

void Protocol_HAL_Process(void) {
    for (int i = 0; i < PROTOCOL_HAL_MAX_INSTANCES; i++) {
        Protocol_HAL_Context_t *h = g_hal_table[i];
        if (h == NULL) continue;

        if (h->rx_need_restart) {
            (void)Protocol_HAL_RestartRx(h);
        }

#if PROTOCOL_HAL_TX_TIMEOUT_MS > 0
        if (h->tx_busy && !h->tx_starting) {
            const uint32_t elapsed = (uint32_t)(HAL_GetTick() - h->tx_start_tick);
            if (elapsed > hal_tx_timeout_ms(h)) {
                uint32_t primask;
                (void)HAL_UART_AbortTransmit(h->huart);
                primask = __get_PRIMASK();
                __disable_irq();
                h->tx_busy = false;
                h->tx_starting = false;
                h->tx_error_count++;
                __set_PRIMASK(primask);
            }
        }
#endif

        if (!h->tx_busy) hal_tx_pump(h);
    }
}

/* ==================== 手动重新挂起接收 ==================== */
bool Protocol_HAL_RestartRx(Protocol_HAL_Context_t *h) {
    if (h == NULL || h->huart == NULL) return false;

    h->rx_active = false;
    if (h->huart->RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(h->huart);
    }
    return hal_start_rx(h);
}

/* ==================== 解绑 ==================== */
bool Protocol_HAL_Deinit(Protocol_HAL_Context_t *h) {
    if (h == NULL) return false;

    hal_unregister(h);

    if (h->huart != NULL) {
        if (h->huart->RxState != HAL_UART_STATE_READY) {
            (void)HAL_UART_AbortReceive(h->huart);
        }
        if ((h->huart->gState == HAL_UART_STATE_BUSY_TX) ||
            (h->huart->gState == HAL_UART_STATE_BUSY_TX_RX)) {
            (void)HAL_UART_AbortTransmit(h->huart);
        }
    }
    if (h->proto != NULL) {
        Protocol_RegisterSender(h->proto, NULL, NULL);
    }

    h->huart           = NULL;
    h->proto           = NULL;
    h->tx_busy         = false;
    h->tx_starting     = false;
    h->rx_active       = false;
    h->rx_need_restart = false;
    h->tx_head         = 0U;
    h->tx_tail         = 0U;
    h->tx_drop_count   = 0U;
    h->tx_error_count  = 0U;
    h->tx_start_tick   = 0U;
    return true;
}

uint32_t Protocol_HAL_GetTxDropCount(const Protocol_HAL_Context_t *h) {
    return (h != NULL) ? h->tx_drop_count : 0U;
}

uint32_t Protocol_HAL_GetTxErrorCount(const Protocol_HAL_Context_t *h) {
    return (h != NULL) ? h->tx_error_count : 0U;
}
