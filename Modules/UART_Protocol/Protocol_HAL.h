#ifndef PROTOCOL_HAL_H_INCLUDED
#define PROTOCOL_HAL_H_INCLUDED

#include "Protocol.h"

/* HAL 头文件可配置：
 * - 默认包含 CubeMX 工程的 main.h（其内部包含正确的 stm32xxxx_hal.h）
 * - 若工程结构不同，可在编译选项中定义：
 *     -DPROTOCOL_HAL_HEADER='"stm32h7xx_hal.h"'
 */
#ifndef PROTOCOL_HAL_HEADER
#define PROTOCOL_HAL_HEADER "main.h"
#endif
#include PROTOCOL_HAL_HEADER

/* 单批 DMA 接收缓冲。必须 <= PROTOCOL_RX_BUFFER_SIZE，
 * 否则 Protocol_Parse 会整批拒绝并清零（见 Protocol.c 的 RX_OVERFLOW 分支）。
 * 上限 0xFFFF 因 HAL_UARTEx_ReceiveToIdle_DMA 的 size 形参为 uint16_t。 */
#ifndef PROTOCOL_HAL_RX_BUF_SIZE
#define PROTOCOL_HAL_RX_BUF_SIZE 128
#endif

/* 可同时初始化的串口实例上限 */
#ifndef PROTOCOL_HAL_MAX_INSTANCES
#define PROTOCOL_HAL_MAX_INSTANCES 4
#endif

/* 发送环深度（含 1 个空槽用于区分满/空，实际可排队 DEPTH-1 帧）。
 * 每槽 68 B；DEPTH=4 约 272 B RAM。最小 2（=1 帧排队）。 */
#ifndef PROTOCOL_HAL_TX_QUEUE_DEPTH
#define PROTOCOL_HAL_TX_QUEUE_DEPTH 4
#endif

/* 发送超时下限（ms）：tx_busy 持续超过该时间视为卡死，主循环强制 Abort 并恢复。
 * 实际超时 = max(本值, 按波特率估算的整帧时间 + 20 ms)，低速波特率自动放宽。
 * 0 表示关闭超时检测。 */
#ifndef PROTOCOL_HAL_TX_TIMEOUT_MS
#define PROTOCOL_HAL_TX_TIMEOUT_MS 50
#endif

/* 1：在 UART 错误回调（ISR）里直接 Abort+重启 RX；0：只置标志，由
 * Protocol_HAL_Process 恢复。默认 0，避免在 ISR 里执行较长的 HAL Abort。 */
#ifndef PROTOCOL_HAL_AUTO_RESTART_RX
#define PROTOCOL_HAL_AUTO_RESTART_RX 0
#endif

/* 单帧最大长度：SOF+CMD+LEN+DATA+CRC */
#define PROTOCOL_HAL_FRAME_MAX (PROTOCOL_MAX_PAYLOAD + PROTOCOL_FRAME_OVERHEAD)

/* ==================== HAL 上下文 ====================
 * 每个 UART 实例一个，与一个 Protocol_Context_t 绑定。
 * 由用户静态分配（不要放栈上），在 Protocol_HAL_Init 时填入 huart/proto。
 *
 * 内存放置（重要）：本结构体含 DMA 收发缓冲，必须放在 DMA 可访问的 SRAM。
 * STM32G4 的 CCM SRAM（0x1000_0000）DMA 不可访问，不能把本结构体放进 CCM。
 * 例：
 *     __attribute__((section(".sram_dma"))) Protocol_HAL_Context_t g_hal1;
 */
typedef struct {
    UART_HandleTypeDef *huart;
    Protocol_Context_t *proto;

    uint8_t  rx_dma_buf[PROTOCOL_HAL_RX_BUF_SIZE];
    uint8_t  rx_pending[PROTOCOL_HAL_RX_BUF_SIZE];  /* 先拷出再重启 RX，避免 handler 期间丢字节 */
    uint8_t  tx_buf[PROTOCOL_HAL_FRAME_MAX];

    uint8_t  tx_queue[PROTOCOL_HAL_TX_QUEUE_DEPTH][PROTOCOL_HAL_FRAME_MAX];
    uint16_t tx_head;
    uint16_t tx_tail;

    volatile bool tx_busy;
    volatile bool tx_starting;      /* HAL_UART_Transmit_DMA 正在启动（防错误中断重入） */
    volatile bool rx_active;        /* RX DMA 正在挂起（错误回调判断来源） */
    volatile bool rx_need_restart;  /* RX 错误后置位，由主循环 Protocol_HAL_Process 处理 */

    volatile uint32_t tx_drop_count;   /* 队列满丢弃的帧数 */
    volatile uint32_t tx_error_count;  /* HAL 启动失败 / TX DMA 错误 / 超时次数 */

    uint32_t tx_start_tick;            /* HAL_GetTick() 时间戳，用于 TX 超时 */
} Protocol_HAL_Context_t;

/**
 * @brief 绑定 UART 与协议上下文，并以 DMA + IDLE 方式启动接收
 * @param h      HAL 上下文（用户静态分配，必须在 DMA 可访问 SRAM）
 * @param proto  绑定的协议上下文（用户静态分配，由本函数调用 Protocol_Init 初始化）
 * @param huart  CubeMX 初始化好的 UART 句柄
 * @return true 成功
 *
 * @warning 内部会调用 Protocol_Init(proto)，**清空该 proto 的 handler 表**，
 *          请在本函数成功返回后再注册业务命令。
 * @warning 本层仅支持 RX DMA 为 **Normal** 模式；Circular 直接返回 false。
 * @warning 必须同时配置 **TX DMA（Normal）**（发送环使用 HAL_UART_Transmit_DMA）；缺少 hdmatx 或为 Circular 返回 false。
 * @warning 执行顺序为 Protocol_Init → RegisterSender → StartRx。
 *          副作用：若 StartRx 失败，proto 的 handler 表已被清空，用户需重新
 *          Init + 注册。
 * @note 支持重复调用（同一 h 再次 Init 会先 Abort 旧绑定再重启）。
 * @note 若目标 huart 已被其它 HAL 上下文占用，会在拆旧绑定之前返回 false。
 * @note 依赖 HAL_UARTEx_ReceiveToIdle_DMA（HAL 1.13+）。
 */
bool Protocol_HAL_Init(Protocol_HAL_Context_t *h,
                       Protocol_Context_t *proto,
                       UART_HandleTypeDef *huart);

/* 以下三个在用户 HAL 回调中转发调用 */
void Protocol_HAL_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void Protocol_HAL_TxCpltCallback(UART_HandleTypeDef *huart);
void Protocol_HAL_ErrorCallback(UART_HandleTypeDef *huart);

/**
 * @brief 主循环调用：处理 RX 错误后的延迟恢复、TX 超时恢复、发送环泵送
 * @note 若你的工程转发了 HAL_UART_ErrorCallback，**必须**在主循环定期调用本函数，
 *        否则接收错误后 RX 不会自动重启。
 *        如果不想依赖主循环，也可以手动调用 Protocol_HAL_RestartRx(&h)。
 */
void Protocol_HAL_Process(void);

/**
 * @brief 手动重新挂起接收（不清空 handler 表）
 */
bool Protocol_HAL_RestartRx(Protocol_HAL_Context_t *h);

/**
 * @brief 解绑：Abort RX/TX、从实例表注销、清空 proto 的发送器
 * @note 解绑后 h 可再次用于 Protocol_HAL_Init。
 */
bool Protocol_HAL_Deinit(Protocol_HAL_Context_t *h);

/**
 * @brief 读取发送丢弃计数（队列满）
 */
uint32_t Protocol_HAL_GetTxDropCount(const Protocol_HAL_Context_t *h);

/**
 * @brief 读取发送错误计数（HAL 启动失败 / TX DMA 错误 / 超时）
 */
uint32_t Protocol_HAL_GetTxErrorCount(const Protocol_HAL_Context_t *h);

#endif /* PROTOCOL_HAL_H_INCLUDED */
