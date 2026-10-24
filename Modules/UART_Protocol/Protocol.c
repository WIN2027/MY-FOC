#include "Protocol.h"
#include <string.h>

/* ==================== 编译器内存屏障 ====================
 * 用于延迟分发环形队列的释放/获取语义，以及 handler 表写序约束。
 * 强序架构（Cortex-M 单核）配合 volatile 已足够；
 * 弱序/SMP 平台建议重定义为硬件屏障（如 CMSIS 的 __DMB()）。
 */
#ifndef PROTOCOL_COMPILER_BARRIER
  #if defined(__GNUC__) || defined(__clang__)
    #define PROTOCOL_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
  #else
    #define PROTOCOL_COMPILER_BARRIER() ((void)0)
  #endif
#endif

#if PROTOCOL_DEFER_HANDLER
#define PROTOCOL_QUEUE_MASK ((uint16_t)(PROTOCOL_QUEUE_SIZE - 1))
#endif

/* ==================== 原子错误计数辅助 ====================
 * 统一递增/读取/清零语义，消除"清零原子、递增不原子"的不一致。
 * 所有 err_count 访问必须走下列三个内联函数。
 */
static inline void proto_err_inc(Protocol_Context_t *ctx, Protocol_ErrorType_t type) {
#if defined(__GNUC__) || defined(__clang__)
    (void)__atomic_add_fetch((uint32_t *)&ctx->err_count[type], 1u, __ATOMIC_RELAXED);
#else
    ctx->err_count[type]++;
#endif
}

static inline uint32_t proto_err_read(Protocol_Context_t *ctx, Protocol_ErrorType_t type) {
#if defined(__GNUC__) || defined(__clang__)
    return __atomic_load_n((uint32_t *)&ctx->err_count[type], __ATOMIC_RELAXED);
#else
    return ctx->err_count[type];
#endif
}

/* ==================== CRC8 ==================== */
static uint8_t CRC8_Calc(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ==================== 延迟分发：入队 ==================== */
#if PROTOCOL_DEFER_HANDLER
static void enqueue_frame(Protocol_Context_t *ctx, uint8_t cmd,
                          const uint8_t *data, uint8_t len) {
    uint16_t next = (uint16_t)((ctx->q_head + 1) & PROTOCOL_QUEUE_MASK);
    if (next == ctx->q_tail) {
        proto_err_inc(ctx, PROTOCOL_ERR_QUEUE_FULL);   /* 队列满，丢弃 */
        return;
    }
    ctx->frame_queue[ctx->q_head].cmd = cmd;
    ctx->frame_queue[ctx->q_head].len = len;
    if (len > 0) memcpy(ctx->frame_queue[ctx->q_head].data, data, len);
    PROTOCOL_COMPILER_BARRIER();
    ctx->q_head = next;
}
#endif

/* ==================== 内部：分发 ==================== */
static bool invoke_handler(Protocol_Context_t *ctx, uint8_t cmd,
                           const uint8_t *data, uint8_t len) {
    for (int i = 0; i < PROTOCOL_MAX_HANDLERS; i++) {
        if (ctx->handlers[i].enabled && ctx->handlers[i].cmd == cmd) {
            /* 防御性再读一次 handler：配合 volatile + 注册/注销写序，
             * 理论上不会为 NULL；此处兜底避免极端平台下的空指针调用。 */
            Protocol_Handler_t h = ctx->handlers[i].handler;
            if (h != NULL) {
                h(cmd, data, len);
                return true;
            }
        }
    }
    proto_err_inc(ctx, PROTOCOL_ERR_UNKNOWN_CMD);
    return false;
}

/* ==================== 内部：分发一帧 ====================
 * 两条路径（Parse 直发 / Process 延迟）共用，避免自动应答逻辑分叉。
 */
static void dispatch_frame(Protocol_Context_t *ctx, uint8_t cmd,
                           const uint8_t *data, uint8_t len) {
    const bool handled = invoke_handler(ctx, cmd, data, len);

#if PROTOCOL_AUTO_REPLY_ERROR
    /* 只对业务命令(0x00~0x7F)回错误，避免协议层错误帧乒乓 */
    if (!handled && (cmd & 0x80U) == 0U) {
        (void)Protocol_Send(ctx, PROTOCOL_CMD_ERR_INVALID, NULL, 0);
    }
#else
    (void)handled;
#endif
}

/* ==================== 初始化 ==================== */
void Protocol_Init(Protocol_Context_t *ctx) {
    if (ctx == NULL) return;

    for (int i = 0; i < PROTOCOL_MAX_HANDLERS; i++) {
        ctx->handlers[i].cmd     = 0;
        ctx->handlers[i].handler = NULL;
        ctx->handlers[i].enabled = false;
    }
    ctx->send_func = NULL;
    ctx->send_user = NULL;
    ctx->rx_len    = 0;
    for (int i = 0; i < PROTOCOL_ERR__COUNT; i++) {
        ctx->err_count[i] = 0;
    }
#if PROTOCOL_DEFER_HANDLER
    ctx->q_head = 0;
    ctx->q_tail = 0;
#endif
}

void Protocol_Reset(Protocol_Context_t *ctx) {
    if (ctx == NULL) return;
    ctx->rx_len = 0;
#if PROTOCOL_DEFER_HANDLER
    /* 队列在 SPSC 约束下安全复位：需在通信停止时调用 */
    ctx->q_head = 0;
    ctx->q_tail = 0;
#endif
}

void Protocol_RegisterSender(Protocol_Context_t *ctx,
                             Protocol_SendFunc_t send_func, void *user) {
    if (ctx == NULL) return;
    /* 假设运行期不切换 sender；若需切换请在通信停止时调用 */
    ctx->send_func = send_func;
    ctx->send_user = user;
}

/* ==================== 注册/注销 ==================== */
bool Protocol_RegisterHandler(Protocol_Context_t *ctx, uint8_t cmd,
                              Protocol_Handler_t handler) {
    if (ctx == NULL || handler == NULL) return false;

    bool ok = false;
    PROTOCOL_LOCK_HANDLERS();

    /* 优先匹配已有条目（更新） */
    for (int i = 0; i < PROTOCOL_MAX_HANDLERS; i++) {
        if (ctx->handlers[i].enabled && ctx->handlers[i].cmd == cmd) {
            ctx->handlers[i].handler = handler;
            /* 屏障 + 写序：handler 先写，enabled 再置（本分支 enabled 已为 true，
             * 无需重写，屏障保证 handler 对中断可见） */
            PROTOCOL_COMPILER_BARRIER();
            ok = true;
            break;
        }
    }
    /* 未匹配到则分配空闲槽 */
    if (!ok) {
        for (int i = 0; i < PROTOCOL_MAX_HANDLERS; i++) {
            if (!ctx->handlers[i].enabled) {
                ctx->handlers[i].cmd     = cmd;
                ctx->handlers[i].handler = handler;
                /* 写序约束：cmd/handler 写完后再置 enabled=true */
                PROTOCOL_COMPILER_BARRIER();
                ctx->handlers[i].enabled = true;
                ok = true;
                break;
            }
        }
    }

    PROTOCOL_UNLOCK_HANDLERS();
    return ok;
}

bool Protocol_UnregisterHandler(Protocol_Context_t *ctx, uint8_t cmd) {
    if (ctx == NULL) return false;

    bool ok = false;
    PROTOCOL_LOCK_HANDLERS();
    for (int i = 0; i < PROTOCOL_MAX_HANDLERS; i++) {
        if (ctx->handlers[i].enabled && ctx->handlers[i].cmd == cmd) {
            /* 写序约束：先清 enabled，再清 handler */
            ctx->handlers[i].enabled = false;
            PROTOCOL_COMPILER_BARRIER();
            ctx->handlers[i].handler = NULL;
            ok = true;
            break;
        }
    }
    PROTOCOL_UNLOCK_HANDLERS();
    return ok;
}

/* ==================== 发送 ==================== */
bool Protocol_Send(Protocol_Context_t *ctx, uint8_t cmd,
                   const uint8_t *data, uint8_t len) {
    if (ctx == NULL)                  return false;
    if (ctx->send_func == NULL)       return false;
    if (len > PROTOCOL_MAX_PAYLOAD)   return false;
    if (len > 0 && data == NULL)      return false;

    uint8_t frame[PROTOCOL_MAX_PAYLOAD + PROTOCOL_FRAME_OVERHEAD];
    frame[0] = PROTOCOL_SOF;
    frame[1] = cmd;
    frame[2] = len;
    if (len > 0) memcpy(&frame[3], data, len);
    frame[3 + len] = CRC8_Calc(&frame[1], (uint16_t)(2 + len));

    /* frame 位于栈上；SendFunc 实现必须在返回 true 之前完成数据拷贝 */
    return ctx->send_func(ctx->send_user, frame,
                          (uint16_t)(PROTOCOL_FRAME_OVERHEAD + len));
}

/* ==================== 延迟分发：出队 ==================== */
#if PROTOCOL_DEFER_HANDLER
void Protocol_Process(Protocol_Context_t *ctx) {
    if (ctx == NULL) return;

    while (ctx->q_tail != ctx->q_head) {
        /* 获取屏障：保证先读到 head 的最新值，再读帧内容 */
        PROTOCOL_COMPILER_BARRIER();
        Protocol_Frame_t *f = &ctx->frame_queue[ctx->q_tail];
        dispatch_frame(ctx, f->cmd, f->data, f->len);
        ctx->q_tail = (uint16_t)((ctx->q_tail + 1) & PROTOCOL_QUEUE_MASK);
    }
}
#endif

/* ==================== 解析 ==================== */
void Protocol_Parse(Protocol_Context_t *ctx, const uint8_t *buf, uint16_t len) {
    if (ctx == NULL || buf == NULL || len == 0) return;

    /* ---------- 1. 追加到内部缓冲 ---------- */
    if (len > PROTOCOL_RX_BUFFER_SIZE) {
        proto_err_inc(ctx, PROTOCOL_ERR_RX_OVERFLOW);
        ctx->rx_len = 0;
        return;
    }

    if ((uint32_t)ctx->rx_len + len > PROTOCOL_RX_BUFFER_SIZE) {
        proto_err_inc(ctx, PROTOCOL_ERR_RX_OVERFLOW);

        /* 从尾部回溯，找最后一个 SOF 作为保留起点 */
        uint16_t keep_from = ctx->rx_len;
        for (uint16_t i = ctx->rx_len; i > 0; i--) {
            if (ctx->rx_buffer[i - 1] == PROTOCOL_SOF) {
                keep_from = (uint16_t)(i - 1);
                break;
            }
        }
        uint16_t keep_len = (uint16_t)(ctx->rx_len - keep_from);

        if ((uint32_t)keep_len + len > PROTOCOL_RX_BUFFER_SIZE) {
            keep_from = ctx->rx_len;
            keep_len  = 0;
        }
        if (keep_from > 0 && keep_len > 0) {
            memmove(ctx->rx_buffer, &ctx->rx_buffer[keep_from], keep_len);
        }
        ctx->rx_len = keep_len;
    }

    memcpy(&ctx->rx_buffer[ctx->rx_len], buf, len);
    ctx->rx_len = (uint16_t)(ctx->rx_len + len);

    /* ---------- 2. 逐帧解析 ---------- */
    uint16_t offset = 0;
    while (offset < ctx->rx_len) {
        while (offset < ctx->rx_len && ctx->rx_buffer[offset] != PROTOCOL_SOF) offset++;
        if (ctx->rx_len - offset < PROTOCOL_FRAME_OVERHEAD) break;

        uint8_t cmd      = ctx->rx_buffer[offset + 1];
        uint8_t data_len = ctx->rx_buffer[offset + 2];

        if (data_len > PROTOCOL_MAX_PAYLOAD) {
            proto_err_inc(ctx, PROTOCOL_ERR_INVALID_LEN);
            offset++;
            continue;
        }

        uint16_t expected = (uint16_t)(PROTOCOL_FRAME_OVERHEAD + data_len);
        if (ctx->rx_len - offset < expected) break;   /* 半包，等下次 */

        uint8_t calc = CRC8_Calc(&ctx->rx_buffer[offset + 1],
                                 (uint16_t)(2 + data_len));
        uint8_t recv = ctx->rx_buffer[offset + 3 + data_len];

        if (calc == recv) {
#if PROTOCOL_DEFER_HANDLER
            enqueue_frame(ctx, cmd, &ctx->rx_buffer[offset + 3], data_len);
#else
            dispatch_frame(ctx, cmd, &ctx->rx_buffer[offset + 3], data_len);
#endif
            offset += expected;
        } else {
            proto_err_inc(ctx, PROTOCOL_ERR_CRC);
            /* 注意：CRC 失败时**不回复错误帧**。
             * CRC 失败表明整帧（含 CMD/LEN）均不可信，回帧可能对伪 SOF 后的
             * 垃圾数据反复触发，反而加剧总线负担。只做本机计数 + 重同步。 */
            offset++;
        }
    }

    /* ---------- 3. 保留未解析数据 ---------- */
    if (offset > 0) {
        uint16_t remain = (uint16_t)(ctx->rx_len - offset);
        if (remain > 0) memmove(ctx->rx_buffer, &ctx->rx_buffer[offset], remain);
        ctx->rx_len = remain;
    }
}

/* ==================== 错误统计 ==================== */
uint32_t Protocol_GetErrorCount(Protocol_Context_t *ctx, Protocol_ErrorType_t type) {
    if (ctx == NULL) return 0;
    if ((unsigned)type >= (unsigned)PROTOCOL_ERR__COUNT) return 0;
    return proto_err_read(ctx, type);
}

void Protocol_ResetErrorCounts(Protocol_Context_t *ctx) {
    if (ctx == NULL) return;
    for (int i = 0; i < PROTOCOL_ERR__COUNT; i++) {
#if defined(__GNUC__) || defined(__clang__)
        /* 原子写：避免与中断中的 err_count[i]++ 竞争导致部分清零或丢失递增 */
        __atomic_store_n((uint32_t *)&ctx->err_count[i], 0, __ATOMIC_RELAXED);
#else
        ctx->err_count[i] = 0;
#endif
    }
}