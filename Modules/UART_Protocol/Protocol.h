#ifndef PROTOCOL_H_INCLUDED
#define PROTOCOL_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

/* ==================== 协议常量 ==================== */
#define PROTOCOL_SOF            0x5A
#define PROTOCOL_MAX_HANDLERS   32
#define PROTOCOL_MAX_PAYLOAD    64

/* 帧开销：SOF(1) + CMD(1) + LEN(1) + CRC(1) = 4 字节 */
#define PROTOCOL_FRAME_OVERHEAD 4

#ifndef PROTOCOL_RX_BUFFER_SIZE
#define PROTOCOL_RX_BUFFER_SIZE 256
#endif

#ifndef PROTOCOL_AUTO_REPLY_ERROR
#define PROTOCOL_AUTO_REPLY_ERROR 1
#endif

#ifndef PROTOCOL_DEFER_HANDLER
#define PROTOCOL_DEFER_HANDLER 0
#endif

#if PROTOCOL_DEFER_HANDLER
#ifndef PROTOCOL_QUEUE_SIZE
#define PROTOCOL_QUEUE_SIZE 8
#endif
#endif

/* 命令空间划分：0x00~0x7F 业务命令；0x80~0xFF 协议层保留 */
#define PROTOCOL_CMD_ACK_OK          0x80
#define PROTOCOL_CMD_ERR_CRC         0x81
#define PROTOCOL_CMD_ERR_INVALID     0x82

/* ==================== 编译期检查 ==================== */
#if defined(__cplusplus)
  #define PROTOCOL_SA(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
  #define PROTOCOL_SA(cond, msg) _Static_assert(cond, msg)
#else
  /* C99 兜底：用负长度数组在编译期报错，避免宏为空导致检查静默失效 */
  #define PROTOCOL_SA_CAT_(a, b) a##b
  #define PROTOCOL_SA_CAT(a, b) PROTOCOL_SA_CAT_(a, b)
  #define PROTOCOL_SA(cond, msg) \
      typedef char PROTOCOL_SA_CAT(protocol_sa_, __LINE__)[(cond) ? 1 : -1]
#endif

PROTOCOL_SA(PROTOCOL_MAX_PAYLOAD <= 253,
            "PROTOCOL_MAX_PAYLOAD too large: 2+LEN must fit in uint8_t CRC length");
PROTOCOL_SA(PROTOCOL_RX_BUFFER_SIZE >= PROTOCOL_MAX_PAYLOAD + PROTOCOL_FRAME_OVERHEAD,
            "PROTOCOL_RX_BUFFER_SIZE too small for one max frame");
PROTOCOL_SA(PROTOCOL_RX_BUFFER_SIZE <= 0xFFFF,
            "PROTOCOL_RX_BUFFER_SIZE must be <= 65535 (rx_len is uint16_t)");
PROTOCOL_SA(PROTOCOL_MAX_HANDLERS > 0 && PROTOCOL_MAX_HANDLERS <= 255,
            "PROTOCOL_MAX_HANDLERS must be in (0, 255]");
PROTOCOL_SA(PROTOCOL_FRAME_OVERHEAD == 4,
            "PROTOCOL_FRAME_OVERHEAD must be 4 (SOF+CMD+LEN+CRC)");
#if PROTOCOL_DEFER_HANDLER
PROTOCOL_SA(PROTOCOL_QUEUE_SIZE >= 2 &&
            (PROTOCOL_QUEUE_SIZE & (PROTOCOL_QUEUE_SIZE - 1)) == 0,
            "PROTOCOL_QUEUE_SIZE must be power of 2 and >= 2");
#endif

/* ==================== handler 表并发保护（可选） ====================
 * Protocol_Parse 可能在中断上下文遍历 handlers 表；
 * 若运行期调用 Protocol_RegisterHandler / Protocol_UnregisterHandler，
 * 需保证表操作的原子性。默认无保护（依赖初始化阶段一次性注册完毕）。
 *
 * 注意（多实例）：这两个宏作用于**整个模块**，是一把全局锁，
 * 保护所有 Protocol_Context_t 的 handler 表。同一时刻只会有一个注册
 * 动作，因此全局锁即可。
 *
 * 示例（Cortex-M / CMSIS，定义在编译选项或工程全局头文件中）：
 *   #define PROTOCOL_LOCK_HANDLERS()                                    \
 *       uint32_t _protocol_pm_ = __get_PRIMASK(); __disable_irq()
 *   #define PROTOCOL_UNLOCK_HANDLERS()                                  \
 *       __set_PRIMASK(_protocol_pm_)
 *
 * 两个宏必须在同一作用域内成对使用；不要用 do{...}while(0) 包裹。
 * 若目标工程为 C89 严格模式（-Wdeclaration-after-statement），需在函数体
 * 语句末尾调用 LOCK（即函数体只允许前面有声明，LOCK 之后不再出现新声明）。
 */
#ifndef PROTOCOL_LOCK_HANDLERS
  #define PROTOCOL_LOCK_HANDLERS()   ((void)0)
#endif
#ifndef PROTOCOL_UNLOCK_HANDLERS
  #define PROTOCOL_UNLOCK_HANDLERS() ((void)0)
#endif

/* ==================== 错误统计类型 ==================== */
typedef enum {
    PROTOCOL_ERR_CRC          = 0,   /* CRC 校验失败 */
    PROTOCOL_ERR_INVALID_LEN  = 1,   /* LEN > PROTOCOL_MAX_PAYLOAD */
    PROTOCOL_ERR_UNKNOWN_CMD  = 2,   /* 未注册 handler 的命令 */
    PROTOCOL_ERR_QUEUE_FULL   = 3,   /* 延迟分发队列满（仅 DEFER 模式） */
    PROTOCOL_ERR_RX_OVERFLOW  = 4,   /* 接收缓冲溢出（半包被截断或丢弃） */
    PROTOCOL_ERR__COUNT       = 5    /* 内部哨兵，用户不要使用 */
} Protocol_ErrorType_t;

/* ==================== 回调类型 ==================== */
/**
 * @brief 命令处理回调
 * @param cmd  命令码
 * @param data 指向数据域，**仅在回调执行期间有效**，禁止保存指针到回调外使用
 * @param len  数据长度
 *
 * @note 默认(PROTOCOL_DEFER_HANDLER=0)：handler 在 Protocol_Parse 的调用上下文中执行，
 *       通常即为 UART 中断。handler 必须短小、非阻塞，禁止 HAL_Delay / 打印 / 长耗时逻辑；
 *       建议只做状态置位或入队，复杂处理交给主循环。
 * @note 开启 PROTOCOL_DEFER_HANDLER 后：handler 在 Protocol_Process() 的调用上下文中执行，
 *       与中断解耦，可执行较长逻辑。
 */
typedef void (*Protocol_Handler_t)(uint8_t cmd, const uint8_t *data, uint8_t len);

/**
 * @brief 底层发送函数
 * @param user 注册时传入的用户上下文（典型用法：指向 Protocol_HAL_Context_t）
 * @param data 待发送数据
 * @param len  长度
 * @return true 表示底层已接受（不等于已发出）
 *
 * @warning `data` 指向 **Protocol_Send 内部的栈缓冲**，本函数返回后立即失效。
 *          实现方必须在返回 true 之前把数据拷贝到自管理内存（或同步发送完毕），
 *          禁止把 `data` 保存下来供 DMA / 中断异步使用。
 */
typedef bool (*Protocol_SendFunc_t)(void *user, const uint8_t *data, uint16_t len);

/* ==================== 内部类型（暴露在头文件中供 context 使用） ==================== */
typedef struct {
    uint8_t cmd;
    Protocol_Handler_t handler;
    bool enabled;
} Protocol_HandlerEntry_t;

#if PROTOCOL_DEFER_HANDLER
typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[PROTOCOL_MAX_PAYLOAD];
} Protocol_Frame_t;
#endif

/* ==================== 协议上下文 ====================
 * 每个 UART 实例一个 Protocol_Context_t，彼此完全隔离：
 *   - handler 表、接收缓冲、发送回调、错误计数、延迟队列均独立。
 *   - 同一命令码在不同 context 上可以注册不同的 handler。
 *
 * 内存占用（实测，GCC 13 / Cortex-M4）：
 *   - DEFER_HANDLER=0：约 672 B / 实例；
 *   - DEFER_HANDLER=1：约 1.2 KB / 实例（含 8 帧延迟队列）。
 *   不含 Protocol_HAL_Context_t（约 624 B，含 128 B RX + 128 B 先拷缓冲 + 4×68 B 发送环）。
 */
typedef struct {
    /* handler 表：volatile 保证中断与主循环并发读写时每次访问都读内存，
     * 配合注册/注销的写序约束，避免"看到 enabled=true 但 handler 仍为 NULL"的窗口 */
    volatile Protocol_HandlerEntry_t handlers[PROTOCOL_MAX_HANDLERS];

    Protocol_SendFunc_t send_func;
    void               *send_user;

    /* 跨调用保留的半包缓冲 */
    uint8_t  rx_buffer[PROTOCOL_RX_BUFFER_SIZE];
    uint16_t rx_len;

    /* 错误统计（中断中递增，主循环读取）。
     * 所有递增均使用 __atomic_add_fetch，读取使用 __atomic_load_n，
     * 清零使用 __atomic_store_n，保证三端并发安全（GCC/Clang 下有效）。 */
    volatile uint32_t err_count[PROTOCOL_ERR__COUNT];

#if PROTOCOL_DEFER_HANDLER
    /* 延迟分发环形队列：SPSC（中断写 head，主循环写 tail） */
    Protocol_Frame_t frame_queue[PROTOCOL_QUEUE_SIZE];
    volatile uint16_t q_head;
    volatile uint16_t q_tail;
#endif
} Protocol_Context_t;

/* ==================== API（全部首参为 context） ==================== */
/**
 * @brief 初始化协议上下文
 * @note  会清空回调表、接收缓冲、错误计数。若先注册 handler 再调用本函数，注册会丢失。
 *        不要在通信进行中调用（会与中断中的 Protocol_Parse 产生竞态）。
 *        Protocol_HAL_Init 内部会调用本函数。
 */
void Protocol_Init(Protocol_Context_t *ctx);

/**
 * @brief 清空内部接收缓冲与（开启 DEFER 时的）延迟分发队列
 * @note  用于通信异常后的复位。调用时应确保该实例通信已停止。
 */
void Protocol_Reset(Protocol_Context_t *ctx);

/**
 * @brief 注册底层发送函数
 * @param user 传给 SendFunc 的用户上下文（HAL 层传 Protocol_HAL_Context_t*）
 */
void Protocol_RegisterSender(Protocol_Context_t *ctx,
                             Protocol_SendFunc_t send_func,
                             void *user);

/**
 * @brief 注册命令回调
 * @note  默认无锁；若需运行期动态注册/注销，请配置 PROTOCOL_LOCK_HANDLERS /
 *        PROTOCOL_UNLOCK_HANDLERS 宏（见文件顶部说明）。
 * @return true 成功；false 参数非法或表满
 */
bool Protocol_RegisterHandler(Protocol_Context_t *ctx, uint8_t cmd,
                              Protocol_Handler_t handler);

/**
 * @brief 注销命令回调
 * @return true 成功；false 未找到
 */
bool Protocol_UnregisterHandler(Protocol_Context_t *ctx, uint8_t cmd);

/**
 * @brief 解析新增的串口数据（有状态，支持跨调用半包）
 * @note  每次传入的是"增量数据"（例如 DMA+IDLE 收到的本批数据）。
 * @note  **不可重入**，不要在 handler 内再次调用。
 * @note  len 必须 <= PROTOCOL_RX_BUFFER_SIZE，否则整批被拒绝并复位缓冲。
 */
void Protocol_Parse(Protocol_Context_t *ctx, const uint8_t *buf, uint16_t len);

#if PROTOCOL_DEFER_HANDLER
/**
 * @brief 主循环中调用，取出队列中的帧并分发到 handler
 * @note  仅 PROTOCOL_DEFER_HANDLER=1 时可用；每个 context 单独调用。
 *        单生产者-单消费者无锁环形队列，不要多任务并发调用同一 context。
 */
void Protocol_Process(Protocol_Context_t *ctx);
#endif

/**
 * @brief 组帧并发送
 * @return true 表示底层发送函数已接受（HAL 层带发送环时=已入队，不等于已发出）；
 *         false 表示参数错误或发送资源耗尽（队列满/串口忙），调用方可重试。
 * @note  同一批到达的多条命令的应答会进入 HAL 发送环，不再因 tx_busy 丢弃。
 */
bool Protocol_Send(Protocol_Context_t *ctx, uint8_t cmd,
                   const uint8_t *data, uint8_t len);

/**
 * @brief 读取错误统计计数（原子读，可在中断运行中安全调用）
 */
uint32_t Protocol_GetErrorCount(Protocol_Context_t *ctx, Protocol_ErrorType_t type);

/**
 * @brief 清零所有错误计数
 * @note  使用原子写，可在中断运行时安全调用。
 */
void Protocol_ResetErrorCounts(Protocol_Context_t *ctx);

#endif /* PROTOCOL_H_INCLUDED */