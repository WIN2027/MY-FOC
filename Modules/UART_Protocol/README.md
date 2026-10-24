# 通用串口通信协议模块 (UART_Protocol)

硬件无关的串口协议解析与分发组件，支持粘包、半包、CRC 校验，通过回调注册实现业务层与底层驱动解耦。附带 STM32 HAL 适配层（DMA + IDLE），**支持多串口实例隔离**。

> v1.7（2026-10-24）：完成一轮完整代码审查后的修复。新增发送环、RX 先重启后解析、TX 超时、错误来源区分、Deinit、C99 断言兜底。变更摘要见第 10 章。

---

## 1. 架构设计

```text
┌────────────────────────────────────────────────────┐
│  业务层（用户代码）                                │
│   - Protocol_RegisterHandler(&ctx, cmd, fn)        │
│   - Protocol_Send(&ctx, cmd, data, len)            │
│   - (可选) Protocol_Process(&ctx) 分发              │
└────────────────────────────────────────────────────┘
                    ▲  每个实例独立 context
                    ▼
┌────────────────────────────────────────────────────┐
│  协议层  Protocol.c / Protocol.h（纯 C）           │
│   Protocol_Context_t  每实例一份（handler/rx/tx/err）│
│   - 组帧 / 拆帧 / CRC8 / 状态化缓冲                │
│   - 可选延迟分发环形队列                           │
│   - 错误统计（全原子访问）                         │
└────────────────────────────────────────────────────┘
                    ▲  g_send_func / Protocol_Parse
                    ▼
┌────────────────────────────────────────────────────┐
│  HAL 适配层  Protocol_HAL.c / Protocol_HAL.h       │
│   Protocol_HAL_Context_t  每 UART 一份             │
│   - huart → 实例 查表（HAL 回调唯一入口）           │
│   - DMA + IDLE 接收（仅 Normal）、DMA 发送         │
│   - 发送环（4 槽）+ TX 超时 + 错误延迟恢复          │
└────────────────────────────────────────────────────┘
```

**多实例隔离**：每个 UART 各持有独立的 `Protocol_Context_t` + `Protocol_HAL_Context_t`。handler 表、接收缓冲、延迟队列、错误计数、发送环完全独立；两个串口可以注册同名命令不同行为。

**为什么分离**：协议层可在 PC 上用 mock 发送做单元测试；HAL 层仅做 adapter，可替换为 Linux/ESP-IDF 等实现。

---

## 2. 帧格式规范

```text
+------+------+------+---------------+------+
| 0x5A | CMD  | LEN  |  DATA x LEN   | CRC8 |
+------+------+------+---------------+------+
```

| 字段 | 长度 | 说明 |
|---|---|---|
| SOF  | 1 | 固定 `0x5A` |
| CMD  | 1 | 命令码 |
| LEN  | 1 | DATA 长度，`0 ≤ LEN ≤ PROTOCOL_MAX_PAYLOAD (64)` |
| DATA | LEN | 数据域 |
| CRC8 | 1 | 对 CMD+LEN+DATA 计算，多项式 `0x07`，初值 `0xFF` |

命令码空间：

- `0x00 ~ 0x7F`：业务命令（用户自定义）
- `0x80 ~ 0xFF`：协议层保留（`ACK_OK=0x80`、`ERR_CRC=0x81`、`ERR_INVALID=0x82`）

> 错误帧回复只对**通过 CRC 的业务命令未注册**时触发；CRC 失败时**不回复**（整帧不可信，避免对伪 SOF 反复回帧）。

---

## 3. API 说明

### 3.1 协议层（Protocol.h）

| API | 说明 |
|---|---|
| `void Protocol_Init(Protocol_Context_t *ctx)` | 初始化上下文，清空回调表/接收缓冲/错误计数 |
| `void Protocol_Reset(Protocol_Context_t *ctx)` | 清空接收缓冲与延迟队列（通信停止时调用） |
| `void Protocol_RegisterSender(Protocol_Context_t *ctx, Protocol_SendFunc_t, void *user)` | 注册底层发送函数与用户上下文 |
| `bool Protocol_RegisterHandler(Protocol_Context_t *ctx, uint8_t cmd, Protocol_Handler_t)` | 注册命令回调 |
| `bool Protocol_UnregisterHandler(Protocol_Context_t *ctx, uint8_t cmd)` | 注销命令回调 |
| `void Protocol_Parse(Protocol_Context_t *ctx, const uint8_t *buf, uint16_t len)` | **有状态**解析，传入**增量数据** |
| `void Protocol_Process(Protocol_Context_t *ctx)` | **仅 `PROTOCOL_DEFER_HANDLER=1`**，主循环分发 |
| `bool Protocol_Send(Protocol_Context_t *ctx, uint8_t cmd, const uint8_t *data, uint8_t len)` | 组帧发送；`true`=底层已接受（HAL 层带发送环时=已入队） |
| `uint32_t Protocol_GetErrorCount(Protocol_Context_t *ctx, Protocol_ErrorType_t)` | 读取错误计数（原子读） |
| `void Protocol_ResetErrorCounts(Protocol_Context_t *ctx)` | 原子清零所有错误计数 |

**回调签名**

```c
typedef bool (*Protocol_SendFunc_t)(void *user, const uint8_t *data, uint16_t len);
```

**`Protocol_SendFunc_t` 数据生命周期契约**：`data` 指向 `Protocol_Send` 内部的栈缓冲，返回后立即失效。HAL 层在返回前已拷贝到发送环；其它异步实现必须先 `memcpy` 到自管理内存。

**`Protocol_Handler_t` 上下文**：默认在 `Protocol_Parse` 调用上下文（通常中断）执行，必须短小非阻塞；DEFER 模式下在 `Protocol_Process` 中执行。

**`Protocol_Handler_t` 中 `data` 指针**：仅回调执行期间有效，禁止保存。

### 3.2 HAL 适配层（Protocol_HAL.h）

| API | 说明 |
|---|---|
| `bool Protocol_HAL_Init(Protocol_HAL_Context_t *h, Protocol_Context_t *proto, UART_HandleTypeDef *huart)` | 绑定 UART 与协议上下文，启动 DMA+IDLE 接收；**会调用 `Protocol_Init(proto)` 清空 handler**；**RX DMA 必须为 Normal**；**同一 huart 不得重复绑定** |
| `void Protocol_HAL_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)` | 在 `HAL_UARTEx_RxEventCallback` 内转发 |
| `void Protocol_HAL_TxCpltCallback(UART_HandleTypeDef *huart)` | 在 `HAL_UART_TxCpltCallback` 内转发，发送环继续下一帧 |
| `void Protocol_HAL_ErrorCallback(UART_HandleTypeDef *huart)` | 在 `HAL_UART_ErrorCallback` 内转发；区分 RX 行错误 / DMA 错误，不再误伤 TX/RX |
| `void Protocol_HAL_Process(void)` | **主循环调用**：处理 RX 错误恢复、TX 超时恢复、发送环泵送 |
| `bool Protocol_HAL_RestartRx(Protocol_HAL_Context_t *h)` | 手动重新挂起接收（不清空 handler） |
| `bool Protocol_HAL_Deinit(Protocol_HAL_Context_t *h)` | 解绑：Abort RX/TX、注销实例、清空发送器 |
| `uint32_t Protocol_HAL_GetTxDropCount(const Protocol_HAL_Context_t *h)` | 发送环满丢弃计数 |
| `uint32_t Protocol_HAL_GetTxErrorCount(const Protocol_HAL_Context_t *h)` | HAL 启动失败 / TX DMA 错误 / 超时计数 |

**多实例约束**：`Protocol_HAL_Context_t` 数量 ≤ `PROTOCOL_HAL_MAX_INSTANCES`（默认 4），否则 Init 返回 false。

**Init 顺序**：`Protocol_Init → RegisterSender → StartRx`。副作用：StartRx 失败时 handler 表已清空，用户需重新 Init + 注册。

**Init 的占用检查**：若目标 `huart` 已被其它 HAL 上下文占用，**在拆旧绑定之前**返回 false，不会把旧实例拆坏。

### 3.3 错误统计

| 枚举 | 含义 |
|---|---|
| `PROTOCOL_ERR_CRC` | CRC 校验失败（每次重同步 1 字节计一次） |
| `PROTOCOL_ERR_INVALID_LEN` | LEN > `PROTOCOL_MAX_PAYLOAD` |
| `PROTOCOL_ERR_UNKNOWN_CMD` | 分发的帧无匹配 handler |
| `PROTOCOL_ERR_QUEUE_FULL` | 延迟队列满（DEFER 模式） |
| `PROTOCOL_ERR_RX_OVERFLOW` | 接收缓冲溢出 |

> 发送侧的丢弃/错误不在 `err_count[]` 中，使用 `Protocol_HAL_GetTxDropCount()` / `Protocol_HAL_GetTxErrorCount()` 读取，避免协议层感知 HAL。
>
> `PROTOCOL_ERR_CRC` 按“每次重同步 1 字节”累计：一帧损坏且数据域含伪 SOF 时可能计多次，属于“重同步次数”而非“坏帧数”；如需按帧统计，请在应用层另计。

---

## 4. 集成步骤（STM32 HAL）

### 4.1 CubeMX 配置

1. 使能 UART RX/TX DMA。
2. **RX DMA 必须配置为 Normal 模式**（本层不兼容 Circular）。
3. 打开 `USARTx global interrupt`（IDLE 依赖 UART 中断）。
4. **TX DMA 必须存在且为 Normal 模式**（发送环使用 `HAL_UART_Transmit_DMA`）；缺少 `hdmatx` 或为 Circular 时 `Protocol_HAL_Init` 返回 false。
5. **内存放置**：`Protocol_HAL_Context_t` 含 DMA 收发缓冲，必须放在 DMA 可访问 SRAM。STM32G4 的 CCM SRAM（`0x1000_0000`）DMA 不可访问；不要把该结构体放进 CCM。
   ```c
   __attribute__((section(".sram_dma"))) Protocol_HAL_Context_t g_hal1;
   ```

### 4.2 转发 HAL 回调

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Protocol_HAL_RxEventCallback(huart, Size);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    Protocol_HAL_TxCpltCallback(huart);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    Protocol_HAL_ErrorCallback(huart);
}
```

### 4.3 单串口示例

```c
static Protocol_Context_t     g_proto1;
static Protocol_HAL_Context_t g_hal1;

static void OnPing(uint8_t cmd, const uint8_t *data, uint8_t len) {
    (void)cmd; (void)data; (void)len;
    Protocol_Send(&g_proto1, 0x01, (const uint8_t *)"PONG", 4);  /* 入发送环 */
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();

    if (!Protocol_HAL_Init(&g_hal1, &g_proto1, &huart1)) { Error_Handler(); }
    Protocol_RegisterHandler(&g_proto1, 0x01, OnPing);

    while (1) {
        Protocol_HAL_Process();               /* RX/TX 恢复 + 发送环泵送 */
#if PROTOCOL_DEFER_HANDLER
        Protocol_Process(&g_proto1);
#endif
    }
}
```

### 4.4 双串口（完全隔离）

```c
static Protocol_Context_t     g_proto1, g_proto2;
static Protocol_HAL_Context_t g_hal1,   g_hal2;

static void OnPingA(uint8_t c, const uint8_t *d, uint8_t l) {
    (void)c;(void)d;(void)l;
    Protocol_Send(&g_proto1, 0x01, (const uint8_t *)"PONG-A", 6);
}
static void OnPingB(uint8_t c, const uint8_t *d, uint8_t l) {
    (void)c;(void)d;(void)l;
    Protocol_Send(&g_proto2, 0x01, (const uint8_t *)"PONG-B", 6);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    if (!Protocol_HAL_Init(&g_hal1, &g_proto1, &huart1)) Error_Handler();
    if (!Protocol_HAL_Init(&g_hal2, &g_proto2, &huart2)) Error_Handler();

    Protocol_RegisterHandler(&g_proto1, 0x01, OnPingA);
    Protocol_RegisterHandler(&g_proto2, 0x01, OnPingB);

    while (1) {
        Protocol_HAL_Process();
#if PROTOCOL_DEFER_HANDLER
        Protocol_Process(&g_proto1);
        Protocol_Process(&g_proto2);
#endif
    }
}
```

### 4.5 通信异常恢复

```c
while (1) {
    Protocol_HAL_Process();          /* 推荐：自动处理 RX 重启 / TX 超时 / 发送环 */
}
/* 手动恢复 */
if (!Protocol_HAL_RestartRx(&g_hal1)) {
    Protocol_Reset(&g_proto1);
    (void)Protocol_HAL_RestartRx(&g_hal1);
}
```

### 4.6 运行期动态注册

默认协议层对 handler 表无锁保护。若需运行期动态注册/注销：

```c
#define PROTOCOL_LOCK_HANDLERS()                                    \
    uint32_t _protocol_pm_ = __get_PRIMASK(); __disable_irq()
#define PROTOCOL_UNLOCK_HANDLERS()                                  \
    __set_PRIMASK(_protocol_pm_)
```

两个宏作用于**整个模块**（所有 context 共享一把锁），同一时刻只有一个注册动作，因此安全。

> **C89 兼容性**：上述示例会在函数体中插入变量声明；若工程启用了 `-Wdeclaration-after-statement`，请把 `PROTOCOL_LOCK_HANDLERS()` 放在函数体所有声明之后、语句之前。

---

## 5. 编译期配置

`Protocol.h`：

| 宏 | 默认 | 说明 |
|---|---|---|
| `PROTOCOL_SOF` | `0x5A` | 帧头 |
| `PROTOCOL_MAX_HANDLERS` | `32` | 每实例命令上限（断言 `0 < n ≤ 255`） |
| `PROTOCOL_MAX_PAYLOAD` | `64` | 单帧最大数据（断言 `≤ 253`） |
| `PROTOCOL_FRAME_OVERHEAD` | `4` | 断言 `== 4` |
| `PROTOCOL_RX_BUFFER_SIZE` | `256` | 每实例接收缓冲（断言 `MAX_PAYLOAD+OVERHEAD ≤ n ≤ 65535`） |
| `PROTOCOL_AUTO_REPLY_ERROR` | `1` | 是否回错误帧（仅对 CRC 通过的未注册业务命令） |
| `PROTOCOL_DEFER_HANDLER` | `0` | 延迟分发到主循环 |
| `PROTOCOL_QUEUE_SIZE` | `8` | 延迟队列深度（断言 `≥2 && is_power_of_2`） |
| `PROTOCOL_LOCK_HANDLERS` / `UNLOCK` | 空操作 | 全局 handler 表并发保护 |

`Protocol_HAL.h`：

| 宏 | 默认 | 说明 |
|---|---|---|
| `PROTOCOL_HAL_HEADER` | `"main.h"` | HAL 头文件 |
| `PROTOCOL_HAL_RX_BUF_SIZE` | `128` | 单批 DMA 缓冲；**断言 `≤ PROTOCOL_RX_BUFFER_SIZE`** |
| `PROTOCOL_HAL_MAX_INSTANCES` | `4` | 同时可初始化的串口数 |
| `PROTOCOL_HAL_TX_QUEUE_DEPTH` | `4` | 发送环深度（含 1 个空槽，实际排队 3 帧）；每槽 68 B |
| `PROTOCOL_HAL_TX_TIMEOUT_MS` | `50` | TX 超时下限；实际按波特率自动放宽（整帧时间+20 ms） |
| `PROTOCOL_HAL_AUTO_RESTART_RX` | `0` | 1 = 在 UART 错误 ISR 内直接 Abort+重启 RX（否则由主循环恢复） |

`Protocol.c` 可覆盖：

| 宏 | 说明 |
|---|---|
| `PROTOCOL_COMPILER_BARRIER` | 编译屏障；弱序/SMP 平台替换为 `__DMB()` |

**内存占用（实测，GCC 13 / Cortex-M4）**：

| 组件 | DEFER=0 | DEFER=1 |
|---|---|---|
| `Protocol_Context_t` | 672 B | 1,204 B |
| `Protocol_HAL_Context_t` | 624 B | 624 B |
| **合计/实例** | **1,296 B** | **1,828 B** |

HAL 上下文含 128 B RX + 128 B 先拷缓冲 + 68 B TX + 4×68 B 发送环；未计发送队列时可降到约 1 KB（需自行改宏/裁剪）。

**接收缓冲上限**：`ctx->rx_len` 为 `uint16_t`，`PROTOCOL_RX_BUFFER_SIZE` 必须 ≤ 65535（编译期断言）。

---

## 6. 发送路径与背压（v1.7）

发送不再“忙时丢弃”：

```text
Protocol_Send
  └─ Protocol_HAL_Send：帧写入 4 槽发送环（拷贝）
       ├─ 入环成功 → hal_tx_pump：若空闲则启动 DMA
       └─ 环满 → tx_drop_count++，返回 false
DMA TC → Protocol_HAL_TxCpltCallback：清 busy → 泵出下一帧
```

- 同一批 DMA 里到达的 2 条命令，两条应答都能入环，不再只发第一条。
- 环深度 4 → 实际最多 3 帧等待 + 1 帧在发；`PROTOCOL_HAL_TX_QUEUE_DEPTH` 可调（≥2）。
- 发送失败/超时通过 `Protocol_HAL_GetTxErrorCount()` 观测；环满通过 `Protocol_HAL_GetTxDropCount()` 观测。
- 超时：`tx_busy` 超过 `max(50 ms, 整帧时间+20 ms)` 时，`Protocol_HAL_Process` 强制 `AbortTransmit` 并恢复。
- 临界区只保护环索引/拷贝/状态，`HAL_UART_Transmit_DMA` 在临界区外执行；`tx_starting` 标志避免启动窗口被错误中断重入。

---

## 7. 接收路径（v1.7）

```text
DMA IDLE/TC → HAL_UARTEx_RxEventCallback
  ├─ 1. 把本批数据拷贝到 rx_pending
  ├─ 2. 立刻重启 RX（Normal DMA 必须重挂）
  └─ 3. 再 Protocol_Parse(rx_pending) → handler/队列
```

- **先重启再解析**：handler 在 ISR 中执行期间 UART 仍在接收，避免默认模式下丢字节。
- `rx_pending` 与 `rx_dma_buf` 分离，解析时 DMA 可以继续写入下一批。
- HT 中断在启动后被关闭；若 HAL 仍回调 HT，模块按增量数据解析且**不重启**接收，避免重复解析/误重启。
- RX 行错误（PE/NE/FE/ORE）只置 `rx_need_restart`，不再动 TX；DMA 错误则分别恢复 TX/RX。

---

## 8. 错误恢复

```text
RX 行错误：HAL 结束接收 → ErrorCallback
             └─ rx_need_restart = true（不动 TX）
DMA 错误：  HAL 可能同时结束 TX/RX
             ├─ TX busy → AbortTransmit + 清 busy + 泵下一帧
             └─ RX active → rx_need_restart = true
主循环：    Protocol_HAL_Process
             ├─ rx_need_restart → RestartRx
             ├─ tx_busy 超时 → AbortTransmit + 计数
             └─ 空闲且环非空 → 泵送
```

- `PROTOCOL_HAL_AUTO_RESTART_RX=1` 可在错误 ISR 内直接恢复 RX，代价是 ISR 内执行 HAL Abort。
- 默认 0：必须周期调用 `Protocol_HAL_Process()`，否则 RX 错误后不会自动重启。
- TX 超时恢复不依赖 RX 状态；TX 错误不再误触发 RX 重启。

---

## 9. 使用注意事项

1. **每串口一个 `Protocol_Context_t` + `Protocol_HAL_Context_t`**（用户静态分配，勿放栈）。
2. **`Protocol_Parse` 传入增量数据**，不要传累积缓冲区。
3. **handler 的 `data` 指针仅回调期间有效**。
4. **`SendFunc` 的 `data` 指针仅函数调用期间有效**，异步发送必须拷贝（HAL 层已拷贝）。
5. **默认模式 handler 在中断上下文**，短小非阻塞；复杂逻辑开 `PROTOCOL_DEFER_HANDLER`。
6. **`Protocol_Send` 返回 false** 表示参数错误或发送环满；调用方可重试或记录 `tx_drop_count`。
7. **`Protocol_HAL_Init` 会清空对应 `proto` 的 handler 表**，注册请放在其后。
8. **`Protocol_HAL_Init` 可安全重复调用**（同一 h 会先 Abort 旧绑定再重启），但每次成功都会重置 handler 表与发送环计数。
9. **RX DMA 必须 Normal**，Circular 直接返回 false。
10. **转发 `HAL_UART_ErrorCallback` 后必须主循环调用 `Protocol_HAL_Process()`**（除非开启 `PROTOCOL_HAL_AUTO_RESTART_RX`）。
11. **`PROTOCOL_HAL_RX_BUF_SIZE ≤ PROTOCOL_RX_BUFFER_SIZE ≤ 65535`** 均已在编译期断言。
12. **CRC 失败不回复错误帧**，只计数并重同步 1 字节。
13. **非法 LEN** 跳 1 字节重同步，计 `PROTOCOL_ERR_INVALID_LEN`。
14. **接收缓冲溢出**从尾部回溯最后一个 SOF 保留半包，计 `PROTOCOL_ERR_RX_OVERFLOW`。
15. **`Protocol_Reset()` / `Protocol_Init()` 需在通信停止时调用**。
16. **运行期动态注册/注销**必须配置 `PROTOCOL_LOCK_HANDLERS/UNLOCK`。
17. **SOF 选型**：若数据域高频出现 `0x5A`，溢出回溯的保留起点可能落在帧中间，需自行评估。
18. **DMA 缓冲必须放 SRAM**，不能放 CCM/DTCM（G4 CCM DMA 不可访问）。
19. **`err_count[]` 全链路原子**（GCC/Clang）；非 GCC 编译器回退为普通读写，此时请勿在中断运行中 `ResetErrorCounts`。
20. **`Protocol_HAL_Context_t` 增大约 416 B**（RX 先拷缓冲 128 + 发送环 272 + 标志/时间戳），请确认 RAM 预算。

---

## 10. 变更记录

### v1.7（2026-10-24，审查修复）
- **P1**：新增 HAL 发送环（默认 4 槽），`Protocol_HAL_Send` 入环即返回；`TxCplt` 自动泵送。修复“同批多帧只有首帧应答”的丢帧问题。
- **P1/P2**：`Protocol_HAL_ErrorCallback` 区分 RX 行错误与 DMA/TX 错误；TX 错误不再置位 `rx_need_restart`，不再误伤 RX。
- **P2**：RX 回调改为“先拷贝到 `rx_pending` → 重启 RX → 再解析”，handler 执行期间不丢字节。
- **P2**：新增 TX 超时恢复（按波特率动态计算）与 `tx_error_count`；新增 `tx_drop_count`。
- **P2**：新增 `Protocol_HAL_Deinit`；Init 在拆旧绑定前先校验目标 huart 占用，修复跨 huart 重绑破坏旧实例。
- **P2**：HT 回调按增量数据解析且不重启；启动后关闭 HT，避免半批重启。
- **P3**：`PROTOCOL_SA` 在 C99 下用负长度数组兜底，编译期检查不再静默失效。
- **P3**：抽取 `dispatch_frame()`，消除 DEFER/非 DEFER 两条路径的重复自动应答逻辑。
- **文档**：更新内存实测、DMA 放置、发送环/超时/错误恢复说明。

### v1.6（2026-10-23，一致性修复）
- `PROTOCOL_RX_BUFFER_SIZE <= 0xFFFF` 断言；错误计数全链路原子；Init 顺序 Init→RegisterSender→StartRx；README 补充队列深度/锁 C89 说明；Init 失败回滚清 proto。

### v1.5（多实例 + 小问题修复）
- 多实例重构；`SendFunc` 增加 user；`Protocol_HAL_Init(h, proto, huart)`；handler 表 volatile + 写序；ErrorCallback 不在 ISR Abort RX；CRC 失败不回复；`Protocol_ResetErrorCounts` 原子写。

### v1.4
- 延迟队列复位；RX DMA Mode 检查；`Protocol_Parse` 溢出回溯；CRC 长度 uint16；可配置临界区宏；错误统计 API。

### v1.3
- 队列断言修复；Send 临界区；延迟队列屏障；SendFunc 数据生命周期契约；`RestartRx`；Init 幂等。

### v1.2
- 拒绝超长批；StartRx 返回值检查；ErrorCallback；DEFER 选项；HAL 头可配置；失败回滚。

### v1.1
- 有状态半包解析；非法 LEN 检查；错误帧仅业务命令；`Protocol_Send` 返回 bool；`Protocol_Reset`；DMA+IDLE HAL 层。

### v1.0
- 首个版本（纯协议层，无状态）。

---

## 附录 A：帧格式与示例

```c
/* 业务命令 0x01，数据 "PONG" */
static const uint8_t payload[] = {'P', 'O', 'N', 'G'};
Protocol_Send(&g_proto, 0x01, payload, sizeof(payload));

/* 生成帧：5A 01 04 50 4F 4E 47 <CRC8> */
```

## 附录 B：错误处理速查

| 现象 | 检查 |
|---|---|
| 完全收不到 | RX DMA 是否 Normal、UART 中断是否打开、`Protocol_HAL_Process` 是否调用、缓冲区是否在 CCM |
| 偶发丢应答 | `GetTxDropCount` 是否增长（环满）；`GetTxErrorCount` 是否增长（HAL 启动/超时） |
| 偶发丢帧 | RX 行错误标志、`PROTOCOL_ERR_CRC/INVALID_LEN/RX_OVERFLOW` 计数 |
| 发送卡死 | `tx_busy` 是否长期为真；`Protocol_HAL_Process` 是否周期调用 |
| 动态注册崩溃 | 是否配置 `PROTOCOL_LOCK_HANDLERS/UNLOCK` |

## 附录 C：v1.7 行为变化

| 场景 | v1.6 | v1.7 |
|---|---|---|
| 同批两帧命令 | 第二条应答丢弃 | 两条应答都入环发送 |
| TX DMA 错误 | 同时置 RX 重启，可能丢半帧 | 只恢复 TX，不动 RX |
| handler 执行 1 ms | 期间 RX 停止，丢约 11 B | 先重启 RX，handler 期间继续收 |
| TC 中断丢失 | `tx_busy` 永久为真 | 超时 Abort 并恢复 |
| 跨 huart 重绑失败 | 旧实例已被拆一半 | 先校验，失败不破坏旧实例 |
| 无 Deinit | 无法解绑 | `Protocol_HAL_Deinit` |
| C99 编译 | 静态断言失效 | 负长度数组兜底 |
