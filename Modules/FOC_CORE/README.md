# 通用 FOC 控制模块 (FOC_Core)

硬件无关的无感 FOC 核心算法模块（SMO + PI/PID + SVPWM + 状态机），**面向单电机 + 单 MCU 场景**。核心层不依赖 HAL/RTOS；硬件能力通过 `FOC_PORT_*` 宏下沉到移植层。

> v1.3（2026-10-24）：完成一轮完整代码审查后的修复与加固。变更摘要见第 11 章。

---

## 1. 特性

| 特性 | 说明 |
|---|---|
| 无感启动 | 对齐与启动分离：`0x09`/`FOC_Start(ALIGN)` 仅对齐；`0x04`/`FOC_Start(OPENLOOP)` 跳过对齐，直接 OPENLOOP（IF 强拖）→ TRANSITION（角度混合）→ CLOSEDLOOP（速度闭环） |
| SMO 观测器 | 反电动势 LPF + 反转 π 补偿；当前 `theta_comp` 暂时停用，SMO 输出角直接用于过渡和闭环；`FOC_LEGACY_CONTROL=0` 使用自动 Kslide/前馈 |
| 电流环 | d/q 双 PI，抗积分饱和，电压圆限幅；前馈解耦/反电动势补偿仅在闭环后生效 |
| SVPWM | min-max 零序注入，占空比按实际母线电压缩放；由比较值反算实际 α/β 电压供 SMO 使用 |
| 故障安全 | 故障锁存（FAULT 不会被状态迁移覆盖）；软件过流、开环收敛超时、非法输入均触发 FAULT |
| 配置安全 | 所有浮点配置做 NaN/Inf 校验；`FOC_Start` 快照配置；编译期检查 PWM/控制频率一致性 |
| 实时性 | 每 PWM 周期一次 `FOC_Run`，测时用 DWT；无动态内存、无阻塞等待 |
| 可移植 | 9 个必需 port 宏 + 1 个可选断言宏；临界区用 PRIMASK，中断/任务均可调用，不绑定 RTOS |
| 内存 | 核心 `.text ≈ 7.3 KB`、`.bss ≈ 404 B`（`-O3`，不含 libm）；`FOC_Run` 栈约 80 B |

---

## 2. 架构

```text
应用层（ADC 完成回调 / HRTIM 重复事件 / 上位机）
  │  FOC_Run(Iu_A, Iv_A)           每 PWM 周期 48 kHz
  │  FOC_SpeedLoopTick()           每个 HRTIM 重复事件（内部分频 25）
  ▼
核心层 FOC_CORE.c（纯 C、单实例、静态状态）
  Clarke → Park → 电流 PI → 逆 Park → SVPWM → PWM 写入
                     ↘ SMO 更新（角度/速度/收敛判定）
  状态机：IDLE → OPENLOOP → TRANSITION → CLOSEDLOOP；可选 IDLE → ALIGN → OPENLOOP；FAULT 旁路
  │  FOC_PORT_* 宏（9 个必需 + 1 个可选）
  ▼
硬件层 FOC_PORT_STM32G4.h / .c
  HRTIM 比较寄存器/输出使能、CORDIC 相位、DWT 测时、PRIMASK 临界区
```

**为什么单实例**：FOC 与电机一一对应，多电机 = 多 MCU。所有 API 不带 `ctx`，状态为文件级 static/全局，调用点最简。

**为什么宏下沉**：48 kHz 下每帧都调用，函数指针跳转和 `ctx->` 间接访问都值得省；宏展开与直接写寄存器等价，零开销。

**为什么核心层可单测**：定义 `FOC_PORT_TEST_MODE` 后 `FOC_PORT.h` 用空实现（CORDIC 退化为 `sqrtf + atan2f`，测时返回 0），可在 PC 上编译并运行完整状态机逻辑。

---

## 3. 状态机、故障与保护

### 3.1 状态

| 状态 | 电流给定 | 角度来源 | 退出条件 |
|---|---|---|---|
| `IDLE` | PWM 输出 0 | — | `FOC_Start(ALIGN)` 或 `FOC_Start(OPENLOOP)` |
| `ALIGN` | `align_current_a`，`iq=0` | θ=0 强制对齐 | 计时到 `align_time_ms` → OPENLOOP（应用层仅对齐时在进入 OPENLOOP 的同一拍停机并清状态） |
| `OPENLOOP` | `openloop_current_a` | IF 强拖斜坡 | SMO 收敛 → TRANSITION |
| `TRANSITION` | 强拖给定不变 | 开环/SMO 角度按 α 混合 | α≥0.999 → CLOSEDLOOP |
| `CLOSEDLOOP` | 速度环输出（PI_q） | SMO 估计 | `FOC_Stop()` / `FOC_OnFault()` |
| `FAULT` | PWM 关闭 | — | `FOC_Start(ALIGN)` 或 `FOC_Start(OPENLOOP)`（清故障锁存） |

### 3.2 故障锁存（v1.3）

`FOC_OnFault()` 会置位内部 `s_fault_latched`，并关闭 PWM、切 `FAULT`。所有状态迁移（`OPENLOOP→TRANSITION`、`TRANSITION→CLOSEDLOOP`、`ALIGN→OPENLOOP`）都走 `foc_transition()` 守卫：**当前状态必须是预期状态且故障未锁存**，否则不写状态。

这样即使高优先级 HRTIM Fault 中断抢占了 `FOC_Run`，`FAULT` 也不会被后来执行的 `foc_state = TRANSITION` 覆盖。

`FOC_Start(ALIGN)` / `FOC_Start(OPENLOOP)` 会清锁存（这是复位入口）；`FOC_Stop()` 也会清锁存。

### 3.3 保护

| 保护 | 触发条件 | 故障码 |
|---|---|---|
| 非法输入 | `Iu/Iv` 为 NaN/Inf | `FOC_FAULT_INVALID_INPUT` |
| 软件过流 | 相电流峰值 `> FOC_SW_OC_TRIP_A` 连续 `FOC_SW_OC_TRIP_CNT` 个 PWM 周期 | `FOC_FAULT_OVERCURRENT` |
| 开环超时 | OPENLOOP/TRANSITION 持续 `2 × openloop_time_ms + 500 ms` | `FOC_FAULT_OPENLOOP_TIMEOUT` |
| CORDIC 模式 | `FOC_Init` 自检失败（`FOC_Start` 返回 `FOC_ERR_HW_SELFTEST`） | `FOC_FAULT_CORDIC_MODE` |
| 硬件故障 | 应用调用 `FOC_OnFault(code)`（HRTIM Fault 等） | 用户自定义 |

> CORDIC 自检失败时 `FOC_Start` 会拒绝启动并保留 `last_fault`，不再“启动一次就清掉故障码”。

### 3.4 收敛阈值与 Kslide（v1.3）

- 收敛阈值不是固定 800 rpm：进入 OPENLOOP 时按
  `conv_rpm_min = min(FOC_SMO_CONV_RPM_MIN, FOC_SMO_CONV_RATIO × 强拖目标机械转速)` 计算，下限 50 rpm。
  例：7 对极、60 Hz（≈514 rpm）→ 阈值 ≈437 rpm，可以正常进入闭环；175 Hz 时仍为 800 rpm。
- `Kslide` 在 `FOC_Init`、`FOC_SetStartupConfig`、`FOC_SetSpeedTarget` 时按运行范围内最高转速重算：
  `Kslide = MARGIN × ω_e(max_rpm) × ψ_f`，并钳位在 `[FOC_SMO_KSLIDE_MIN, FOC_SMO_Z_MAX]`。
  这样强拖频率或目标转速提高时，z 不会先被钳死。

---

## 4. API

### 4.1 生命周期

| API | 说明 |
|---|---|
| `void FOC_Init(void)` | 初始化全部结构体/状态，清 `cfg_valid`、计算 Kslide、做 CORDIC 模式自检。**必须在 HRTIM/CORDIC 初始化之后、首次 `FOC_Run` 之前调用一次**。 |
| `void FOC_Deinit(void)` | 关 PWM、清故障锁存、清零动态状态；置为未初始化，再次使用前需重新 `FOC_Init`。 |

### 4.2 启动参数

```c
typedef struct {
    float    align_current_a;     /* 对齐电流 A */
    uint32_t align_time_ms;       /* 对齐时长 ms */
    float    openloop_current_a;  /* 强拖电流 A */
    uint32_t openloop_time_ms;    /* 强拖加速时间 ms */
    float    openloop_freq_hz;    /* 强拖目标电频率 Hz */
} FOC_StartupConfig_t;
```

| API | 说明 |
|---|---|
| `FOC_Err_t FOC_SetStartupConfig(const FOC_StartupConfig_t *cfg)` | 仅在 IDLE/FAULT 可调用；所有字段做范围 + NaN/Inf 检查；成功后会重算 Kslide。 |
| `FOC_Err_t FOC_GetStartupConfig(FOC_StartupConfig_t *cfg)` | 读取用户配置（不是运行快照）。 |

**配置锁语义**：上电 `cfg_valid=false`，`FOC_Start` 返回 `FOC_ERR_NOT_CONFIGURED`；首次配置成功后保持到下一次 `FOC_Init` 或复位；运行中修改返回 `FOC_ERR_BUSY`；`FOC_Start` 会把配置快照到 `run_cfg`，运行期间只读快照。

### 4.3 启停与故障

| API | 说明 |
|---|---|
| `FOC_Err_t FOC_Start(FOC_State_t mode)` | 接受 `FOC_STATE_ALIGN`（仅对齐）或 `FOC_STATE_OPENLOOP`（跳过对齐直接强拖）。需已配置、状态为 IDLE/FAULT、CORDIC 自检通过。内部清运行时状态并快照配置、使能 PWM；若使能期间检测到故障，返回 `FOC_ERR_INVALID_STATE` 并关 PWM。 |
| `void FOC_Stop(void)` | 关 PWM、清故障锁存、切 IDLE、清运行时状态（角度/电流环/SMO）与遥测。 |
| `void FOC_OnFault(uint32_t code)` | 关 PWM、锁存 FAULT、清运行时状态与遥测、记录故障码。可由任意中断/任务调用。 |
| `uint32_t FOC_GetLastFault(void)` | 最近一次故障码（见第 5 章）。 |

### 4.4 周期调用

| API | 说明 |
|---|---|
| `void FOC_Run(float Iu_A, float Iv_A)` | 每 PWM 周期一次（ADC 完成回调）。输入为相电流安培值；内部完成输入校验/过流判断 → Clarke → Park → 电流环 → 逆 Park → SVPWM → SMO。 |
| `void FOC_SpeedLoopTick(void)` | 每 HRTIM 重复事件一次；按 `FOC_SPEED_LOOP_DIV` 分频。只有 CLOSEDLOOP 执行速度环。 |

### 4.5 参数

| API | 说明 |
|---|---|
| `void FOC_SetSpeedTarget(int16_t rpm)` | 设置目标机械转速；会按新目标重算 Kslide。 |
| `FOC_Err_t FOC_SetStartupConfig(...)` / `FOC_GetStartupConfig(...)` | 设置/读取对齐与强拖参数；仅 IDLE/FAULT 可写。 |
| `FOC_Err_t FOC_SetMotorParams(...)` / `FOC_GetMotorParams(...)` | 设置/读取电机参数（R、Ld、Lq、磁链、极对数）；仅 IDLE/FAULT 可写，写入后同步更新 SMO 系数与 Kslide。 |
| `void FOC_SetCurrentTarget(float iq_a)` | 直接设置 q 轴电流目标（OPENLOOP 阶段有意义）；非有限值被忽略。 |
| `void FOC_SetSpeedPID(float kp, float ki, float kd)` | 设置速度环 PID；非有限值被忽略。 |
| `void FOC_SetCurrentPID(float kp_d, float ki_d, float kp_q, float ki_q)` | 设置 d/q 电流环 PI。 |
| `void FOC_GetSpeedPID(...)` / `FOC_GetCurrentPID(...)` | 读取参数。 |
| `void FOC_SetSpeedLimits(...)` / `FOC_SetCurrentLimits(...)` | 设置输出/积分限幅（min≤max、有限值校验）。 |
| `void FOC_SetVbus(float vbus)` | 在线更新母线电压（1~200 V，有限值），同时更新 SVPWM 缩放与电压圆限幅。 |

### 4.6 遥测

```c
typedef struct {
    uint16_t foc_dwt;    /* FOC_Run 入口 → PWM 写入完成周期数，饱和 65535 */
    int16_t  IU, IV, IW; /* 三相电流，100 倍放大 */
} FOC_Telemetry_t;
```

| API | 说明 |
|---|---|
| `void FOC_GetTelemetry(FOC_Telemetry_t *tele)` | 临界区（PRIMASK 保存/恢复）拷贝；**任务或中断上下文都可调用**。 |
| `FOC_State_t FOC_GetState(void)` | 当前状态。 |
| `void FOC_SetCycleCount(uint32_t cycles)` | 硬件层回写周期数（内部饱和），应用一般不调用。 |

> `foc_dwt` 是墙上时间，包含高优先级中断抢占；评估算法耗时应取多次采样的最小值。`FOC_PORT_TEST_MODE` 下恒为 0。

### 4.7 对外只读状态

```c
extern volatile FOC_State_t foc_state;
extern PID_Handle_t         speed_pid;
extern SMO_Handle_t         smo;      /* 含 v1.3 新增 conv_rpm_min */
extern PI_Handle_t          pi_d, pi_q;
extern FOC_PWM_t            PWM;
extern FOC_Telemetry_t      s_telemetry;
```

应用层可读不可写；修改请用 `FOC_Set*` API。`foc_state` 为 `volatile`，保证故障抢占后每次从内存重读。

---

## 5. 错误码与故障码

```c
typedef enum {
    FOC_OK = 0,
    FOC_ERR_NOT_CONFIGURED,      /* 未配置启动参数 */
    FOC_ERR_PARAM_OUT_OF_RANGE,  /* 参数越界或非有限值 */
    FOC_ERR_BUSY,                /* 运行中禁止操作 */
    FOC_ERR_INVALID_STATE,       /* 状态/参数不合法 */
    FOC_ERR_HW_SELFTEST          /* CORDIC 自检失败等硬件问题 */
} FOC_Err_t;
```

| 故障码 | 含义 |
|---|---|
| `FOC_FAULT_NONE`（0） | 无故障 |
| `FOC_FAULT_CORDIC_MODE`（0xC0DE0001） | CORDIC 不是 PHASE 模式（自检失败，`FOC_Start` 被拒绝） |
| `FOC_FAULT_OPENLOOP_TIMEOUT`（0xC0DE0002） | 开环/过渡超时，SMO 未收敛 |
| `FOC_FAULT_OVERCURRENT`（0xC0DE0003） | 软件过流 |
| `FOC_FAULT_INVALID_INPUT`（0xC0DE0004） | 电流输入 NaN/Inf |

与串口错误码的映射仍由应用层决定（建议 `FOC_OK→0x80`、`NOT_CONFIGURED→0x83`、`BUSY→0x85`、`INVALID_STATE→0x82`、`HW_SELFTEST→0x86`）。

---

## 6. 编译期配置（`FOC_CONFIG.h`）

### 6.1 电机与硬件

| 宏 | 默认 | 说明 |
|---|---|---|
| `FOC_MOTOR_R` / `LD` / `LQ` / `FLUX_LINK` / `POLE_PAIRS` | 视电机 | 上电默认电机参数；运行时可经 `FOC_SetMotorParams` 覆盖，极对数不再依赖编译期宏 |
| `FOC_PWM_FREQ_HZ` | 48000 | PWM 频率 |
| `FOC_CTRL_FREQ_HZ` | 48000 | **必须等于** `FOC_PWM_FREQ_HZ`（编译期 `#error`） |
| `FOC_PWM_PERIOD` | 1770 | HRTIM 计数周期，必须 ≤ 65535 |
| `FOC_PWM_DT_S` | `1/FOC_PWM_FREQ_HZ` | 控制周期（秒）；旧名 `FOC_PWM_DT_HZ` 保留为别名 |
| `FOC_VBUS_VOLTAGE` | 24.0f | 默认母线电压，运行时可 `FOC_SetVbus` 覆盖 |
| `FOC_I_MAX_DECOUPLE` | 4.0f | 解耦项电流限幅 |
| `FOC_SW_OC_TRIP_A` | 8.0f | 软件过流阈值（相电流峰值） |
| `FOC_SW_OC_TRIP_CNT` | 3 | 过流连续确认周期数 |
| `FOC_SPEED_LOOP_DIV` | 25 | 速度环分频（1~65535） |

### 6.2 启动参数范围

| 宏 | 默认 | 说明 |
|---|---|---|
| `FOC_ALIGN_CURRENT_MIN/MAX_A` | 0.1 / 5.0 | 对齐电流范围 |
| `FOC_ALIGN_TIME_MIN/MAX_MS` | 10 / 10000 | 对齐时长 |
| `FOC_OPENLOOP_CURRENT_MIN/MAX_A` | 0.1 / 5.0 | 强拖电流 |
| `FOC_OPENLOOP_TIME_MIN/MAX_MS` | 100 / 30000 | 强拖加速时间 |
| `FOC_OPENLOOP_FREQ_MIN/MAX_HZ` | 60 / 500 | 强拖电频率范围 |
| `FOC_DEFAULT_TARGET_RPM` | 1500 | 上电默认速度目标（`FOC_Init` 写入；本工程副本进入 OPENLOOP 时不再覆盖上位机目标） |

### 6.3 SMO / 过渡

| 宏 | 默认 | 说明 |
|---|---|---|
| `FOC_SMO_LP_OMEGA_RAD_S` | 800 | 反电动势 LPF 截止角速度（rad/s，≈127 Hz） |
| `FOC_SMO_KSLIDE_MARGIN` | 2.0 | `Kslide = MARGIN × E_peak(max_rpm)` |
| `FOC_SMO_KSLIDE_MIN` | 0.5 | Kslide 下限 |
| `FOC_SMO_Z_MAX` | 12.0 | z 项 / Kslide 上限（V） |
| `FOC_SMO_CONV_E_MIN` | 0.3 | 收敛所需最小反电动势（V） |
| `FOC_SMO_CONV_RPM_MIN` | 800 | 收敛阈值上限（rpm） |
| `FOC_SMO_CONV_RATIO` | 0.85 | 收敛阈值 = `min(RPM_MIN, RATIO × 强拖转速)` |
| `FOC_SMO_CONV_CNT_MIN/MAX` | 10 / 1000 | 连续满足次数 / 计数上限 |
| `FOC_TRANS_ALPHA_MAX/STEP` | 0.999 / 0.001 | 过渡混合系数 |

> 低速配置：7 对极、60 Hz 对应约 514 rpm；按 `RATIO=0.85` 收敛阈值降到约 437 rpm，不必再把强拖频率硬拉到 93 Hz 以上。

### 6.4 可选项

| 宏 | 默认 | 说明 |
|---|---|---|
| `FOC_USE_CCM_RAM` | 关闭 | 核心状态/代码放 `.ccmram_*`（需链接脚本） |
| `FOC_PORT_HEADER` | `"FOC_PORT_STM32G4.h"` | 平台 port 头（大小写与实际文件名一致） |
| `FOC_PORT_TEST_MODE` | 未定义 | 定义后用空 port 跑 PC 单测 |
| `FOC_LEGACY_CONTROL` | 0（本工程当前） | 1=原版基线：Kslide 固定 1.0、前馈关闭；0=自动 Kslide/前馈 |

配置在编译期自检：PWM 频率/周期、控制频率一致性、分频比、极对数、过流确认次数等不合法会直接 `#error`；`FOC_VBUS_VOLTAGE`/`FOC_SW_OC_TRIP_A` 是浮点宏，请在修改时确保 > 0（运行时 `FOC_SetVbus` 会再次校验）。

---

## 7. 集成（STM32G4，RTOS 可选）

### 7.1 文件

```text
Inc/
├── FOC_CONFIG.h
├── FOC_CORE.h
├── FOC_PORT.h
└── FOC_PORT_STM32G4.h
Src/
├── FOC_CORE.c
└── FOC_PORT_STM32G4.c
```

> 文件名大小写与 `#include` 一致；移植到 Linux/CI 时不要改成小写。

### 7.2 CubeMX 前提

- **HRTIM**：TA1/TA2、TB1/TB2、TD1/TD2 输出；TIMER_B=U，TIMER_A=V，TIMER_D=W。
- **CORDIC**：`FUNCTION_PHASE / SCALE_0 / INSIZE_32BITS / OUTSIZE_32BITS / NBWRITE_2 / NBREAD_2 / PRECISION_6CYCLES`。
- **DWT**：应用在初始化时使能 `DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;`（无 DWT 平台让 `FOC_PORT_TS_*` 返回 0）。
- **ADC**：由应用层负责采样与换算，`FOC_Run` 只接收安培值。

### 7.3 初始化顺序

```c
void Peripheral_Init(void) {
    HAL_Init();
    SystemClock_Config();
    /* ... 各外设初始化，包括 MX_HRTIM_Init / MX_CORDIC_Init ... */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    FOC_Init();                   /* 必须在 HRTIM/CORDIC 之后 */
}
```

### 7.4 回调转发

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    FOC_Run(ADC_GetPhaseCurrentU(), ADC_GetPhaseCurrentV());
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx) {
    FOC_SpeedLoopTick();
}

void HAL_HRTIM_Fault1Callback(HRTIM_HandleTypeDef *hhrtim) {
    __HAL_HRTIM_CLEAR_FLAG(&hhrtim1, HRTIM_FLAG_FLT1);
    if (FOC_GetState() != FOC_STATE_IDLE) {
        FOC_OnFault(0x01U);       /* 或应用自定义故障码 */
        /* 通知应用层：UART/LED ... */
    }
}
```

### 7.5 上位机命令示例

```c
static void SetStartupCfg_Callback(uint8_t cmd, const uint8_t *data, uint8_t len) {
    if (len != sizeof(FOC_StartupConfig_t)) { /* 回错误 */ return; }
    FOC_Err_t e = FOC_SetStartupConfig((const FOC_StartupConfig_t *)data);
    /* 注意：协议层应先校验长度/字节序；不要直接把不可信数据当结构体 */
    (void)e;
}

static void StartMotor_Callback(uint8_t cmd, const uint8_t *data, uint8_t len) {
    FOC_Err_t e = FOC_Start(FOC_STATE_OPENLOOP);   /* 0x04：直接强拖 */
    /* e == FOC_OK / NOT_CONFIGURED / BUSY / HW_SELFTEST */
}
```

---

## 8. 移植到新平台

### 8.1 必需宏（9 个）

| 宏 | 作用 |
|---|---|
| `FOC_PORT_PWM_WRITE(a,b,c)` | 写三相 PWM 比较值（`uint16_t`） |
| `FOC_PORT_PWM_ENABLE()` | 使能 PWM 输出（并归零计数器） |
| `FOC_PORT_PWM_DISABLE()` | 关闭 PWM 输出 |
| `FOC_PORT_ENTER_CRITICAL()` | 保存并关闭 PRIMASK（展开为一条声明） |
| `FOC_PORT_EXIT_CRITICAL()` | 恢复 PRIMASK |
| `FOC_PORT_CORDIC_PHASE(x,y,s,m,a)` | 相位模式 CORDIC；无 CORDIC 用 `sqrtf + atan2f` |
| `FOC_PORT_CORDIC_SELFTEST()` | 返回 bool；无 CORDIC 返回 1 |
| `FOC_PORT_TS_BEGIN()` | 周期计数开始（无 DWT 返回 0） |
| `FOC_PORT_TS_END(t0)` | 自 t0 起的周期数（无 DWT 返回 0） |

可选：`FOC_PORT_CRITICAL_CHECK_NOT_ISR()`（v1.3 起核心层不再调用，仅为兼容保留）。

### 8.2 步骤

1. 复制 `FOC_CONFIG.h` / `FOC_CORE.h` / `FOC_PORT.h` / `FOC_CORE.c`（核心层，不改）。
2. 新建 `FOC_PORT_<平台>.h`，实现上表 9 个宏；无 CORDIC 平台用软件实现。
3. 修改 `FOC_CONFIG.h` 的 `FOC_PORT_HEADER` 指向新 port 头（或 `-DFOC_PORT_HEADER='"..."'`）。
4. 确认 `FOC_USE_CCM_RAM` 未定义（除非链接脚本支持）。
5. 逐文件编译：每个 `.c` 单独 `-fsyntax-only` 通过、头文件可自包含。

### 8.3 必查项

| 检查点 | 说明 |
|---|---|
| `FOC_PWM_PERIOD` | 与定时器实际周期一致，否则 SVPWM 缩放错误 |
| `FOC_VBUS_VOLTAGE` | 与硬件一致；运行时可 `FOC_SetVbus` |
| `FOC_MOTOR_POLE_PAIRS` | 仅作为上电默认值；运行时以 `FOC_MotorConfig_t.pole_pairs` 为准，必须与电机一致，否则转速/收敛阈值换算错误 |
| ADC 输入单位 | 必须是安培值 |
| CORDIC 模式 | 必须 PHASE / 32BITS / NBWRITE_2 / NBREAD_2 |
| DWT | 需应用使能；无 DWT 返回 0 |
| 临界区 | 必须在中断里也能安全保存/恢复 PRIMASK |

---

## 9. 使用注意事项

1. `FOC_Init()` 必须在 HRTIM/CORDIC 初始化之后调用；CORDIC 自检失败时 `FOC_Start` 返回 `FOC_ERR_HW_SELFTEST`。`FOC_Init` 之前调用 `FOC_Run` 是安全空操作，`FOC_Start` 返回 `FOC_ERR_INVALID_STATE`。
2. `FOC_Start` 前必须成功 `FOC_SetStartupConfig`。
3. `FOC_Start` 接受 `FOC_STATE_ALIGN` 或 `FOC_STATE_OPENLOOP`，内部自动清运行时状态、快照配置并使能 PWM。
4. `FOC_GetTelemetry` 可在任务或中断中调用（PRIMASK 临界区）。
5. 非 legacy 构建前馈默认开启，但只在 TRANSITION/CLOSEDLOOP 生效；ALIGN/OPENLOOP 不注入前馈。当前 legacy 构建前馈关闭。
6. `FOC_SetSpeedTarget` 会按新目标重算 Kslide；目标远超设计点时会受 `FOC_SMO_Z_MAX` 限制。
7. `FOC_SetVbus` 的更新在临界区内完成，ISR 不会看到半新半旧的缩放。
8. 软件过流阈值 `FOC_SW_OC_TRIP_A` 应按电机/驱动能力调整；默认 8 A 只适用于小功率云台类电机。
9. SVPWM 使用比较值反算实际 α/β 电压供 SMO，过调制/钳位时观测器输入更接近真实值；死区与母线纹波仍未建模。
10. `foc_dwt` 含中断抢占时间；评估算法耗时取最小值。
11. `.ccmram_*` 段默认关闭；开启需链接脚本定义。
12. 故障恢复：`FOC_OnFault` 后状态为 FAULT；`FOC_Start(ALIGN/OPENLOOP)` 清锁存并重启，但应先排查硬件根因。
13. 反转：SMO 在 `omega_est < 0` 时补偿 `atan2` 的 π 偏差；速度目标可正可负。
14. 修改 `FOC_DEFAULT_TARGET_RPM`/强拖频率会改变 Kslide 计算；三者是耦合参数。

---

## 10. 常见问题

### Q1：电机不转 / 电流为零
- 检查 `FOC_Start` 返回值：`NOT_CONFIGURED` / `BUSY` / `HW_SELFTEST`。
- 检查 `FOC_GetLastFault()`：`0xC0DE0001` 表示 CORDIC 模式错误。
- 检查 HRTIM 是否被应用层意外关闭（`FOC_Start` 会自动使能并归零计数器）。
- 检查 `FOC_Run` 是否被 ADC 回调转发。

### Q2：电机抖动 / 电流尖峰
- 检查 ADC 输入是否为安培值。
- 检查 `FOC_VBUS_VOLTAGE` 与实际母线一致，或调用 `FOC_SetVbus()`。
- 检查电流环 PI 参数；前馈现在只在闭环后生效，启动阶段不再注入。
- 查看 `smo.converged`、`smo.Kslide`、`smo.conv_rpm_min`。

### Q3：一直进不了闭环
- v1.3 起收敛阈值按强拖转速动态计算；若仍不收敛，检查 SMO 参数、极对数、相电流极性。
- 若 `FOC_GetLastFault()` 返回 `0xC0DE0002`，说明开环超时（默认 2×加速时间 + 500 ms），应加大强拖电流/时间或检查电机参数。
- 若返回 `0xC0DE0003`，检查是否触发软件过流（默认 8 A / 3 周期）。

### Q4：闭环后失控
- 检查极对数、相序、编码器（若有）方向。
- 检查 `smo.Kslide` 是否被钳在 `FOC_SMO_Z_MAX`。
- 检查 `FOC_SpeedLoopTick` 是否真的按 PWM 频率调用（`FOC_CTRL_FREQ_HZ` 必须等于 `FOC_PWM_FREQ_HZ`）。

### Q5：故障频繁触发
- 检查 HRTIM Fault 回调是否清除 FLT1 标志。
- 检查软件过流阈值是否过低（ADC 噪声/尖峰）。
- 检查 `FOC_Run` 输入是否有 NaN（会立即触发 `INVALID_INPUT`）。

---

## 11. 变更记录

### v1.3（2026-10-24，审查修复）
- **P0**：`FOC_PORT_STM32G4.h` 增加 `<stdbool.h>`，port `.c` 可单独编译。
- **P1**：新增故障锁存 + 状态迁移守卫，修复 `FOC_OnFault` 抢占后 FAULT 被覆盖。
- **P1**：收敛阈值按强拖转速动态计算（`FOC_SMO_CONV_RATIO`），修复 60~80 Hz 配置永远进不了闭环。
- **P1**：配置/输入 NaN/Inf 校验；`FOC_Run` 非法输入触发故障。
- **P1**：新增开环超时、软件过流保护及对应故障码。
- **P2**：前馈仅在 TRANSITION/CLOSEDLOOP 生效；Kslide 按运行范围重算；SMO 使用实际施加电压；速度环 dt 改用 PWM 频率。
- **P2**：临界区改为 PRIMASK，去除 FreeRTOS 依赖；`FOC_GetTelemetry` 可在中断中调用。
- **P2**：CORDIC 自检失败阻止启动并保留故障码；`FOC_PORT.h` 自包含；include 大小写统一。
- **P3**：CORDIC 超时 10000→200 次；HRTIM 启动归零计数器；清理 `theta_elec`/`SVPWM_SCALE`；`FOC_PWM_DT_S` 更名；配置编译期自检；补齐编译告警。
- **文档**：README 重写；文件名 `READM.md` → `README.md`；补充故障码、保护、移植与验收说明。

### v1.2（2026-10-23）
- TEST_MODE 修复；`speed_rpm` 饱和；Kslide 自动计算；TRANSITION 更新 theta_comp；ISR 断言；`foc_state` volatile；限幅 API；`FOC_SetVbus`；清理死字段；`align_target_cycles` 32 位；文档修正。

### v1.1（2026-10-23）
- 前馈公式修正；CORDIC Q1.31 饱和；Deinit 关 PWM；CORDIC RRDY 超时；CORDIC 模式自检；`FOC_PORT_PWM_ENABLE`；反转 π 补偿；遥测电流饱和；速度环计数 uint16；常数统一。

### v1.0
- 从原工程 `FOC.c` 拆分为核心层 + 硬件层；单实例；端口宏；配置锁；自动状态机；故障入口；CORDIC 直写；DWT 测时；CCM 开关；错误码。

---

## 附录 A：API 快速索引

```c
void        FOC_Init(void);
void        FOC_Deinit(void);
FOC_Err_t   FOC_SetStartupConfig(const FOC_StartupConfig_t *cfg);
FOC_Err_t   FOC_GetStartupConfig(FOC_StartupConfig_t *cfg);
FOC_Err_t   FOC_Start(FOC_State_t mode);
void        FOC_Stop(void);
void        FOC_OnFault(uint32_t code);
uint32_t    FOC_GetLastFault(void);
void        FOC_Run(float Iu_A, float Iv_A);
void        FOC_SpeedLoopTick(void);
void        FOC_SetSpeedTarget(int16_t rpm);
void        FOC_SetCurrentTarget(float iq_a);
void        FOC_SetSpeedPID(float kp, float ki, float kd);
void        FOC_SetCurrentPID(float kp_d, float ki_d, float kp_q, float ki_q);
void        FOC_GetSpeedPID(float *kp, float *ki, float *kd);
void        FOC_GetCurrentPID(float *kp_d, float *ki_d, float *kp_q, float *ki_q);
void        FOC_SetSpeedLimits(float out_min, float out_max, float integ_min, float integ_max);
void        FOC_SetCurrentLimits(float out_min, float out_max, float integ_min, float integ_max);
void        FOC_SetVbus(float vbus);
FOC_State_t FOC_GetState(void);
void        FOC_GetTelemetry(FOC_Telemetry_t *tele);
void        FOC_SetCycleCount(uint32_t cycles);
void        Cordic_Phase(float x, float y, float scale, float *mag, float *angle);
```

## 附录 B：状态迁移表

| 当前状态 | 触发 | 下一状态 | 副作用 |
|---|---|---|---|
| IDLE | `FOC_Start(ALIGN)` | ALIGN | 快照配置、清运行时状态、使能 PWM（计数器归零） |
| IDLE | `FOC_Start(OPENLOOP)` | OPENLOOP | 快照配置、清运行时状态、初始化 IF、直接使能 PWM |
| ALIGN | 计时到 `align_time_ms` | OPENLOOP | 切 IF 给定、算动态收敛阈值与超时、清 SMO 角度/速度 |
| OPENLOOP | `smo.converged` | TRANSITION | α=0 |
| TRANSITION | α≥0.999 | CLOSEDLOOP | 速度环接管 |
| CLOSEDLOOP | `FOC_Stop()` | IDLE | 关 PWM、清锁存、清运行时状态/遥测 |
| 任意运行态 | `FOC_OnFault(code)` | FAULT | 关 PWM、锁存、清运行时状态/遥测、记录码 |
| FAULT | `FOC_Start(ALIGN)` 或 `FOC_Start(OPENLOOP)` | ALIGN/OPENLOOP | 清锁存、重新使能 PWM |

## 附录 C：v1.3 行为变化

| 场景 | v1.2 | v1.3 |
|---|---|---|
| 60 Hz 强拖 | 收敛阈值 800 rpm，可能永不闭环 | 阈值 ≈437 rpm，可闭环 |
| 高目标转速 | Kslide 固定 2.64 V，可能饱和 | 按 max_rpm 重算，上限 Z_MAX |
| 前馈 | ALIGN/OPENLOOP 也生效 | 仅 TRANSITION/CLOSEDLOOP |
| 故障抢占 | FAULT 可能被覆盖 | 锁存 + 迁移守卫 |
| NaN 配置 | 通过校验 | 拒绝并返回参数错误 |
| 开环不收敛 | 无限强拖 | 超时 → FAULT |
| 过流 | 仅硬件保护 | 软件过流 → FAULT |
| 遥测读取 | 仅任务上下文 | 任务/中断均可 |
| FreeRTOS | port 依赖 taskENTER_CRITICAL | PRIMASK，无 RTOS 依赖 |
| CORDIC 自检失败 | 仅记录，仍可启动 | `FOC_Start` 返回 HW_SELFTEST |
| HRTIM 启动 | 不归零计数器 | 先归零再使能 |
