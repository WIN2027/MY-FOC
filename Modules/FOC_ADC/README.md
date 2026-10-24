# ADC 滤波与电流换算模块（FOC_ADC）

> **适用场景**：无感/有感 FOC 工程中，把 ADC 原始码值转换为相电流 (A)。
> **模块性质**：纯算法模块，不含 ADC 外设初始化，与 HAL 解耦，只依赖 C99 标准库。

> v1.3（2026-10-24）：修复与优化版本。
> **文件已从 `ADC.h/.c` 重命名为 `FOC_ADC.h/.c`**，避免与工程已有的 `DOC/Inc/ADC.h`、`DOC/Src/ADC.c` 冲突。

---

## 1. 功能与处理链

```
ADC 原始码值 (uint16_t)
        │
        ▼
┌───────────────────┐
│ 1. 三点中值滤波    │  剔除单次毛刺（如开关噪声尖峰）
└───────────────────┘
        │
        ▼
┌───────────────────┐
│ 2. 一阶 IIR 低通   │  平滑高频噪声，系数可在线调整
└───────────────────┘
        │
        ▼
┌───────────────────┐
│ 3. 码值 → 电流换算 │  current = (filtered - midpoint) * scale
└───────────────────┘
        │
        ▼
   相电流 (float, A)
```

### 1.1 设计原则

| 原则 | 说明 |
|---|---|
| **无硬件依赖** | 不调用 `HAL_ADC_*`，不访问寄存器，只接收 `uint16_t` |
| **FOC 专用输出** | `ADC_Filter_Update()` 直接返回电流 (A) |
| **多实例** | 每个滤波器独立结构体，天然支持 Ia/Ib/Ic 多相 |
| **可配置** | IIR 系数、零点、比例系数均可在运行期修改 |
| **输入防护** | `alpha` / `midpoint` / `current_scale` 全部 sanitize |
| **状态防护** | 内部 `iir_prev` 若被异常写成 NaN，`Update` 会用中点恢复 |
| **内建自检** | `ADC_SelfTest()` 覆盖零点、尖峰、NaN、Reset、中值、alpha 边界、负 scale 等 12 组回归 |
| **CCM 可选** | 数据与代码可**独立**放入 CCM；不再用“一个宏同时搬数据和代码”的危险做法 |
| **可裁剪** | `ADC_FILTER_ENABLE_SELFTEST=0` 可去掉自检代码 |

---

## 2. 文件结构

```text
FOC_ADC/
├── FOC_ADC.h      # 对外 API、句柄、编译期宏
├── FOC_ADC.c      # 实现与自检
└── README.md
```

> **为什么改名**：原工程的 `DOC/Inc/ADC.h`、`DOC/Src/ADC.c` 与模块同名；若把模块文件拷入工程，可能覆盖原文件或让 `#include "ADC.h"` 命中错误头文件。改为 `FOC_ADC.*` 后，函数名仍为 `ADC_Filter_*`，但文件名不再冲突。

---

## 3. 数据结构

```c
typedef struct {
    /* 可配置参数 */
    float alpha;           // IIR 系数 [0,1]
    float one_minus_alpha; // 预存 1-alpha（内部）
    float midpoint;        // 零电流中点码值
    float current_scale;   // 码值 → 电流系数 (A/LSB)，负值合法

    /* 内部状态 */
    uint16_t window[3];    // 三点中值窗口（[0] 最旧，[2] 最新）
    float    iir_prev;     // IIR 上一拍输出（码值域）
    float    current;      // 最近一次电流 (A)
    bool     inited;
} ADC_Filter_t;
```

- 单实例 `sizeof` = **36 字节**（GCC 13 / Cortex-M4 实测）。
- 结构体可放普通 RAM 或 CCM；不要放 DMA 缓冲区（CCM 本身不可 DMA，模块也不要求）。

---

## 4. 编译期配置

| 宏 | 默认 | 说明 |
|---|---|---|
| `ADC_FILTER_DEFAULT_MIDPOINT` | `2048.0f` | NaN midpoint 的兜底值；16 位 ADC 建议 `32768.0f` |
| `ADC_FILTER_ENABLE_SELFTEST` | `1` | 1=编译 `ADC_SelfTest`；0=裁剪（省约 550 字节 Flash） |
| `ADC_FILTER_DATA_CCM` | 未定义 | 把 `ADC_Filter_t` 实例放 `.ccmram_data` |
| `ADC_FILTER_TEXT_CCM` | 未定义 | 把模块函数放 `.ccmram_text`（需芯片支持 + 复制） |
| `ADC_FILTER_USE_CCM` | 未定义 | 旧宏：等价于同时定义 DATA_CCM + TEXT_CCM，仅为兼容保留 |

> **F4 的 F401/F411/F427/F429 等 CCM 只连 D-bus 的型号，只能用 `ADC_FILTER_DATA_CCM`，绝不能定义 `ADC_FILTER_TEXT_CCM` / `ADC_FILTER_USE_CCM`**，否则执行模块函数会 HardFault。

---

## 5. API

### 5.1 初始化

```c
void ADC_Filter_Init(ADC_Filter_t *f,
                     float alpha,
                     float midpoint,
                     float current_scale);
```

- `alpha` 限幅到 `[0,1]`；NaN 视为 0；负值视为 0；同步刷新 `one_minus_alpha`。
- `midpoint`：NaN → `ADC_FILTER_DEFAULT_MIDPOINT`；负值 → 0；>65535 → 65535。
- `current_scale`：NaN / ±inf / 极端值 → 0；负值保留（传感器反接）。
- `window` 以 `midpoint` 四舍五入后的整数码初始化；`iir_prev` 用 sanitize 后的 `midpoint`。
- 若 `midpoint` 为整数，首拍输出严格为 0；若为小数，首拍输出反映该小数与整数码之间的物理偏差。

### 5.2 主更新

```c
float ADC_Filter_Update(ADC_Filter_t *f, uint16_t raw);
```

- 返回滤波后的电流 (A)。
- 在 ADC DMA 完成回调中调用。
- **非线程安全**；不要和 `SetXxx` 跨上下文并发调用。
- 内部增加一次 NaN 状态检查：若 `iir_prev` 被异常写成 NaN，会先用 `midpoint` 恢复，再参与滤波。

### 5.3 运行时配置

```c
void ADC_Filter_SetAlpha   (ADC_Filter_t *f, float alpha);         // NaN→0，限幅 [0,1]
void ADC_Filter_SetMidpoint(ADC_Filter_t *f, float midpoint);      // NaN→默认中点，并重置内部状态
void ADC_Filter_SetScale   (ADC_Filter_t *f, float current_scale); // NaN/±inf→0，负值保留
```

- `SetMidpoint` 会同步重置 `window` / `iir_prev` / `current`，新零点立刻生效，无需再 `Reset`。
- 跨上下文修改（如主循环校准 vs ADC 中断）必须加临界区，或先停 ADC 触发。

### 5.4 复位与只读访问

```c
void  ADC_Filter_Reset(ADC_Filter_t *f);                       // 清状态，保留配置
float ADC_Filter_GetCurrent(const ADC_Filter_t *f);            // 最近一次电流 (A)
float ADC_Filter_GetFilteredCode(const ADC_Filter_t *f);       // 最近一次滤波码值
```

### 5.5 辅助函数

```c
uint16_t ADC_Median3(uint16_t a, uint16_t b, uint16_t c);  // 三点中值
bool     ADC_SelfTest(void);                               // 自检（可裁剪）
```

- `ADC_Median3` 是排序网络，通常编译为条件传送；`Update` 内部使用同一套 inline 实现，避免 48 kHz 热路径的函数调用。
- `ADC_SelfTest` 覆盖 12 组用例：静止零点、单拍尖峰、NaN midpoint、NaN alpha、NaN/inf scale、中值排序、Reset、alpha=0、alpha=1、负 scale、内部 NaN 恢复、NULL/未初始化保护。
- 自检函数本身固定在 `.text`；但它调用的 `ADC_Filter_*` 在启用 `TEXT_CCM` 时位于 CCM，见 §8。

---

## 6. 集成

### 6.1 声明实例

```c
#include "FOC_ADC.h"

ADC_FILTER_DATA ADC_Filter_t g_adc1_filter;   // U 相
ADC_FILTER_DATA ADC_Filter_t g_adc2_filter;   // V 相
```

### 6.2 初始化

```c
void FOC_ADC_Init(void)
{
    ADC_Filter_Init(&g_adc1_filter, 0.7f, ADC_MIDPOINT, FOC_CURRENT_SCALE);
    ADC_Filter_Init(&g_adc2_filter, 0.7f, ADC_MIDPOINT, FOC_CURRENT_SCALE);
}
```

### 6.3 ADC DMA 完成回调

```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) ADC1_Ready = true;
    if (hadc->Instance == ADC2) ADC2_Ready = true;

    if (ADC1_Ready && ADC2_Ready) {
        ADC1_Ready = false;
        ADC2_Ready = false;

        const float IU = ADC_Filter_Update(&g_adc1_filter, ADC1_DMA_BUF);
        const float IV = ADC_Filter_Update(&g_adc2_filter, ADC2_DMA_BUF);

        FOC_Run(IU, IV);       /* FOC_Run 入参已经是安培值 */
    }
}
```

### 6.4 上层 FOC_Run 不要再换算

```c
/* 删除旧代码：(raw - ADC_MIDPOINT) * FOC_CURRENT_SCALE */
const float ia = Iu;   /* 已经是 A */
const float ib = Iv;
```

### 6.5 遥测

```c
static uint16_t clamp_u16(float v)
{
    if (!(v > 0.0f))    return 0u;
    if (v >= 65535.0f)  return 65535u;
    return (uint16_t)(v + 0.5f);
}

const uint16_t Tamp = clamp_u16(ADC_Filter_GetFilteredCode(&g_adc1_filter) * 16.0f);
```

> 12 位 ADC：4095 × 16 = 65520 < 65535，2 字节报文刚好不溢出；升级到 16 位 ADC 时请改成 4 字节。

---

## 7. 多相 / 多电机

模块天然支持多实例。三电阻 + 独立 W 相：

```c
ADC_FILTER_DATA ADC_Filter_t g_adc3_filter;

ADC_Filter_Init(&g_adc3_filter, 0.7f, ADC_MIDPOINT, FOC_CURRENT_SCALE);
const float IW = ADC_Filter_Update(&g_adc3_filter, ADC3_DMA_BUF);
```

两路电机共用 ADC 时，声明两套实例即可，实例之间无共享状态。

---

## 8. CCM 优化（可选）

### 8.1 数据与代码分开控制

| 宏 | 作用 | 适用 |
|---|---|---|
| `ADC_FILTER_DATA_CCM` | `ADC_Filter_t` 实例放 `.ccmram_data` | 几乎所有 STM32 |
| `ADC_FILTER_TEXT_CCM` | 模块函数放 `.ccmram_text` | 仅 CCM 连 I-bus 的型号，且需复制代码 |
| `ADC_FILTER_USE_CCM` | 旧宏，等价于两者都开 | 兼容旧工程；F4 D-bus-only 型号禁止使用 |

**典型配置**：

```makefile
# F401/F411/F427/F429：只能放数据
CFLAGS += -DADC_FILTER_DATA_CCM

# G4/F303/F334：可放数据 + 代码（仍需复制代码）
CFLAGS += -DADC_FILTER_USE_CCM
```

### 8.2 CCM 能否执行代码

| 系列 | CCM 连 I-Bus | 能否执行 |
|---|---|---|
| STM32G4 | ✅ | 可以 |
| STM32F303 / F334 | ✅ | 可以 |
| STM32F401 / F411 | ❌ | 不可以 |
| STM32F427 / F429 | ❌ | 不可以 |

判断依据：RM 中 CCM 是否描述为 “connected to I-bus”。

### 8.3 链接与复制

`.ccmram_text` 必须 **LMA 在 Flash、VMA 在 CCM**，并在启动阶段复制。示例链接脚本：

```ld
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCM   (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
}

SECTIONS
{
    .ccmram_data (NOLOAD) :
    {
        . = ALIGN(4);
        *(.ccmram_data)
        *(.ccmram_data*)
        . = ALIGN(4);
    } > CCM

    .ccmram_text :
    {
        . = ALIGN(4);
        _siccmram_text = LOADADDR(.ccmram_text);
        _sccmram_text  = ADDR(.ccmram_text);
        *(.ccmram_text)
        *(.ccmram_text*)
        . = ALIGN(4);
        _eccmram_text  = ADDR(.ccmram_text) + SIZEOF(.ccmram_text);
    } > CCM AT > FLASH
}
```

```c
extern uint32_t _siccmram_text, _sccmram_text, _eccmram_text;

static void Copy_CCM_Text(void)
{
    uint32_t *src = &_siccmram_text;
    uint32_t *dst = &_sccmram_text;
    while (dst < &_eccmram_text) *dst++ = *src++;
}
```

复制必须在任何 CCM 函数被调用之前完成。

### 8.4 自检与 CCM 的关系

`ADC_SelfTest` 本身在 `.text`，但它调用 `ADC_Filter_Init/Update/...`。若这些函数被放入 CCM：

- 必须先完成 CCM 代码复制，再调用 `ADC_SelfTest`；
- 或者构建一个**不启用 `ADC_FILTER_TEXT_CCM`** 的版本专门做启动自检（`ADC_FILTER_ENABLE_SELFTEST` 保持 1）；
- 或者只启用 `ADC_FILTER_DATA_CCM`，不把代码放 CCM。

---

## 9. 零电流校准

推荐流程：停 PWM → 采样 N 次 → 求均值 → `ADC_Filter_SetMidpoint()`（自动重置）。

```c
float ADC_Filter_CalibrateMidpoint(ADC_Filter_t *f,
                                   const uint16_t *samples,
                                   uint32_t n)
{
    if (f == NULL || samples == NULL || n == 0) {
        return (f != NULL) ? f->midpoint : 0.0f;
    }
    uint32_t sum = 0;
    for (uint32_t i = 0; i < n; i++) sum += samples[i];
    const float avg = (float)sum / (float)n;
    ADC_Filter_SetMidpoint(f, avg);
    return avg;
}
```

低 RAM 的累积式实现：

```c
typedef struct { uint32_t sum, count, target; } ADC_CalibCtx_t;

static inline void ADC_Calib_Begin(ADC_CalibCtx_t *c, uint32_t target)
{ c->sum = 0; c->count = 0; c->target = target; }

static inline bool ADC_Calib_Feed(ADC_CalibCtx_t *c, uint16_t raw)
{
    if (c->count < c->target) { c->sum += raw; c->count++; }
    return (c->count >= c->target);
}

static inline float ADC_Calib_GetAvg(const ADC_CalibCtx_t *c)
{ return (c->count > 0) ? ((float)c->sum / (float)c->count) : 0.0f; }
```

> 样本数建议 256~1024；`sum` 用 `uint32_t` 时注意 `n × 65535 < 2^32`。

---

## 10. 参数整定

### 10.1 IIR 系数

`y[n] = α·x[n] + (1-α)·y[n-1]` 的 -3 dB 截止频率：

```
cos(ωc) = [1 + (1-α)² − 2α²] / [2(1-α)]
fc = ωc / (2π) × fs
```

fs = 48 kHz 时：

| α | fc | 特点 |
|---|---|---|
| 0.9 | 无 -3 dB 点（Nyquist 约 -1.74 dB） | 几乎不平滑，响应极快 |
| 0.7 | ≈10.6 kHz | 原工程默认 |
| 0.5 | ≈5.5 kHz | 均衡 |
| 0.3 | ≈2.8 kHz | 高平滑 |
| 0.1 | ≈0.8 kHz | 极平滑，滞后大 |

> 不要用 `fc ≈ -ln(1-α)·fs/(2π)` 反推大 α 的截止频率，误差很大。

### 10.2 中值窗口

固定 3 点，只能剔除单拍脉冲；连续多拍噪声需要降低 α、优化采样点或加硬件 RC。
另外，3 点中值会引入约 1 个采样周期的等效延迟（fs=48 kHz 约 20.8 µs），电流环相位裕度设计应计入。

### 10.3 current_scale 校准

1. 电机堵转，施加已知电流 `I_ref`；
2. 读 `ADC_Filter_GetFilteredCode()`；
3. `scale = I_ref / (code - midpoint)`；
4. 多次平均。

若 `code < midpoint` 时 `I_ref > 0`，scale 为负，模块允许。

---

## 11. 移植检查清单

- [ ] 编译器支持 C99；
- [ ] 包含的是 `FOC_ADC.h`（不是工程里旧的 `ADC.h`）；
- [ ] `ADC_Filter_t` 实例数量与相数一致；
- [ ] `midpoint` / `current_scale` 已标定；
- [ ] 只在需要时定义 `ADC_FILTER_DATA_CCM`；
- [ ] 只有 CCM 可执行且已复制代码时，才定义 `ADC_FILTER_TEXT_CCM`；
- [ ] 16 位 ADC 已把 `ADC_FILTER_DEFAULT_MIDPOINT` 改为 32768；
- [ ] 启动时调用 `ADC_SelfTest()`（或量产裁剪）；
- [ ] 上层 `FOC_Run` 已删除二次换算；
- [ ] 遥测使用 `ADC_Filter_GetFilteredCode` 并 clamp；
- [ ] `SetXxx` 与 `Update` 跨上下文时已加临界区。

---

## 12. 常见问题

**Q1：电机一启动就过流，但采样读数很小？**
检查 `FOC_Run` 是否还保留了 `(raw - midpoint) * scale` 的二次换算。

**Q2：静止时电流不为 0？**
做零电流校准，`ADC_Filter_SetMidpoint()` 会自动重置状态。

**Q3：毛刺滤不掉？**
3 点中值只滤单拍；降低 α、优化采样时刻或加硬件滤波。

**Q4：编译报 `unknown type name ADC_Filter_t`？**
确认包含的是 `FOC_ADC.h`，且该路径在 include 搜索路径中。

**Q5：启用 CCM 后 HardFault？**
先确认芯片 CCM 是否连 I-bus；再确认 `.ccmram_text` 已 LMA/VMA 分离并完成复制。F4 D-bus-only 型号只能用 `ADC_FILTER_DATA_CCM`。

**Q6：`ADC_SelfTest` 在 CCM 代码未复制时调用会怎样？**
`ADC_SelfTest` 本身在 `.text`，但它会调用 CCM 中的 `ADC_Filter_*`，仍会 HardFault。先复制代码，或用不启用 TEXT_CCM 的构建做自检。

**Q7：`SetMidpoint` 后输出为什么立即归零？**
设计如此：同步重置窗口/IIR/current，让新零点立刻生效。

**Q8：NaN 中点会怎样？**
`Init` / `SetMidpoint` 会归到 `ADC_FILTER_DEFAULT_MIDPOINT`（默认 2048），不会污染 IIR。

---

## 13. 版本历史

| 版本 | 日期 | 说明 |
|---|---|---|
| v1.0 | — | 从原工程 `ADC.c` 抽取重构 |
| v1.1 | — | `midpoint_to_code` 越界 UB、NaN 防护、`SetMidpoint` 重置、README 频率表 |
| v1.2 | — | NaN midpoint/scale 防护、`one_minus_alpha` 预存、`ADC_SelfTest`、函数声明段属性 |
| **v1.3** | **2026-10-24** | 文件改名 `FOC_ADC.*` 避免与工程 `ADC.*` 冲突；CCM 数据/代码拆分为 `DATA_CCM` / `TEXT_CCM`（旧宏兼容）；新增 `ADC_FILTER_DEFAULT_MIDPOINT`、`ADC_FILTER_ENABLE_SELFTEST`；自检扩展到 12 组；`ADC_Median3` inline 热路径；NaN 比较改为显式 `!=` 判断（兼容对 `!(x>=0)` 处理不一致的编译器）；`Update` 增加 NaN 状态恢复；README 重写 |

---

## 附录 A：与原工程对应关系

| 原工程 | 本模块 |
|---|---|
| `ADC1_BUF[3]` / `ADC2_BUF[3]` | `ADC_Filter_t.window[3]`（每实例） |
| `ADC1_Prev` / `ADC2_Prev` | `ADC_Filter_t.iir_prev`（每实例） |
| `ADC1_Filter()` / `ADC2_Filter()` | `ADC_Filter_Update()` |
| `ADC_Init()` | `ADC_Filter_Init()`（每实例） |
| `(Iu - ADC_MIDPOINT) * FOC_CURRENT_SCALE` | 内置于 `ADC_Filter_Update` |
| `__attribute__((section(".ccmram_*")))` | `ADC_FILTER_DATA_CCM` / `ADC_FILTER_TEXT_CCM` |
| 全局变量直读 | `ADC_Filter_GetCurrent/GetFilteredCode` |

## 附录 B：内存与性能

| 项目 | 数值 |
|---|---|
| `ADC_Filter_t` 单实例 | 36 B |
| `.text`（含自检） | 约 1.6 KB（GCC 13.3 `-O2`） |
| `.text`（`ENABLE_SELFTEST=0`） | 约 1.0 KB |
| `ADC_Filter_Update` 栈 | 24 B |
| `ADC_SelfTest` 栈 | 64 B |
| `ADC_Filter_Update` 时间 | 约 30~60 周期（估算，需目标板 DWT 复核） |

**运行验证**：`ADC_SelfTest()` 在 TCC 0.9.27 (x64) 上通过；另用 20 万组随机参数（含 NaN/Inf/负 scale/0/1 alpha/16 位中点）×20 次更新做属性测试，输出均为有限值，`current == (filtered - midpoint) * scale` 严格成立。
