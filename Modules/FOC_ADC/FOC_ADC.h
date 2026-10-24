/**
 * @file    FOC_ADC.h
 * @brief   可移植 ADC 滤波 + 电流换算模块（FOC 专用）
 * @note    不涉及 ADC 硬件初始化；输入原始 ADC 码值，输出电流 (A)。
 *          处理流程：3 点中值滤波 → 一阶 IIR 低通 → (code-mid)*scale。
 *
 * 依赖：C99 标准库 <stdint.h> <stdbool.h>，适配任何 HAL 平台的 ADC_DMA_BUF。
 *
 * 编译期配置（均可通过 -D 覆盖）：
 *   ADC_FILTER_DEFAULT_MIDPOINT   NaN midpoint 的兜底值，默认 2048.0f（12 位）；
 *                                 16 位 ADC 建议设为 32768.0f。
 *   ADC_FILTER_ENABLE_SELFTEST    1=编译 ADC_SelfTest（默认）；0=裁剪。
 *   ADC_FILTER_DATA_CCM           把 ADC_Filter_t 实例放 .ccmram_data。
 *   ADC_FILTER_TEXT_CCM           把模块函数放 .ccmram_text（需芯片支持 + 复制）。
 *   ADC_FILTER_USE_CCM            兼容旧宏：等价于同时定义 DATA_CCM + TEXT_CCM。
 *
 * ⚠️ CCM 使用边界（务必阅读）：
 *   ─ 数据（ADC_Filter_t 实例）放 CCM：F4/G4 均可，可降低访问延迟；
 *     CCM 不能被 DMA 访问，但本模块的输入是 ADC DMA 缓冲的副本，无冲突。
 *   ─ 代码（.ccmram_text）放 CCM：
 *       • F4 部分型号（F401/F411/F427/F429 等）：CCM 仅连 D-bus，无指令总线，
 *         执行代码会触发 HardFault，请勿定义 ADC_FILTER_TEXT_CCM。
 *       • G4 / F303 / F334：CCM 连 I-bus + D-bus，可执行代码。
 *     判断依据：查阅对应芯片 Reference Manual 中 CCM 是否 "connected to I-bus"。
 *   ─ 若启用 TEXT_CCM，.ccmram_text 段通常 LMA 在 Flash、VMA 在 CCM，
 *     必须在初始化阶段（startup 或 main 开头）把代码从 Flash 复制到 CCM，
 *     详见 README §7。
 *   ─ ADC_SelfTest 本身固定在 .text，但它会调用 ADC_Filter_Init/Update；
 *     若这些函数被放入 .ccmram_text，调用前必须先完成 CCM 代码复制，
 *     或构建仅 DATA_CCM 的版本做自检。
 */

#ifndef ADC_FILTER_H
#define ADC_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 编译期配置 ---------- */
#ifndef ADC_FILTER_DEFAULT_MIDPOINT
#define ADC_FILTER_DEFAULT_MIDPOINT  2048.0f   /* 12 位 ADC 默认中点；16 位改 32768.0f */
#endif

#ifndef ADC_FILTER_ENABLE_SELFTEST
#define ADC_FILTER_ENABLE_SELFTEST   1
#endif

/* 旧宏兼容：ADC_FILTER_USE_CCM = 数据 + 代码都放 CCM。
 * 仅想放数据时请用 ADC_FILTER_DATA_CCM，避免在不支持 CCM 取指的芯片上 HardFault。 */
#if defined(ADC_FILTER_USE_CCM)
    #ifndef ADC_FILTER_DATA_CCM
        #define ADC_FILTER_DATA_CCM
    #endif
    #ifndef ADC_FILTER_TEXT_CCM
        #define ADC_FILTER_TEXT_CCM
    #endif
#endif

/* ---------- 可选的 CCM 段修饰（默认关闭，保持平台无关） ---------- */
#ifdef ADC_FILTER_DATA_CCM
    #define ADC_FILTER_DATA  __attribute__((section(".ccmram_data")))
#else
    #define ADC_FILTER_DATA
#endif

#ifdef ADC_FILTER_TEXT_CCM
    #define ADC_FILTER_TEXT  __attribute__((section(".ccmram_text")))
#else
    #define ADC_FILTER_TEXT
#endif

/* ---------- 滤波器句柄 ---------- */
typedef struct {
    /* ---- 可配置参数（运行时可修改） ---- */
    float alpha;           /* IIR 系数，[0,1]；越大跟随越快，越小越平滑 */
    float one_minus_alpha; /* 预存 1-alpha（内部，勿手改；Update 热路径直接使用） */
    float midpoint;        /* ADC 中点码值（对应零电流），12 位通常 2048 */
    float current_scale;   /* 码值 → 电流比例系数 (A/LSB)；负值合法（传感器反接） */

    /* ---- 内部状态（勿手动修改） ---- */
    uint16_t window[3];    /* 三点中值窗口（[0]最旧，[2]最新） */
    float    iir_prev;     /* IIR 上一拍输出（ADC 码值域） */
    float    current;      /* 最近一次输出的电流值 (A) */
    bool     inited;
} ADC_Filter_t;

/* ---------- API ---------- */

/* 注意：以下声明的 ADC_FILTER_TEXT 必须与 FOC_ADC.c 中的定义保持一致，
 *       否则在 LTO / 跨 TU 优化时可能出现段属性不匹配的告警或优化失效。
 *       例外：ADC_SelfTest 刻意不加 ADC_FILTER_TEXT，保证测试函数本身落
 *       在 .text；但其内部调用的滤波器 API 若在 CCM，仍需先复制代码。 */

/**
 * @brief 初始化滤波器实例
 * @param f             滤波器句柄
 * @param alpha         IIR 系数 [0,1]；越界自动限幅，NaN 视为 0
 * @param midpoint      ADC 中点码值；NaN → ADC_FILTER_DEFAULT_MIDPOINT（默认 2048），
 *                      负值 → 0，超过 65535 → 65535
 * @param current_scale 码值 → 电流系数；NaN / ±inf → 0，负值保留（传感器反接）
 * @note  window 以 midpoint 四舍五入值初始化，iir_prev 用 sanitize 后的 midpoint；
 *        若 midpoint 为整数，首拍输出严格为 0；若为小数，首拍输出反映该小数
 *        与整数码之间的物理偏差（可通过精确校准消除）。
 */
ADC_FILTER_TEXT
void ADC_Filter_Init(ADC_Filter_t *f, float alpha, float midpoint, float current_scale);

/** @brief 运行时修改 IIR 系数（限幅到 [0,1]，NaN 视为 0），同步刷新 one_minus_alpha */
ADC_FILTER_TEXT
void ADC_Filter_SetAlpha(ADC_Filter_t *f, float alpha);

/**
 * @brief 运行时修改中点（例如零电流校准后写入）
 * @note  入参经 sanitize：NaN → ADC_FILTER_DEFAULT_MIDPOINT，负值 → 0，>65535 → 65535。
 *        若实例已 inited，会同步重置 window / iir_prev / current，
 *        使新零点立刻生效，无需再手动调用 ADC_Filter_Reset()。
 *        ⚠️ 与 ADC_Filter_Update 跨上下文并发调用时需加临界区保护。
 */
ADC_FILTER_TEXT
void ADC_Filter_SetMidpoint(ADC_Filter_t *f, float midpoint);

/** @brief 运行时修改比例系数（NaN / ±inf → 0；负值保留） */
ADC_FILTER_TEXT
void ADC_Filter_SetScale(ADC_Filter_t *f, float current_scale);

/** @brief 复位内部状态（窗口、IIR、输出电流都归零；保留配置参数） */
ADC_FILTER_TEXT
void ADC_Filter_Reset(ADC_Filter_t *f);

/**
 * @brief 【主 API】输入 ADC 原始码值，更新滤波并返回电流值 (A)
 * @param f    滤波器句柄
 * @param raw  原始 ADC 码值（如 ADC1_DMA_BUF）
 * @return     滤波后的电流值 (A)，零电流为 0.0f
 * @note  非线程安全；请与 ADC_Filter_SetXxx 在同一上下文调用，或加临界区。
 */
ADC_FILTER_TEXT
float ADC_Filter_Update(ADC_Filter_t *f, uint16_t raw);

/** @brief 只读访问：最近一次输出的电流值 (A) */
ADC_FILTER_TEXT
float ADC_Filter_GetCurrent(const ADC_Filter_t *f);

/** @brief 只读访问：最近一次滤波后的 ADC 码值（float 域，用于遥测） */
ADC_FILTER_TEXT
float ADC_Filter_GetFilteredCode(const ADC_Filter_t *f);

/** @brief 三点中值排序辅助函数（排序网络，通常由编译器转为条件传送） */
ADC_FILTER_TEXT
uint16_t ADC_Median3(uint16_t a, uint16_t b, uint16_t c);

#if ADC_FILTER_ENABLE_SELFTEST
/**
 * @brief 模块自检（无外部依赖；可在启动或单元测试中调用）
 * @return true 全部通过；false 有断言失败
 * @note  覆盖：静止零点、单拍尖峰抑制、NaN midpoint/alpha/scale、Reset、
 *        中值排序、alpha=0/1、负 scale。
 *        本函数本身不加 ADC_FILTER_TEXT，永远落在 .text；但它调用的
 *        ADC_Filter_* 在启用 TEXT_CCM 时位于 CCM，需先完成代码复制。
 *        量产可通过 ADC_FILTER_ENABLE_SELFTEST=0 裁剪（约 550 字节 Flash）。
 */
bool ADC_SelfTest(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ADC_FILTER_H */
