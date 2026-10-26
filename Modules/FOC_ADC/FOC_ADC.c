/**
 * @file    FOC_ADC.c
 * @brief   可移植 ADC 滤波 + 电流换算（无硬件依赖）
 */

#include "FOC_ADC.h"
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* 三点中值：排序网络；内部用 inline 版本，热路径避免函数调用            */
/* ------------------------------------------------------------------ */
static inline uint16_t median3_inline(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

ADC_FILTER_TEXT
uint16_t ADC_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    return median3_inline(a, b, c);
}

/* ------------------------------------------------------------------ */
/* 内部辅助：把 float 中点安全地转换为 uint16 码值                      */
/*   - NaN / 负值 → 0                                                  */
/*   - 上界留 0.5f 余量，防止 m+0.5f 越界造成 UB                        */
/* ------------------------------------------------------------------ */
static uint16_t midpoint_to_code(float m)
{
    if ((m != m) || (m <= 0.0f)) return 0u;   /* NaN 与负值（含 -0.0f） */
    if (m >= 65534.5f)           return 65535u;  /* 保证 +0.5f 不越界 */
    return (uint16_t)(m + 0.5f);
}

/* ------------------------------------------------------------------ */
/* 内部辅助：alpha 限幅，NaN 视为 0                                     */
/*   显式用 != 判断 NaN，避免依赖编译器对 !(x >= 0) 的 NaN 处理         */
/* ------------------------------------------------------------------ */
static float sanitize_alpha(float alpha)
{
    if ((alpha != alpha) || (alpha < 0.0f)) alpha = 0.0f;  /* NaN 或负数 */
    if (alpha > 1.0f)                       alpha = 1.0f;
    return alpha;
}

/* ------------------------------------------------------------------ */
/* 内部辅助：midpoint sanitize                                          */
/*   - NaN   → ADC_FILTER_DEFAULT_MIDPOINT（默认 12 位 2048）            */
/*   - 负值  → 0（差分 ADC 的合法零点）                                  */
/*   - >65535 → 65535                                                   */
/* ------------------------------------------------------------------ */
static float sanitize_midpoint(float m)
{
    if (m != m)        return ADC_FILTER_DEFAULT_MIDPOINT;  /* NaN */
    if (m < 0.0f)      return 0.0f;
    if (m > 65535.0f)  return 65535.0f;
    return m;
}

/* ------------------------------------------------------------------ */
/* 内部辅助：current_scale sanitize                                     */
/*   - NaN / ±inf / 极端值 → 0（不换算，输出恒 0，避免 NaN/inf 流出）   */
/*   - 负值保留（传感器反接场景，scale = I/(code-mid) 可为负）          */
/* ------------------------------------------------------------------ */
static float sanitize_scale(float s)
{
    if (s != s)                  return 0.0f;   /* NaN */
    if (s > 1e30f || s < -1e30f) return 0.0f;   /* ±inf / 极端值 */
    return s;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
ADC_FILTER_TEXT
void ADC_Filter_Init(ADC_Filter_t *f, float alpha, float midpoint, float current_scale)
{
    if (f == NULL) return;

    f->alpha           = sanitize_alpha(alpha);
    // f->one_minus_alpha = 1.0f - f->alpha;
    f->midpoint        = sanitize_midpoint(midpoint);
    f->current_scale   = sanitize_scale(current_scale);

    /* 窗口以中点（四舍五入到整数码）初始化；IIR 用 sanitize 后的 midpoint，
     * 保证「首个周期输出电流 = (iir_prev - midpoint)*scale = 0」，
     * 且内部状态与窗口中心一致，避免首拍跳变。 */
    const uint16_t mid = midpoint_to_code(f->midpoint);
    f->window[0] = mid;
    f->window[1] = mid;
    f->window[2] = mid;
    f->iir_prev  = f->midpoint;
    f->current   = 0.0f;
    f->inited    = true;
}

/* ------------------------------------------------------------------ */
/* 运行时配置                                                          */
/* ------------------------------------------------------------------ */
ADC_FILTER_TEXT
void ADC_Filter_SetAlpha(ADC_Filter_t *f, float alpha)
{
    if (f == NULL) return;
    f->alpha           = sanitize_alpha(alpha);
    f->one_minus_alpha = 1.0f - f->alpha;
}

ADC_FILTER_TEXT
void ADC_Filter_SetMidpoint(ADC_Filter_t *f, float midpoint)
{
    if (f == NULL) return;
    f->midpoint = sanitize_midpoint(midpoint);

    /* 同步重置内部状态，避免旧零点残留导致电流读数跳变。
     * 若未 inited 则跳过（Init 时会正确初始化）。 */
    if (f->inited) {
        const uint16_t mid = midpoint_to_code(f->midpoint);
        f->window[0] = mid;
        f->window[1] = mid;
        f->window[2] = mid;
        f->iir_prev  = f->midpoint;
        f->current   = 0.0f;
    }
}

ADC_FILTER_TEXT
void ADC_Filter_SetScale(ADC_Filter_t *f, float current_scale)
{
    if (f != NULL) f->current_scale = sanitize_scale(current_scale);
}

/* ------------------------------------------------------------------ */
/* 复位（保留配置参数）                                                */
/* ------------------------------------------------------------------ */
ADC_FILTER_TEXT
void ADC_Filter_Reset(ADC_Filter_t *f)
{
    if (f == NULL || !f->inited) return;

    const uint16_t mid = midpoint_to_code(f->midpoint);
    f->window[0] = mid;
    f->window[1] = mid;
    f->window[2] = mid;
    f->iir_prev  = f->midpoint;
    f->current   = 0.0f;
}

/* ------------------------------------------------------------------ */
/* 主更新函数                                                          */
/* ------------------------------------------------------------------ */
ADC_FILTER_TEXT
float ADC_Filter_Update(ADC_Filter_t *f, uint16_t raw)
{
    if (f == NULL || !f->inited) return 0.0f;

    /* 防御：若内部状态被异常写成 NaN，先用中点恢复，避免 NaN 永久传播。
     * 正常路径只多一次比较，代价远小于让 FOC 的 PI 收到 NaN。 */
    if (f->iir_prev != f->iir_prev) f->iir_prev = f->midpoint;

    /* 1. 更新三点中值窗口（移位，丢弃最旧） */
    f->window[0] = f->window[1];
    f->window[1] = f->window[2];
    f->window[2] = raw;

    /* 2. 中值滤波（inline，避免 48 kHz 热路径的函数调用） */
    const uint16_t median = median3_inline(f->window[0], f->window[1], f->window[2]);

    /* 3. 一阶 IIR 低通（在 ADC 码值域），使用预存的 1-alpha 省一次减法 */
    // f->iir_prev = f->alpha * (float)median
    //             + f->one_minus_alpha * f->iir_prev;

    /* 这一句会被 Cortex-M4F/M7F 编译器直接翻译为 VFMA（Fused Multiply-Add），极快 */
    f->iir_prev = f->iir_prev + f->alpha * ((float)median - f->iir_prev);

    /* 4. 码值 → 电流 (A) */
    f->current = (f->iir_prev - f->midpoint) * f->current_scale;

    return f->current;
}

/* ------------------------------------------------------------------ */
/* 只读访问                                                            */
/* ------------------------------------------------------------------ */
ADC_FILTER_TEXT
float ADC_Filter_GetCurrent(const ADC_Filter_t *f)
{
    return (f != NULL) ? f->current : 0.0f;
}

ADC_FILTER_TEXT
float ADC_Filter_GetFilteredCode(const ADC_Filter_t *f)
{
    return (f != NULL) ? f->iir_prev : 0.0f;
}

/* ------------------------------------------------------------------ */
/* 自检（刻意不加 ADC_FILTER_TEXT，函数本体永远落在 .text）             */
/* ------------------------------------------------------------------ */
#if ADC_FILTER_ENABLE_SELFTEST
bool ADC_SelfTest(void)
{
    ADC_Filter_t f;
    bool ok = true;

    /* 构造标准 NaN / +inf，避免 0.0f/0.0f 触发编译器告警 */
    union { uint32_t u; float f; } v;
    v.u = 0x7FC00000u;
    const float NAN_F = v.f;
    v.u = 0x7F800000u;
    const float INF_F = v.f;

    /* 1) 静止在中点 → 首拍输出严格为 0 */
    ADC_Filter_Init(&f, 0.7f, 2048.0f, 0.0179f);
    if (ADC_Filter_Update(&f, 2048u) != 0.0f) ok = false;

    /* 2) 单拍尖峰被中值滤除（iir_prev 应几乎不变） */
    {
        const float before = ADC_Filter_GetFilteredCode(&f);
        ADC_Filter_Update(&f, 65535u);   /* 尖峰进入窗口，但被 median 排除 */
        ADC_Filter_Update(&f, 2048u);    /* 下一拍尖峰被移出窗口 */
        const float after = ADC_Filter_GetFilteredCode(&f);
        const float tol   = 2048.0f * 0.001f;
        const float diff  = (after > before) ? (after - before) : (before - after);
        if (diff > tol) ok = false;
    }

    /* 3) NaN midpoint → 默认中点，且内部状态同步重置 */
    ADC_Filter_Init(&f, 0.7f, 2048.0f, 0.0179f);
    ADC_Filter_SetMidpoint(&f, NAN_F);
    if (f.midpoint != ADC_FILTER_DEFAULT_MIDPOINT) ok = false;
    if (f.iir_prev != ADC_FILTER_DEFAULT_MIDPOINT) ok = false;
    if (f.current != 0.0f) ok = false;
    {
        const float cur = ADC_Filter_Update(&f, 2048u);
        if (cur != cur) ok = false;              /* current 是 NaN → 失败 */
    }

    /* 4) NaN alpha 安全归 0，one_minus_alpha = 1 */
    ADC_Filter_Init(&f, NAN_F, 2048.0f, 0.0179f);
    if (f.alpha != 0.0f)           ok = false;
    // if (f.one_minus_alpha != 1.0f) ok = false;

    /* 5) NaN / +inf scale 安全归 0，输出恒 0 */
    ADC_Filter_Init(&f, 0.7f, 2048.0f, NAN_F);
    if (f.current_scale != 0.0f)               ok = false;
    if (ADC_Filter_Update(&f, 4095u) != 0.0f)  ok = false;
    ADC_Filter_Init(&f, 0.7f, 2048.0f, 0.0179f);
    ADC_Filter_SetScale(&f, INF_F);
    if (f.current_scale != 0.0f) ok = false;

    /* 6) 中值排序网络 */
    if (ADC_Median3(3u, 1u, 2u) != 2u)       ok = false;
    if (ADC_Median3(1u, 2u, 3u) != 2u)       ok = false;
    if (ADC_Median3(3u, 2u, 1u) != 2u)       ok = false;
    if (ADC_Median3(0u, 65535u, 1u) != 1u)   ok = false;

    /* 7) Reset：驱动后复位，输出与滤波码值都回到中点 */
    ADC_Filter_Init(&f, 0.7f, 2048.0f, 0.0179f);
    ADC_Filter_Update(&f, 4095u);
    ADC_Filter_Update(&f, 4095u);
    ADC_Filter_Reset(&f);
    if (ADC_Filter_GetCurrent(&f) != 0.0f)                 ok = false;
    if (ADC_Filter_GetFilteredCode(&f) != 2048.0f)         ok = false;

    /* 8) alpha=0：完全保持初始值，输出恒 0 */
    ADC_Filter_Init(&f, 0.0f, 2048.0f, 0.0179f);
    ADC_Filter_Update(&f, 3000u);
    ADC_Filter_Update(&f, 3000u);
    ADC_Filter_Update(&f, 3000u);
    if (ADC_Filter_GetFilteredCode(&f) != 2048.0f) ok = false;
    if (ADC_Filter_GetCurrent(&f) != 0.0f)         ok = false;

    /* 9) alpha=1：IIR 退化为中值输出；三点窗口两拍后跟上 3000 */
    ADC_Filter_Init(&f, 1.0f, 2048.0f, 0.0179f);
    ADC_Filter_Update(&f, 3000u);
    ADC_Filter_Update(&f, 3000u);
    if (ADC_Filter_GetFilteredCode(&f) != 3000.0f) ok = false;

    /* 10) 负 scale：方向翻转，输出为负 */
    ADC_Filter_Init(&f, 1.0f, 2048.0f, -0.01f);
    ADC_Filter_Update(&f, 4095u);
    ADC_Filter_Update(&f, 4095u);
    if (ADC_Filter_GetCurrent(&f) >= 0.0f) ok = false;

    /* 11) 内部 NaN 状态可恢复 */
    ADC_Filter_Init(&f, 0.7f, 2048.0f, 0.0179f);
    f.iir_prev = NAN_F;
    if (ADC_Filter_Update(&f, 2048u) != 0.0f) ok = false;

    /* 12) NULL / 未初始化保护 */
    if (ADC_Filter_Update(NULL, 0u) != 0.0f) ok = false;
    if (ADC_Filter_GetCurrent(NULL) != 0.0f) ok = false;
    {
        ADC_Filter_t g = {0};
        if (ADC_Filter_Update(&g, 1000u) != 0.0f) ok = false;
    }

    return ok;
}
#endif /* ADC_FILTER_ENABLE_SELFTEST */
