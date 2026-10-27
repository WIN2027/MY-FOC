#ifndef FOC_PORT_STM32G4_H
#define FOC_PORT_STM32G4_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "stm32g4xx_hal.h"

/* =========================================================
 *  段属性宏（如果外部没有定义，这里兜底定义）
 * ========================================================= */
#ifndef FOC_CCM_TEXT
#ifdef FOC_USE_CCM_RAM
#define FOC_CCM_TEXT __attribute__((section(".ccmram_text")))
#else
#define FOC_CCM_TEXT
#endif
#endif

/* =========================================================
 *  HRTIM 硬件宏
 * ========================================================= */
/* U/V/W 三相对应的 HRTIM 定时器：U->B, V->A, W->D（与原工程一致） */
#define FOC_PORT_PWM_WRITE(a, b, c) do { \
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = (uint16_t)(a); \
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = (uint16_t)(b); \
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR = (uint16_t)(c); \
} while (0)

/* 启动顺序与原工程一致：先启动输出，再把三路计数器清零，最后同步使能。
 * 计数器归零保证故障/停止后重启时首周期相位与 ADC 触发点一致。 */
#define FOC_PORT_PWM_ENABLE() do { \
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, \
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
        HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); \
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0); \
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0); \
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, 0); \
    __HAL_HRTIM_ENABLE(&hhrtim1, \
        HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | \
        HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D); \
} while (0)

#define FOC_PORT_PWM_DISABLE() do { \
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, \
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
        HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2); \
    __HAL_HRTIM_DISABLE(&hhrtim1, \
        HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | \
        HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D); \
} while (0)

/* =========================================================
 *  临界区宏
 * ========================================================= */
static inline uint32_t FOC_Port_EnterCritical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void FOC_Port_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#define FOC_PORT_ENTER_CRITICAL()   uint32_t _foc_primask_ = FOC_Port_EnterCritical()
#define FOC_PORT_EXIT_CRITICAL()    FOC_Port_ExitCritical(_foc_primask_)

/* =========================================================
 *  CORDIC 硬件宏与内联函数（强制内联 + 放入 CCM）
 * ========================================================= */
#define CORDIC_PI_F    3.14159265358979323846f
#define CORDIC_2PI_F   6.28318530717958647692f

/* STM32G4 HAL 库提供的基础配置值本身就是已移位掩码，直接按位或即可 */
#define CORDIC_CSR_CFG_PHASE   (                                   \
    CORDIC_FUNCTION_PHASE      |                                   \
    CORDIC_PRECISION_6CYCLES   |                                   \
    CORDIC_SCALE_0             |                                   \
    CORDIC_NBWRITE_2           |                                   \
    CORDIC_NBREAD_2            |                                   \
    CORDIC_INSIZE_32BITS       |                                   \
    CORDIC_OUTSIZE_32BITS)

#define CORDIC_CSR_CFG_COSINE  (                                   \
    CORDIC_FUNCTION_COSINE     |                                   \
    CORDIC_PRECISION_6CYCLES   |                                   \
    CORDIC_SCALE_0             |                                   \
    CORDIC_NBWRITE_2           |                                   \
    CORDIC_NBREAD_2            |                                   \
    CORDIC_INSIZE_32BITS       |                                   \
    CORDIC_OUTSIZE_32BITS)

/* CORDIC 输入 Q1.31 饱和转换 */
static inline __attribute__((always_inline)) FOC_CCM_TEXT
int32_t _cordic_sat_to_q31(float v, float scale)
{
    const float t = v * scale;
    if (t >=  2147483647.0f) return  INT32_MAX;
    if (t <= -2147483648.0f) return  INT32_MIN;
    return (int32_t)t;
}

/* RRDY 等待加超时 */
static inline __attribute__((always_inline)) FOC_CCM_TEXT
bool _cordic_wait_rrdy(uint32_t timeout_cnt)
{
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U)
    {
        if (--timeout_cnt == 0U) return false;
    }
    return true;
}

/* 相位模式（atan2） */
static inline __attribute__((always_inline)) FOC_CCM_TEXT
void FOC_Port_CordicAtan2(float x, float y, float scale, float *mag, float *angle)
{
    const int32_t ix = _cordic_sat_to_q31(x, scale);
    const int32_t iy = _cordic_sat_to_q31(y, scale);

    CORDIC->CSR = CORDIC_CSR_CFG_PHASE;
    CORDIC->WDATA = (uint32_t)ix;
    CORDIC->WDATA = (uint32_t)iy;

    if (!_cordic_wait_rrdy(200U)) { *mag = 0.0f; *angle = 0.0f; return; }
    const int32_t res1 = (int32_t)CORDIC->RDATA;

    if (!_cordic_wait_rrdy(200U)) { *mag = 0.0f; *angle = 0.0f; return; }
    const int32_t res2 = (int32_t)CORDIC->RDATA;

    *angle = ((float)res1 / 2147483648.0f) * CORDIC_PI_F;
    *mag   = (float)res2 / scale;
}

/* 旋转模式（sincos） */
static inline __attribute__((always_inline)) FOC_CCM_TEXT
void FOC_Port_CordicSinCos(float theta, float *sin_out, float *cos_out)
{
    /* 规约到 [-π, π) */
    if (theta >=  CORDIC_PI_F) theta -= CORDIC_2PI_F;
    if (theta <  -CORDIC_PI_F) theta += CORDIC_2PI_F;

    const float qf = theta * (2147483648.0f / CORDIC_PI_F);
    int32_t ang_q31;
    if (qf >=  2147483647.0f)      ang_q31 = INT32_MAX;
    else if (qf <= -2147483648.0f) ang_q31 = INT32_MIN;
    else                            ang_q31 = (int32_t)qf;

    CORDIC->CSR = CORDIC_CSR_CFG_COSINE;
    CORDIC->WDATA = (uint32_t)ang_q31;
    CORDIC->WDATA = 0x7FFFFFFFu; /* 模数 1.0，Q1.31 */

    if (!_cordic_wait_rrdy(200U)) { *cos_out = 1.0f; *sin_out = 0.0f; return; }
    const int32_t res1 = (int32_t)CORDIC->RDATA;

    if (!_cordic_wait_rrdy(200U)) { *cos_out = 1.0f; *sin_out = 0.0f; return; }
    const int32_t res2 = (int32_t)CORDIC->RDATA;

    *cos_out = (float)res1 / 2147483648.0f;
    *sin_out = (float)res2 / 2147483648.0f;
}

/* 统一宏接口 */
#define FOC_PORT_CORDIC_PHASE(x, y, s, m, a)  FOC_Port_CordicAtan2((x), (y), (s), (m), (a))
#define FOC_PORT_CORDIC_SINCOS(theta, s, c)   FOC_Port_CordicSinCos((theta), (s), (c))

/* CORDIC 模式自检 */
#define FOC_PORT_CORDIC_SELFTEST() \
    ((((CORDIC->CSR) & CORDIC_CSR_FUNC) >> CORDIC_CSR_FUNC_Pos) == CORDIC_FUNCTION_PHASE)

/* 周期测时 */
#define FOC_PORT_TS_BEGIN()   ((uint32_t)DWT->CYCCNT)
#define FOC_PORT_TS_END(t0)   ((uint32_t)(DWT->CYCCNT - (uint32_t)(t0)))

extern HRTIM_HandleTypeDef hhrtim1;

#endif /* FOC_PORT_STM32G4_H */