#ifndef FOC_PORT_STM32G4_H
#define FOC_PORT_STM32G4_H

#include <stdbool.h>
#include "main.h"
#include "stm32g4xx_hal.h"

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

/* v1.3：临界区改为 PRIMASK 保存/恢复，不再依赖 FreeRTOS。
 * 任务上下文与中断上下文都可安全调用（中断里恢复为关中断状态）。
 * 注意：宏展开为一条声明，必须在同一作用域内成对使用。 */
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

/* CORDIC 相位模式：直接写寄存器，实现在 foc_port_stm32g4.c 中。
 * 前提：hcordic 已由 CubeMX 的 MX_CORDIC_Init 配置为
 *      FUNCTION_PHASE / SCALE_0 / INSIZE_32BITS / OUTSIZE_32BITS /
 *      NBWRITE_2 / NBREAD_2 / PRECISION_6CYCLES */
void FOC_Port_CordicPhase(float x, float y, float scale, float *mag, float *angle);
#define FOC_PORT_CORDIC_PHASE(x, y, s, m, a)  FOC_Port_CordicPhase((x), (y), (s), (m), (a))

/* CORDIC 模式自检（只读 CSR，不触发计算，安全） */
#define FOC_PORT_CORDIC_SELFTEST() \
    ((((CORDIC->CSR) & CORDIC_CSR_FUNC) >> CORDIC_CSR_FUNC_Pos) == CORDIC_FUNCTION_PHASE)

/* 周期测时（DWT 为 Cortex-M 专属外设，M0/M0+ 需在此改成 TIM/SYSTICK） */
#define FOC_PORT_TS_BEGIN()   ((uint32_t)DWT->CYCCNT)
#define FOC_PORT_TS_END(t0)   ((uint32_t)(DWT->CYCCNT - (uint32_t)(t0)))

extern HRTIM_HandleTypeDef hhrtim1;

#endif /* FOC_PORT_STM32G4_H */
