#include <stdbool.h>
#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS_Task.h"
#include "UART.h"
#include "ADC.h"
#include "AppFOC.h"
#include "FOC_ADC.h"
#include "FOC_CORE.h"
#include "FOC_PORT_STM32G4.h"
#include "Init.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM20)
    {
        HAL_IncTick();
    }
    if (htim->Instance == TIM4)
    {
        if (g_telemetry_enable)
        {
            (void)osThreadFlagsSet(Telemetry_TaskHandle, 0x01U);
        }
    }
}

void HAL_HRTIM_Fault1Callback(HRTIM_HandleTypeDef *hhrtim)
{
    (void)hhrtim;
    __HAL_HRTIM_CLEAR_FLAG(&hhrtim1, HRTIM_FLAG_FLT1);

    const FOC_State_t state = FOC_GetState();
    if (state != FOC_STATE_IDLE && state != FOC_STATE_FAULT)
    {
        App_FOC_AlignOnly = false;
        FOC_OnFault(APP_FAULT_HRTIM);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    (void)hhrtim;
    (void)TimerIdx;
    FOC_SpeedLoopTick();
}

FOC_CCM_TEXT void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1){ADC1_Ready = true;}
    if (hadc->Instance == ADC2){ADC2_Ready = true;}

    if (ADC1_Ready && ADC2_Ready)
    {
        /* v1.2: 使用 port 层周期测时宏，TEST_MODE 下返回 0，可编译、跑通全链路。
        * 注意：foc_dwt 含高优先级中断抢占时间，非纯算法耗时。 */
        const uint32_t ts_start = FOC_PORT_TS_BEGIN();

        ADC1_Ready = false;
        ADC2_Ready = false;

        const float IU = ADC_Filter_Update(&s_adc2_filter, ADC1_DMA_BUF);
        const float IV = ADC_Filter_Update(&s_adc2_filter, ADC2_DMA_BUF);

        FOC_Run(IU, IV);

        /* 测时结束 */
        const uint32_t ts_cycles = FOC_PORT_TS_END(ts_start);
        FOC_SetCycleCount(ts_cycles);

        /* 0x09 仅对齐：FOC 核心在 ALIGN 计时结束后会自动切到 OPENLOOP，
         * 这里在同一拍立即停机，避免执行强拖。 */
        if (App_FOC_AlignOnly && FOC_GetState() == FOC_STATE_OPENLOOP)
        {
            App_FOC_Stop();
            App_FOC_AlignDone = true;
        }

    }
}
