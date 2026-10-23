#include <stdbool.h>
#include "cmsis_os2.h"
#include "main.h"
#include "FreeRTOS_Task.h"
#include "FOC.h"
#include "UART.h"
#include "ADC.h"
#include "Init.h"

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM20){HAL_IncTick();}
    if (htim->Instance == TIM4)
    {
        if (g_telemetry_enable) {osThreadFlagsSet(Telemetry_TaskHandle, 0x01);}
    }
}
void HAL_HRTIM_Fault1Callback(HRTIM_HandleTypeDef *hhrtim)
{
    __HAL_HRTIM_CLEAR_FLAG(&hhrtim1, HRTIM_FLAG_FLT1);

    if (foc_state != FOC_STATE_IDLE)
    {
        foc_state = FOC_STATE_IDLE;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

        UART_SendResponse(CMD_Fault1, NULL, 0);

        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
        // // 2. 一次性同步停止主定时器 + A + B + D (HAL 宏支持掩码)
        __HAL_HRTIM_DISABLE(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);

    }
}
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx)
{
    speed_pid.sample_frequency++;
    if (foc_state == FOC_STATE_CLOSEDLOOP && speed_pid.sample_frequency >= 25)
    {
        speed_pid.sample_frequency = 0;SpeedLoop_Update();
        if (smo.omega_est > 5.0f || smo.omega_est < -5.0f) {float dummy;Cordic_Phase(smo.lp_freq, smo.omega_est, CORDIC_FREQ_SCALE, &dummy, &smo.theta_comp);}
    }

}
// void HAL_HRTIM_Fault3Callback(HRTIM_HandleTypeDef *hhrtim)
// {
//     HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
//     HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);
//     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//     UART_SendResponse(CMD_Fault1, NULL, 0);
// }
__attribute__((section(".ccmram_text")))void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1){ADC1_Ready = true;}
    if (hadc->Instance == ADC2){ADC2_Ready = true;}
    if (ADC1_Ready == true && ADC2_Ready == true)
    {
        const uint32_t start_cycle = DWT->CYCCNT;

        ADC1_Ready = false;
        ADC2_Ready = false;
        const float IU = ADC1_Filter(ADC1_DMA_BUF);
        const float IV = ADC2_Filter(ADC2_DMA_BUF);

        FOC_Run(IU,IV);

        const uint32_t end_cycle = DWT->CYCCNT;
        s_telemetry.foc_dwt = end_cycle - start_cycle;

    }
}




// #include "main.h"
// #include "cmsis_os.h"
// #include "Init.h"
// #include "FreeRTOS_Task.h"
//
//
// int main(void)
// {
//
//     Peripheral_Init();
//     osKernelInitialize();
//     FreeRTOS_TaskInit();
//     osKernelStart();
//
//     // ReSharper disable once CppDFAEndlessLoop
//     while (1){}
//
// }