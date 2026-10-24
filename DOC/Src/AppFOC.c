#include "AppFOC.h"
#include "Init.h"
#include "main.h"

volatile bool App_FOC_AlignOnly = false;
volatile bool App_FOC_AlignDone = false;

void App_FOC_Init(void)
{
    const FOC_StartupConfig_t cfg = {
        .align_current_a    = APP_FOC_ALIGN_CURRENT_A,
        .align_time_ms      = APP_FOC_ALIGN_TIME_MS,
        .openloop_current_a = APP_FOC_OPENLOOP_CURRENT_A,
        .openloop_time_ms   = APP_FOC_OPENLOOP_TIME_MS,
        .openloop_freq_hz   = APP_FOC_OPENLOOP_FREQ_HZ,
    };

    FOC_Init();

    if (FOC_SetStartupConfig(&cfg) != FOC_OK)
    {
        Error_Handler();
    }

    FOC_SetSpeedTarget(APP_FOC_TARGET_RPM);
    App_FOC_AlignOnly = false;
    App_FOC_AlignDone = false;
}

FOC_Err_t App_FOC_Start(bool align_only)
{
    FOC_Err_t err;

    App_FOC_AlignOnly = align_only;
    App_FOC_AlignDone = false;

    /* 与旧版启动语义一致：
     *   true  = 仅 ALIGN，对齐完成后由 Back.c 立即停机并清运行状态；
     *   false = 跳过 ALIGN，直接 OPENLOOP 强拖。 */
    err = FOC_Start(align_only ? FOC_STATE_ALIGN : FOC_STATE_OPENLOOP);
    if (err != FOC_OK)
    {
        App_FOC_AlignOnly = false;
        return err;
    }

    /* 遥测任务由 TIM4 周期唤醒；是否真正发包由 g_telemetry_enable 决定 */
    (void)HAL_TIM_Base_Start_IT(&htim4);
    return FOC_OK;
}

void App_FOC_Stop(void)
{
    App_FOC_AlignOnly = false;
    FOC_Stop();
    (void)HAL_TIM_Base_Stop_IT(&htim4);
}

bool App_FOC_IsRunning(void)
{
    const FOC_State_t state = FOC_GetState();
    return (state != FOC_STATE_IDLE) && (state != FOC_STATE_FAULT);
}
