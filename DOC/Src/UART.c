#include "UART.h"

#include <string.h>

#include "main.h"
#include "Init.h"
#include "DRV8320S.h"
#include "FOC_CORE.h"
#include "ADC.h"
#include "AppFOC.h"
#include "Protocol.h"
#include "Protocol_HAL.h"

/* 对外协议布局的编译期校验 */
_Static_assert(sizeof(CurrentPIDConfig_t) == 16U, "CurrentPIDConfig_t must be 16 bytes");
_Static_assert(sizeof(SpeedPIDConfig_t) == 12U, "SpeedPIDConfig_t must be 12 bytes");
_Static_assert(sizeof(DRV8320S_Config_t) == 18U, "DRV8320S_Config_t must be 18 bytes");
_Static_assert(sizeof(DRV8320S_InitTypeDef) == 14U, "DRV8320S_InitTypeDef must be 14 bytes");
_Static_assert(sizeof(StartupConfigWire_t) == 20U, "StartupConfigWire_t must be 20 bytes");
_Static_assert(sizeof(MotorParamsWire_t) == 17U, "MotorParamsWire_t must be 17 bytes");

/* =========================================================
 *  对外状态
 * ========================================================= */
bool FOC_Debug = false;
volatile uint16_t g_telemetry_mask = 0xFFFF;
volatile bool g_telemetry_enable = false;

/* 协议实例必须是普通 SRAM（含 DMA 收发缓冲，不能放 CCM） */
static Protocol_Context_t     g_proto;
static Protocol_HAL_Context_t g_hal;

static uint32_t s_reported_fault = FOC_FAULT_NONE;

/* =========================================================
 *  小工具
 * ========================================================= */
static uint16_t clamp_u16(float v)
{
    if (!(v > 0.0f))      return 0U;
    if (v >= 65535.0f)    return 65535U;
    return (uint16_t)(v + 0.5f);
}

static int16_t clamp_i16(float v)
{
    if (!(v == v))      return 0;       /* NaN 防护，避免浮点转整型 UB */
    if (v > 32767.0f)   return 32767;
    if (v < -32768.0f)  return -32768;
    return (int16_t)(v >= 0.0f ? (v + 0.5f) : (v - 0.5f));
}

static uint16_t append_u16(uint8_t *buf, uint16_t n, uint16_t cap, uint16_t v)
{
    if ((uint32_t)n + 2U > cap) return n;
    buf[n++] = (uint8_t)(v & 0xFFU);
    buf[n++] = (uint8_t)((v >> 8) & 0xFFU);
    return n;
}

static uint16_t append_i16(uint8_t *buf, uint16_t n, uint16_t cap, int16_t v)
{
    return append_u16(buf, n, cap, (uint16_t)v);
}

/* =========================================================
 *  遥测打包
 * ========================================================= */
static uint16_t UART_PackTelemetry(uint8_t *buf, uint16_t cap)
{
    FOC_Telemetry_t tele;
    uint16_t n = 0U;
    const uint16_t mask = g_telemetry_mask;

    FOC_GetTelemetry(&tele);

    if (mask & 0x0001U) n = append_u16(buf, n, cap, clamp_u16(ADC1_Prev * 16.0f));
    if (mask & 0x0002U) n = append_u16(buf, n, cap, clamp_u16(ADC2_Prev * 16.0f));
    if (mask & 0x0004U) n = append_i16(buf, n, cap, tele.IU);
    if (mask & 0x0008U) n = append_i16(buf, n, cap, tele.IV);
    if (mask & 0x0010U) n = append_i16(buf, n, cap, tele.IW);
    if (mask & 0x0020U) n = append_i16(buf, n, cap, clamp_i16(pi_d.actual * 100.0f));
    if (mask & 0x0040U) n = append_i16(buf, n, cap, clamp_i16(pi_q.actual * 100.0f));
    if (mask & 0x0080U) n = append_i16(buf, n, cap, smo.speed_rpm);
    if (mask & 0x0100U) n = append_u16(buf, n, cap, tele.foc_dwt);

    if (mask & 0x0200U)
    {
        n = append_u16(buf, n, cap, PWM.PWM_U);
        n = append_u16(buf, n, cap, PWM.PWM_V);
        n = append_u16(buf, n, cap, PWM.PWM_W);
    }

    if (mask & 0x0400U)
    {
        n = append_i16(buf, n, cap, clamp_i16(pi_d.out * 100.0f));
        n = append_i16(buf, n, cap, clamp_i16(pi_d.error * 100.0f));
        n = append_i16(buf, n, cap, clamp_i16(pi_d.integral * 100.0f));
        n = append_i16(buf, n, cap, clamp_i16(pi_d.actual * 100.0f));
    }

    if (mask & 0x0800U)
    {
        n = append_i16(buf, n, cap, clamp_i16(pi_q.out * 100.0f));
        n = append_i16(buf, n, cap, clamp_i16(pi_q.error * 100.0f));
        n = append_i16(buf, n, cap, clamp_i16(pi_q.integral * 100.0f));
        n = append_i16(buf, n, cap, clamp_i16(pi_q.actual * 100.0f));
    }

    if (mask & 0x1000U)
    {
        n = append_i16(buf, n, cap, clamp_i16(smo.v_est * 100.0f));
    }

    return n;
}

void UART_SendTelemetryReport(void)
{
    uint8_t buf[PROTOCOL_MAX_PAYLOAD];
    const uint16_t n = UART_PackTelemetry(buf, (uint16_t)sizeof(buf));

    (void)UART_SendResponse(CMD_TELEMETRY_REPORT, buf, (uint8_t)n);
}

bool UART_SendResponse(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    return Protocol_Send(&g_proto, cmd, data, len);
}

/* =========================================================
 *  故障码 -> 错误命令码
 * ========================================================= */
static uint8_t foc_err_to_cmd(FOC_Err_t err)
{
    switch (err)
    {
        case FOC_ERR_NOT_CONFIGURED:      return CMD_ERR_NOT_CONFIGURED;
        case FOC_ERR_PARAM_OUT_OF_RANGE:  return CMD_ERR_PARAM;
        case FOC_ERR_BUSY:                return CMD_ERR_BUSY;
        case FOC_ERR_HW_SELFTEST:         return CMD_ERR_HW;
        case FOC_ERR_INVALID_STATE:
        default:                          return CMD_ERR_STATE;
    }
}

/* =========================================================
 *  故障异步上报（0x88 + 4 字节故障码，小端）
 * ========================================================= */
static void UART_ReportFault(void)
{
    const uint32_t fault = FOC_GetLastFault();

    if (fault == FOC_FAULT_NONE)
    {
        s_reported_fault = FOC_FAULT_NONE;
        return;
    }
    if (fault == s_reported_fault)
    {
        return;
    }

    const uint8_t payload[4] = {
        (uint8_t)(fault & 0xFFU),
        (uint8_t)((fault >> 8) & 0xFFU),
        (uint8_t)((fault >> 16) & 0xFFU),
        (uint8_t)((fault >> 24) & 0xFFU),
    };

    /* 发送环满时保持未上报状态，下个周期重试 */
    if (UART_SendResponse(CMD_Fault1, payload, (uint8_t)sizeof(payload)))
    {
        s_reported_fault = fault;
    }
}

/* =========================================================
 *  命令回调
 * ========================================================= */
static void Get_Version_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    static const char version[] = "FOC-G474 v1.8";
    (void)data;
    (void)len;
    (void)UART_SendResponse(cmd, (const uint8_t *)version, (uint8_t)(sizeof(version) - 1U));
}

static void Start_Align_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    if (!ADC_SelfTest_OK)
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
        return;
    }

    /* 功率级上电默认关闭；启动前显式使能 DRV8320S 并等待 tWAKE */
    if (HAL_DRV8320S_Enable(&hdrv8320s, 1) != HAL_OK)
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
        return;
    }
    HAL_Delay(1);

    const FOC_Err_t err = App_FOC_Start(true);
    (void)UART_SendResponse(err == FOC_OK ? CMD_ACK_OK : foc_err_to_cmd(err), NULL, 0U);
}

static void START_MOTOR_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    if (!ADC_SelfTest_OK)
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
        return;
    }

    /* 功率级上电默认关闭；启动前显式使能 DRV8320S 并等待 tWAKE */
    if (HAL_DRV8320S_Enable(&hdrv8320s, 1) != HAL_OK)
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
        return;
    }
    HAL_Delay(1);

    const FOC_Err_t err = App_FOC_Start(false);
    (void)UART_SendResponse(err == FOC_OK ? CMD_ACK_OK : foc_err_to_cmd(err), NULL, 0U);
}

static void STOP_MOTOR_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    App_FOC_Stop();
    (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
}

static void SetSpeed_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;

    if (len != 2U)
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    const int16_t rpm = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    FOC_SetSpeedTarget(rpm);
    (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
}

static void Set_Startup_Config_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;

    if (len != sizeof(StartupConfigWire_t))
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    StartupConfigWire_t wire;
    memcpy(&wire, data, sizeof(wire));

    const FOC_StartupConfig_t cfg = {
        .align_current_a    = wire.align_current_a,
        .align_time_ms      = wire.align_time_ms,
        .openloop_current_a = wire.openloop_current_a,
        .openloop_time_ms   = wire.openloop_time_ms,
        .openloop_freq_hz   = wire.openloop_freq_hz,
    };

    const FOC_Err_t err = FOC_SetStartupConfig(&cfg);
    (void)UART_SendResponse(err == FOC_OK ? CMD_ACK_OK : foc_err_to_cmd(err), NULL, 0U);
}

static void Get_Startup_Config_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

    FOC_StartupConfig_t cfg;
    const FOC_Err_t err = FOC_GetStartupConfig(&cfg);
    if (err != FOC_OK)
    {
        (void)UART_SendResponse(foc_err_to_cmd(err), NULL, 0U);
        return;
    }

    const StartupConfigWire_t wire = {
        .align_current_a    = cfg.align_current_a,
        .align_time_ms      = cfg.align_time_ms,
        .openloop_current_a = cfg.openloop_current_a,
        .openloop_time_ms   = cfg.openloop_time_ms,
        .openloop_freq_hz   = cfg.openloop_freq_hz,
    };

    (void)UART_SendResponse(cmd, (const uint8_t *)&wire, (uint8_t)sizeof(wire));
}

static void Set_Motor_Params_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;

    if (len != sizeof(MotorParamsWire_t))
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    MotorParamsWire_t wire;
    memcpy(&wire, data, sizeof(wire));

    const FOC_MotorConfig_t cfg = {
        .R          = wire.R,
        .Ld         = wire.Ld,
        .Lq         = wire.Lq,
        .flux_link  = wire.flux_link,
        .pole_pairs = wire.pole_pairs,
    };

    const FOC_Err_t err = FOC_SetMotorParams(&cfg);
    (void)UART_SendResponse(err == FOC_OK ? CMD_ACK_OK : foc_err_to_cmd(err), NULL, 0U);
}

static void Get_Motor_Params_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

    FOC_MotorConfig_t cfg;
    const FOC_Err_t err = FOC_GetMotorParams(&cfg);
    if (err != FOC_OK)
    {
        (void)UART_SendResponse(foc_err_to_cmd(err), NULL, 0U);
        return;
    }

    const MotorParamsWire_t wire = {
        .R          = cfg.R,
        .Ld         = cfg.Ld,
        .Lq         = cfg.Lq,
        .flux_link  = cfg.flux_link,
        .pole_pairs = cfg.pole_pairs,
    };

    (void)UART_SendResponse(cmd, (const uint8_t *)&wire, (uint8_t)sizeof(wire));
}

static void Set_Current_PI_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (len != sizeof(CurrentPIDConfig_t))
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    CurrentPIDConfig_t cfg;
    memcpy(&cfg, data, sizeof(cfg));
    FOC_SetCurrentPID(cfg.kp_d, cfg.ki_d, cfg.kp_q, cfg.ki_q);

    CurrentPIDConfig_t cur;
    FOC_GetCurrentPID(&cur.kp_d, &cur.ki_d, &cur.kp_q, &cur.ki_q);
    (void)UART_SendResponse(cmd, (const uint8_t *)&cur, (uint8_t)sizeof(cur));
}

static void Set_Speed_PID_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (len != sizeof(SpeedPIDConfig_t))
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    SpeedPIDConfig_t cfg;
    memcpy(&cfg, data, sizeof(cfg));
    FOC_SetSpeedPID(cfg.kp, cfg.ki, cfg.kd);

    SpeedPIDConfig_t cur;
    FOC_GetSpeedPID(&cur.kp, &cur.ki, &cur.kd);
    (void)UART_SendResponse(cmd, (const uint8_t *)&cur, (uint8_t)sizeof(cur));
}

static void SetTelemetryMask_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;

    if (len != 3U)
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    g_telemetry_mask   = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    g_telemetry_enable = (data[2] != 0U);
    (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
}

static void Set_Debug_Mode(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;

    if (len < 1U)
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }

    FOC_Debug = (data[0] != 0U);
    (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
}

static void Request_Telemetry_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    UART_SendTelemetryReport();
}

/**
 * @brief 获取 DRV8320S 配置并返回给上位机
 */
static void Get_DRV8320S_Config(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    DRV8320S_Config_t config;
    if (HAL_DRV8320S_GetConfig(&hdrv8320s, &config) == HAL_OK)
    {
        (void)UART_SendResponse(CMD_GET_DRV_CONFIG, (const uint8_t *)&config, (uint8_t)sizeof(config));
    }
    else
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
    }
}

/**
 * @brief 清除 DRV8320S 故障寄存器（FAULT1/FAULT2）
 */
static void Clear_Faults_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;

    HAL_DRV8320S_UnlockRegisters(&hdrv8320s);

    if (HAL_DRV8320S_ClearFaults(&hdrv8320s) == HAL_OK)
    {
        (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
    }
    else
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
    }

    HAL_DRV8320S_LockRegisters(&hdrv8320s);
}

/**
 * @brief 设置 DRV8320S 配置（仅 IDLE/FAULT 允许）
 */
static void Set_DRV8320S_Config(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;

    if (len != sizeof(DRV8320S_InitTypeDef))
    {
        (void)UART_SendResponse(CMD_ERR_PARAM, NULL, 0U);
        return;
    }
    if (App_FOC_IsRunning())
    {
        (void)UART_SendResponse(CMD_ERR_BUSY, NULL, 0U);
        return;
    }

    DRV8320S_InitTypeDef config;
    memcpy(&config, data, sizeof(config));

    HAL_DRV8320S_Enable(&hdrv8320s, 1);
    HAL_Delay(4);  /* > tWAKE 1 ms */

    if (HAL_DRV8320S_UnlockRegisters(&hdrv8320s) != HAL_OK)
    {
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
        return;
    }

    if (HAL_DRV8320S_Init(&hdrv8320s, &config) != HAL_OK)
    {
        HAL_DRV8320S_LockRegisters(&hdrv8320s);
        (void)UART_SendResponse(CMD_ERR_HW, NULL, 0U);
        return;
    }

    HAL_DRV8320S_LockRegisters(&hdrv8320s);
    (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
}

static void START_MOTOR_INIT(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    (void)data;
    (void)len;
    (void)UART_SendResponse(CMD_ACK_OK, NULL, 0U);
}

/* =========================================================
 *  初始化与周期处理
 * ========================================================= */
void SerialCom_Init(void)
{
    s_reported_fault = FOC_FAULT_NONE;

    if (!Protocol_HAL_Init(&g_hal, &g_proto, &huart3))
    {
        Error_Handler();
    }

    (void)Protocol_RegisterHandler(&g_proto, CMD_GET_VERSION,        Get_Version_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_START_ALIGN,        Start_Align_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_GET_DRV_CONFIG,     Get_DRV8320S_Config);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_DRV_CONFIG,     Set_DRV8320S_Config);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_CURRENT_PI,     Set_Current_PI_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_SPEED_PID,      Set_Speed_PID_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_CLEAR_FAULTS,       Clear_Faults_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_TELEMETRY_MASK, SetTelemetryMask_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_DEBUG_MODE,         Set_Debug_Mode);
    (void)Protocol_RegisterHandler(&g_proto, CMD_START_MOTOR,        START_MOTOR_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_STOP_MOTOR,         STOP_MOTOR_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_SPEED,          SetSpeed_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_STARTUP_CONFIG, Set_Startup_Config_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_GET_STARTUP_CONFIG, Get_Startup_Config_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_SET_MOTOR_PARAMS,   Set_Motor_Params_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_GET_MOTOR_PARAMS,   Get_Motor_Params_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_REQUEST_TELEMETRY,  Request_Telemetry_Callback);
    (void)Protocol_RegisterHandler(&g_proto, CMD_START_INIT,         START_MOTOR_INIT);
}

void SerialCom_Process(void)
{
    Protocol_HAL_Process();

#if PROTOCOL_DEFER_HANDLER
    Protocol_Process(&g_proto);
#endif

    UART_ReportFault();

    /* 0x09 仅对齐完成事件；发送环满时保留标志，下周期重试 */
    if (App_FOC_AlignDone)
    {
        if (UART_SendResponse(CMD_ALIGN_DONE, NULL, 0U))
        {
            App_FOC_AlignDone = false;
        }
    }
}

/* =========================================================
 *  HAL 回调转发
 * ========================================================= */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Protocol_HAL_TxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Protocol_HAL_RxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Protocol_HAL_ErrorCallback(huart);
}
