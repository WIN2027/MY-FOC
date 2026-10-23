#include "UART.h"
#include <stdio.h>
#include "FOC.h"
#include "Init.h"
#include <string.h>
#include "cmsis_os2.h"
#include "DRV8320S.h"
#include "FreeRTOS_Task.h"
#include "stm32g4xx_hal.h"

// ---------- DEBUG模式 ----------
bool FOC_Debug = false;

// ---------- 接收状态机变量 ----------
static uint8_t Rx_Index = 0;
static bool Frame_Ready = false;

// 遥测使能掩码，默认为全部使能（0xFFFF）
uint16_t g_telemetry_mask = 0xFFFF;
bool g_telemetry_enable = false;   // 主动上报使能标志

// ---------- DMA 接收缓冲区（单次模式） ----------
uint8_t DMA_Rx_Buf[DMA_RX_BUF_SIZE];
uint16_t Rx_Len = 0;

// ---------- 发送 DMA 控制 ----------
uint8_t DMA_Tx_Buf[DMA_TX_BUF_SIZE];
volatile bool Tx_Busy = false;

// ---------- CRC8校验 ----------
static uint8_t crc8(const uint8_t *data, const uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

// ---------- 串口队列发送 ----------
void UART_SendResponse(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (UART_Tx_Queue == NULL) return;

    // 限制数据长度
    if (len > UART_TX_MSG_MAX_SIZE - 4) {len = UART_TX_MSG_MAX_SIZE - 4;}

    UART_TxMsg_t msg;
    msg.data[0] = 0x5A;                     // SOF
    msg.data[1] = cmd;                      // CMD
    msg.data[2] = len;                      // LEN
    if (len && data) {
        memcpy(&msg.data[3], data, len);
    }
    // 计算CRC（从CMD开始）
    msg.data[3 + len] = crc8(&msg.data[1], 2 + len);
    msg.len = 4 + len;                      // 总长度

    // 入队（超时0，若队列满则丢弃）
    if (osMessageQueuePut(UART_Tx_Queue, &msg, 0U, 0U) == osOK)
    {
        // 设置发送队列标志，唤醒 UART_Task
        osThreadFlagsSet(UART_TaskHandle, BIT_TX_QUEUE);
    }
    // 若队列满，消息丢失（可在此增加错误计数）
}

// ---------- 用户回调 ----------
static CmdHandlerEntry_t CMD_Table[MAX_CMD_HANDLERS] = {0};  // 全部初始化为0

/**
 * @brief 处理从 DMA 接收缓冲区中提取的一帧数据
 * @param buf   指向完整帧数据的指针（应包含 SOF、CMD、LEN、DATA、CRC）
 * @param len   接收到的数据总长度（字节数）
 * @retval 无
 * @note  该函数执行以下步骤：
 *        1. 检查最小帧长度（至少 4 字节：SOF+CMD+LEN+CRC）
 *        2. 校验帧头（SOF = 0x5A）
 *        3. 根据 LEN 字段计算预期帧长度，并与实际长度比对
 *        4. 计算 CRC8（从 CMD 到 DATA 结束）并与接收到的 CRC 对比
 *        5. 查找命令处理表，调用对应的回调函数
 *        6. 若校验失败或命令无效，自动发送错误应答帧
 */
void SerialCom_ProcessFrame(const uint8_t *buf, const uint16_t len)
{
    uint16_t offset = 0; // 增加一个游标，记录当前解析到哪个字节

    // 只要游标还没走到缓冲区末尾，就继续解析
    while (offset < len)
    {
        // 1. 寻找帧头 0x5A（防止前面有乱码或半截垃圾数据）
        while (offset < len && buf[offset] != 0x5A) {
            offset++;
        }

        // 如果剩下的数据连最小帧长(4字节)都不够了，说明后面全是残包，直接结束
        if (len - offset < 4) {
            break;
        }

        // 2. 提取协议字段
        uint8_t cmd = buf[offset + 1];
        uint8_t data_len = buf[offset + 2];
        uint16_t expected_len = 4 + data_len; // SOF(1) + CMD(1) + LEN(1) + DATA(N) + CRC(1)

        // 如果剩下的数据不够这一帧的完整长度，说明发生了半包，退出等待下一次接收
        if (len - offset < expected_len) {
            break;
        }

        // 3. 计算并校验 CRC（从 CMD 开始到 DATA 结束）
        uint8_t calc_crc = crc8(&buf[offset + 1], 2 + data_len);
        uint8_t recv_crc = buf[offset + 3 + data_len];

        if (calc_crc == recv_crc)
        {
            // 4. 查找命令处理函数
            SerialDataHandler_t handler = NULL;
            for (int i = 0; i < MAX_CMD_HANDLERS; i++) {
                if (CMD_Table[i].Enabled && CMD_Table[i].CMD == cmd) {
                    handler = CMD_Table[i].Handler;
                    break;
                }
            }

            // 5. 执行回调
            if (handler != NULL) {
                // 注意这里传的是 buf[offset + 3]，因为前面加了偏移
                handler(cmd, &buf[offset + 3], data_len);
            } else {
                UART_SendResponse(CMD_ERR_INVALID_CMD, NULL, 0);
            }

            // 6. 成功解析一帧，游标向前移动一整帧的长度，准备解析下一帧
            offset += expected_len;
        }
        else
        {
            // CRC 校验失败
            UART_SendResponse(CMD_ERR_CRC, NULL, 0);
            // 游标只往前走 1 个字节，跳过这个假帧头，去后面继续寻找真正的 0x5A
            offset += 1;
        }
    }
}
/**
 * @brief UART DMA 发送完成回调（由 HAL 库自动调用）
 * @param huart 触发回调的 UART 句柄
 * @return 无
 * @note  清除 Tx_Busy 标志，允许下一次发送。
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {Tx_Busy = false;}
}
/**
 * @brief UART 空闲中断回调（需在 stm32g4xx_it.c 中调用或直接放于中断函数）
 * @param huart 触发回调的 UART 句柄
 * @return 无
 * @note  当 USART3 产生空闲中断时，此函数被调用。
 *        它计算本次 DMA 接收到的字节数（Rx_Len），然后唤醒 UART_Task 处理数据。
 *        此实现假设 DMA 工作在 Normal 模式（收到一帧后自动停止）。
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   // LED 闪烁指示进入回调
        if (Size !=0 && Size!=DMA_RX_BUF_SIZE) {Rx_Len = Size; Frame_Ready = true;osThreadFlagsSet(UART_TaskHandle, BIT_RX_READY);;}
    }
}
/**
 * @brief 注册一个命令码及其处理函数（精确映射）
 * @param cmd     命令码（如 CMD_SET_SPEED）
 * @param handler 处理该命令的回调函数指针
 * @retval 无
 * @note  如果同一个命令码被重复注册，后注册的会覆盖先注册的。
 *        最多支持 MAX_CMD_HANDLERS 个映射条目。
 */
void SerialCom_RegisterCmdHandler(const uint8_t cmd, SerialDataHandler_t handler)
{
    if (handler == NULL) return;

    // 先检查是否已经注册过同样的命令码（避免重复）
    for (int i = 0; i < MAX_CMD_HANDLERS; i++)
        {
        if (CMD_Table[i].Enabled && CMD_Table[i].CMD == cmd)
        {
            // 可选：更新回调函数（覆盖旧值）
            CMD_Table[i].Handler = handler;return;
        }
    }
    // 找一个空闲槽位
    for (int i = 0; i < MAX_CMD_HANDLERS; i++) {
        if (!CMD_Table[i].Enabled) {
            CMD_Table[i].CMD = cmd;
            CMD_Table[i].Handler = handler;
            CMD_Table[i].Enabled = 1;
            return;
        }
    }

    // 如果执行到这里，说明表已满，可触发错误处理（例如设置全局错误标志）
    // 对于嵌入式系统，也可选择覆盖最旧的条目（这里简单忽略并返回）
}
/**
 * @brief 注销一个命令码的处理函数
 * @param cmd 要注销的命令码
 * @retval 无
 * @note  注销后该命令将无法被识别，底层会回复 CMD_ERR_INVALID_CMD
 */
void SerialCom_UnregisterCmdHandler(const uint8_t cmd)
{
    for (int i = 0; i < MAX_CMD_HANDLERS; i++) {
        if (CMD_Table[i].Enabled && CMD_Table[i].CMD == cmd) {
            CMD_Table[i].Enabled = 0;
            // 可选：清除 handler 和 cmd
            CMD_Table[i].Handler = NULL;
            CMD_Table[i].CMD = 0;
            break;
        }
    }
}

// ---------- 命令回调 ----------
void Slave_ACK(const uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)data;(void)len;

    // HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);
    // HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

    // 使用通用应答函数发送空数据域（仅表示命令已成功接收）
    UART_SendResponse(cmd, NULL, 0);
}

static void Start_Align_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd; (void)data; (void)len;
    FOC_Start(FOC_STATE_ALIGN);

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

    // 1. 启动前清零计数器 (推荐使用 HAL 宏)
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, 0);

    // 2. 一次性同步启动主定时器 + A + B + D (HAL 宏支持掩码)
    __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);

    if (g_telemetry_enable){HAL_TIM_Base_Start_IT(&htim4);}

    UART_SendResponse(CMD_ACK_OK, NULL, 0);
}

static void START_MOTOR_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd; (void)data; (void)len;

    FOC_Start(FOC_STATE_OPENLOOP);

    speed_pid.target = 1500.0f;

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);

    // 1. 启动前清零计数器 (推荐使用 HAL 宏)
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0);
    __HAL_HRTIM_SETCOUNTER(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, 0);

    // 2. 一次性同步启动主定时器 + A + B + D (HAL 宏支持掩码)
    __HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);

    if (g_telemetry_enable){HAL_TIM_Base_Start_IT(&htim4);}

    UART_SendResponse(CMD_ACK_OK, NULL, 0);
}

static void STOP_MOTOR_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd; (void)data; (void)len;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
    // // 2. 一次性同步停止主定时器 + A + B + D (HAL 宏支持掩码)
    __HAL_HRTIM_DISABLE(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);

    FOC_Stop();

    HAL_TIM_Base_Stop_IT(&htim4);

}

/**
 * @brief 获取 DRV8320S 配置并返回给上位机
 */
static void Get_DRV8320S_Config(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd; (void)data; (void)len; // 避免未使用警告

    DRV8320S_Config_t config;
    if (HAL_DRV8320S_GetConfig(&hdrv8320s, &config) == HAL_OK) {
        // 直接发送结构体（二进制）
        UART_SendResponse(CMD_GET_DRV_CONFIG, (const uint8_t*)&config, sizeof(config));
    } else
    {
        // 读取失败，发送错误
        UART_SendResponse(CMD_ERR_CRC, NULL, 0);
    }
}

/**
 * @brief 清除 DRV8320S 故障寄存器（FAULT1/FAULT2）
 * @param cmd   命令码（CMD_CLEAR_FAULTS）
 * @param data  数据域（无数据，此处忽略）
 * @param len   数据长度（应为 0）
 */
static void Clear_Faults_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd; (void)data; (void)len;  // 避免未使用警告

    HAL_DRV8320S_UnlockRegisters(&hdrv8320s);

    if (HAL_DRV8320S_ClearFaults(&hdrv8320s) == HAL_OK) {UART_SendResponse(CMD_ACK_OK, NULL, 0);}
    else {UART_SendResponse(CMD_ERR_CRC, NULL, 0);  }// 清除失败返回通用错误

    HAL_DRV8320S_LockRegisters(&hdrv8320s);
}

/**
 * @brief 设置 DRV8320S 配置（从上位机接收配置结构体）
 * @param cmd   命令码（CMD_SET_DRV_CONFIG）
 * @param data  数据域指针，指向 DRV8320S_InitTypeDef 结构体
 * @param len   数据长度（必须等于 sizeof(DRV8320S_InitTypeDef)）
 */
static void Set_DRV8320S_Config(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    if (len != sizeof(DRV8320S_InitTypeDef))
    {
        UART_SendResponse(CMD_ERR_CRC, NULL, 0);  // 长度错误
        return;
    }
    // 1. 使能器件（拉高 ENABLE）
    HAL_DRV8320S_Enable(&hdrv8320s, 1);
    HAL_Delay(4);  // 等待器件稳定（> tWAKE 1ms）

    // 解锁寄存器（允许写入）
    if (HAL_DRV8320S_UnlockRegisters(&hdrv8320s) != HAL_OK)
    {
        UART_SendResponse(CMD_ERR_CRC, NULL, 0);
        return;
    }

    // 复制配置结构体（注意：数据从 data 指向的缓冲区直接转换）
    const DRV8320S_InitTypeDef *config = (const DRV8320S_InitTypeDef*)data;

    // 调用初始化函数（会执行所有写入操作）
    if (HAL_DRV8320S_Init(&hdrv8320s, config) != HAL_OK) {
        UART_SendResponse(CMD_ERR_CRC, NULL, 0);
        // 可选：配置失败后重新锁定寄存器
        HAL_DRV8320S_LockRegisters(&hdrv8320s);
        return;
    }
    // 可选：锁定寄存器（防止意外修改）
    HAL_DRV8320S_LockRegisters(&hdrv8320s);

    // 发送成功应答（无数据）
    UART_SendResponse(CMD_ACK_OK, NULL, 0);
}

static void Set_Current_PI_Callback(uint8_t cmd, const uint8_t *data, uint8_t len) {
    (void)cmd;
    if (len != sizeof(CurrentPIDConfig_t)) {UART_SendResponse(CMD_ERR_CRC, NULL, 0);return;}

    const CurrentPIDConfig_t *cfg = (const CurrentPIDConfig_t *)data;
    FOC_SetCurrentPID(cfg->kp_d, cfg->ki_d, cfg->kp_q, cfg->ki_q);

    CurrentPIDConfig_t cur_cfg;
    FOC_GetCurrentPID(&cur_cfg.kp_d, &cur_cfg.ki_d, &cur_cfg.kp_q, &cur_cfg.ki_q);
    UART_SendResponse(cmd, (const uint8_t*)&cur_cfg, sizeof(CurrentPIDConfig_t));
}

static void Set_Speed_PID_Callback(uint8_t cmd, const uint8_t *data, uint8_t len) {
    (void)cmd;
    if (len != sizeof(SpeedPIDConfig_t)) {UART_SendResponse(CMD_ERR_CRC, NULL, 0);return;}

    const SpeedPIDConfig_t *cfg = (const SpeedPIDConfig_t *)data;
    FOC_SetSpeedPID(cfg->kp, cfg->ki, cfg->kd);

    SpeedPIDConfig_t cur_cfg;
    FOC_GetSpeedPID(&cur_cfg.kp, &cur_cfg.ki, &cur_cfg.kd);
    UART_SendResponse(cmd, (const uint8_t*)&cur_cfg, sizeof(SpeedPIDConfig_t));
}

static void Set_Debug_Mode(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd; // 保留，本函数不依赖 cmd

    // 检查数据有效性：必须非空且长度至少为1
    if (data == NULL || len < 1) {UART_SendResponse(CMD_ERR_INVALID_CMD, NULL, 0); return;}
    uint8_t mode = data[0];
    // 根据数据域设置调试模式
    if (mode == 0) {FOC_Debug = false;HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);}
    else if (mode == 1) {FOC_Debug = true;HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_D);}
    else {UART_SendResponse(CMD_ERR_INVALID_CMD, NULL, 0);return;}
    if (g_telemetry_enable){HAL_TIM_Base_Start_IT(&htim4);}
    // 操作成功，返回确认
    UART_SendResponse(CMD_ACK_OK, NULL, 0);
}

/**
 * @brief 设置目标转速命令回调
 * @param cmd   命令码（CMD_SET_SPEED）
 * @param data  数据域指针，2字节小端 int16_t RPM
 * @param len   数据长度（应为2）
 */
static void SetSpeed_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    if (len != 2)
    {
        UART_SendResponse(CMD_ERR_CRC, NULL, 0);
        return;
    }
    int16_t rpm = (int16_t)(data[0] | (data[1] << 8));
    FOC_SetSpeedTarget(rpm);   // 使用FOC模块提供的API
    UART_SendResponse(CMD_ACK_OK, NULL, 0);
}

/**
 * @brief 设置遥测使能掩码（支持2字节旧格式和3字节新格式）
 * @param cmd   命令码（CMD_SET_TELEMETRY_MASK）
 * @param data  数据域指针
 *               len=3: 掩码低字节在前，第3字节为使能 (0=禁用, 非0=启用)
 * @param len   数据长度（3）
 */
static void SetTelemetryMask_Callback(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
    if (len != 3) {UART_SendResponse(CMD_ERR_CRC, NULL, 0);return;}
    // 更新掩码（小端模式）
    g_telemetry_mask = (uint16_t)(data[0] | (data[1] << 8));

    g_telemetry_enable = (data[2] != 0);

    UART_SendResponse(CMD_ACK_OK, NULL, 0);
}

static void START_MOTOR_INIT(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    (void)cmd;
}

/**
 * @brief 初始化串口软件状态机
 * @note  此函数不配置硬件UART，仅重置接收索引、帧标志、DMA头指针。
 *        注意：内部启动了DMA循环接收，可能与空闲中断的单次模式冲突，建议结合实际DMA配置修改。
 */
void SerialCom_Init(void) {
    Rx_Index = 0;
    Frame_Ready = false;
    SerialCom_RegisterCmdHandler(CMD_GET_VERSION,Slave_ACK);
    SerialCom_RegisterCmdHandler(CMD_START_ALIGN, Start_Align_Callback);
    SerialCom_RegisterCmdHandler(CMD_GET_DRV_CONFIG, Get_DRV8320S_Config);
    SerialCom_RegisterCmdHandler(CMD_SET_DRV_CONFIG, Set_DRV8320S_Config);
    SerialCom_RegisterCmdHandler(CMD_SET_CURRENT_PI, Set_Current_PI_Callback);
    SerialCom_RegisterCmdHandler(CMD_SET_SPEED_PID, Set_Speed_PID_Callback);
    SerialCom_RegisterCmdHandler(CMD_CLEAR_FAULTS, Clear_Faults_Callback);
    SerialCom_RegisterCmdHandler(CMD_SET_TELEMETRY_MASK, SetTelemetryMask_Callback);
    SerialCom_RegisterCmdHandler(CMD_DEBUG_MODE, Set_Debug_Mode);
    SerialCom_RegisterCmdHandler(CMD_START_MOTOR, START_MOTOR_Callback);
    SerialCom_RegisterCmdHandler(CMD_STOP_MOTOR, STOP_MOTOR_Callback);
    SerialCom_RegisterCmdHandler(CMD_SET_SPEED, SetSpeed_Callback);
    SerialCom_RegisterCmdHandler(CMD_START_INIT,START_MOTOR_INIT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, DMA_Rx_Buf, DMA_RX_BUF_SIZE);
    // 必须在每次重启接收后关闭半传输中断：HT事件会把缓冲区前半段误当成完整帧
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}



