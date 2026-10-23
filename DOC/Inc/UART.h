#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "FOC.h"

/* ------------------- 电流环、速度环结构体 ------------------- */
#pragma pack(1)
typedef struct {
    float kp_d;
    float ki_d;
    float kp_q;
    float ki_q;
} CurrentPIDConfig_t;

typedef struct {
    float kp;
    float ki;
    float kd;
} SpeedPIDConfig_t;

typedef struct {
    float current;       // 对齐电流 (A)
    uint32_t time_ms;    // 对齐时间 (ms)
} AlignParams_t;
#pragma pack()
/**
 * @brief DRV8320S 关键配置数据结构（用于上位机查询）
 */
typedef struct {
    uint8_t  pwm_mode;          // PWM模式 (0~3)
    uint8_t  idrive_hs_source;  // 高侧源电流档位 0~15
    uint8_t  idrive_hs_sink;    // 高侧灌电流档位 0~15
    uint8_t  idrive_ls_source;  // 低侧源电流档位 0~15
    uint8_t  idrive_ls_sink;    // 低侧灌电流档位 0~15
    uint8_t  tdrive;            // TDRIVE时间 (0~3)
    uint8_t  dead_time;         // 死区时间 (0~3)
    uint8_t  ocp_mode;          // OCP模式 (0~3)
    uint8_t  ocp_deglitch;      // OCP消隐时间 (0~3)
    uint8_t  vds_level;         // VDS阈值 (0~15)
    uint8_t  retry_mode;        // 自动重试模式 (0~1)
    uint8_t  cpuv_enabled;      // 电荷泵欠压保护使能 (0/1)
    uint8_t  gdf_enabled;       // 栅极故障检测使能 (0/1)
    uint8_t  otw_report;        // 过热警告报告使能 (0/1)
    uint16_t fault1;            // FAULT1 寄存器原始值
    uint16_t fault2;            // FAULT2 寄存器原始值
} DRV8320S_Config_t;

/* ------------------- 配置参数 ------------------- */

#define RX_BUF_SIZE   64   // 接收缓冲区大小
#define TX_BUF_SIZE   64   // 发送缓冲区大小
#define SERIAL_FRAME_TIMEOUT_MS 10 // 帧间隔超时(ms)
#define DMA_RX_BUF_SIZE   64   // 缓冲区大小
#define DMA_TX_BUF_SIZE   64   // 缓冲区大小
#define BIT_RX_READY  0x01
#define BIT_TX_QUEUE  0x02
#define UART_TX_MSG_MAX_SIZE  64   // 最大帧长度（含SOF+CMD+LEN+DATA+CRC）

typedef struct
{
    uint8_t data[UART_TX_MSG_MAX_SIZE];
    uint16_t len;
} UART_TxMsg_t;

/* ------------------- 命令码定义 ------------------- */
typedef enum {
    CMD_SET_SPEED          = 0x01, //!< 设置目标转速（数据域：int16_t RPM）
    CMD_SET_CURRENT_PI     = 0x02, //!< 设置电流环 PI 系数（数据域：4个float）
    CMD_SET_SPEED_PID      = 0x03, //!< 设置速度环 PID 系数（数据域：3个float）
    CMD_START_MOTOR        = 0x04, //!< 启动电机（无数据域）
    CMD_STOP_MOTOR         = 0x05, //!< 停止电机（无数据域）
    CMD_REQUEST_TELEMETRY  = 0x06, //!< 请求遥测数据（无数据域，应答为 FOC_Telemetry_t）
    CMD_GET_VERSION        = 0x07, //!< 获取版本信息（无数据域，应答字符串）  eg:5A 07 00 BC
    CMD_SET_OPENLOOP_PARAM = 0x08, //!< 设置开环强拖参数（数据域：3个float）
    CMD_START_ALIGN        = 0x09, //!< 执行对齐操作（数据域：无）
    CMD_GET_DRV_CONFIG     = 0x13, //!< 获取DRV8320S配置（无数据域，应答为 DRV8320S_Config_t）  eg :5A 13 00 CRC
    CMD_SET_DRV_CONFIG     = 0x14, //!< 设置 DRV8320S 配置（数据域为 DRV8320S_InitTypeDef 结构体）
    CMD_SET_TELEMETRY_MASK = 0x17, //!< 设置遥测使能掩码（数据域：2字节）
    CMD_TELEMETRY_REPORT   = 0x18, //!< 主动上报遥测数据（由MCU发送）
    CMD_DEBUG_MODE         = 0x19, //!< 进入调试模式，数据域一个字节，0表示关闭，1为使能
    CMD_CLEAR_FAULTS       = 0x15, //!< 清除 DRV8320S 故障寄存器
    CMD_START_INIT         = 0x16, //!< 进行启动前的参数初始化；
    CMD_ACK_OK             = 0x80, //!< 通用成功应答（无数据域）
    CMD_Fault1             = 0x88, //!< 过流故障通知
    CMD_ERR_CRC            = 0x81, //!< CRC 校验错误应答
    CMD_ERR_INVALID_CMD    = 0x82, //!< 无效命令应答
} SerialCmd_t;

/* ------------------- 回调函数类型 ------------------- */
/**
 * @brief 用户命令处理回调函数类型
 * @param cmd   命令码
 * @param data  数据域指针
 * @param len   数据域长度
 */
typedef void (*SerialDataHandler_t)(uint8_t cmd, const uint8_t *data, uint8_t len);

#define MAX_CMD_HANDLERS  20   // 最多支持20条命令映射

typedef struct {
    uint8_t CMD;                     // 命令码
    SerialDataHandler_t Handler;     // 对应的处理函数
    uint8_t Enabled;                 // 该条目是否有效（1-有效，0-空闲）
} CmdHandlerEntry_t;

void SerialCom_RegisterCmdHandler(uint8_t cmd, SerialDataHandler_t handler);
void SerialCom_UnregisterCmdHandler(uint8_t cmd);

/* ------------------- 对外接口函数 ------------------- */
/**
 * @brief 初始化串口通信模块（软件状态机）
 */
void SerialCom_Init(void);

void SerialCom_ProcessFrame(const uint8_t *buf, const uint16_t len);

void SerialCom_SendTelemetry(const FOC_Telemetry_t *tele);

bool SerialCom_IsFrameReady(void);

/**
 * @brief 通过队列发送串口响应帧（替代原 SerialCom_SendResponse）
 * @param cmd  命令码
 * @param data 数据域指针（可为NULL）
 * @param len  数据域长度
 * @note  该函数将帧数据放入发送队列，并由 UART_Task 统一发送。
 *        若队列满，则丢弃该帧（非阻塞）。
 */
void UART_SendResponse(uint8_t cmd, const uint8_t *data, uint8_t len);

extern uint8_t DMA_Rx_Buf[DMA_RX_BUF_SIZE];
extern uint16_t Rx_Len;
extern uint16_t g_telemetry_mask;
extern volatile bool Tx_Busy;
extern bool g_telemetry_enable;   // 主动上报使能标志
extern bool FOC_Debug;

#endif //UART_H
