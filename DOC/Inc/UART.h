#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 *  对外协议常量与命令码
 *  帧格式：SOF(0x5A) + CMD + LEN + DATA[LEN] + CRC8
 *  CRC8：poly=0x07，init=0xFF，覆盖 CMD+LEN+DATA
 * ========================================================= */

/* ------------------- 电流环/速度环配置结构（按 1 字节打包） ------------------- */
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

/* 启动/强拖参数（20 字节，主机<->设备布局） */
typedef struct {
    float    align_current_a;
    uint32_t align_time_ms;
    float    openloop_current_a;
    uint32_t openloop_time_ms;
    float    openloop_freq_hz;
} StartupConfigWire_t;

/* 电机参数（17 字节，极对数 1~255） */
typedef struct {
    float   R;
    float   Ld;
    float   Lq;
    float   flux_link;
    uint8_t pole_pairs;
} MotorParamsWire_t;
#pragma pack()

/**
 * @brief DRV8320S 关键配置数据结构（用于上位机查询）
 * @note  字段顺序即对外协议布局，共 18 字节：
 *        14 个 uint8 配置字段 + fault1/fault2 两个 uint16。
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

/* ------------------- 命令码定义 ------------------- */
typedef enum {
    CMD_SET_SPEED          = 0x01, //!< 设置目标转速（int16_t RPM，小端）
    CMD_SET_CURRENT_PI     = 0x02, //!< 设置电流环 PI（4 个 float）
    CMD_SET_SPEED_PID      = 0x03, //!< 设置速度环 PID（3 个 float）
    CMD_START_MOTOR        = 0x04, //!< 完整启动：ALIGN -> OPENLOOP -> CLOSEDLOOP
    CMD_STOP_MOTOR         = 0x05, //!< 停止电机
    CMD_REQUEST_TELEMETRY  = 0x06, //!< 请求一帧遥测（应答 0x18）
    CMD_GET_VERSION        = 0x07, //!< 获取版本字符串
    CMD_SET_STARTUP_CONFIG = 0x08, //!< 设置启动/强拖参数（20 字节）
    CMD_START_ALIGN        = 0x09, //!< 仅对齐，完成后自动停机
    CMD_GET_STARTUP_CONFIG = 0x0A, //!< 读取启动/强拖参数（20 字节）
    CMD_SET_MOTOR_PARAMS   = 0x0B, //!< 设置电机参数（17 字节）
    CMD_GET_MOTOR_PARAMS   = 0x0C, //!< 读取电机参数（17 字节）
    CMD_GET_DRV_CONFIG     = 0x13, //!< 读取 DRV8320S 配置（应答 18 字节）
    CMD_SET_DRV_CONFIG     = 0x14, //!< 写入 DRV8320S 配置（14 字节打包结构）
    CMD_CLEAR_FAULTS       = 0x15, //!< 清除 DRV8320S 故障寄存器
    CMD_START_INIT         = 0x16, //!< 保留（空操作，ACK）
    CMD_SET_TELEMETRY_MASK = 0x17, //!< 设置遥测掩码（uint16 + enable，共 3 字节）
    CMD_TELEMETRY_REPORT   = 0x18, //!< 设备->上位机：遥测数据
    CMD_DEBUG_MODE         = 0x19, //!< 调试标志（1 字节；不影响功率级）
    CMD_ACK_OK             = 0x80, //!< 通用成功应答
    CMD_ERR_CRC            = 0x81, //!< CRC 错误（协议层不主动发送）
    CMD_ERR_INVALID_CMD    = 0x82, //!< 无效命令/未注册
    CMD_ERR_NOT_CONFIGURED = 0x83, //!< FOC 未配置
    CMD_ERR_PARAM          = 0x84, //!< 参数/长度不合法
    CMD_ERR_BUSY           = 0x85, //!< 电机运行中，命令被拒绝
    CMD_ERR_HW             = 0x86, //!< 硬件自检失败
    CMD_ERR_STATE          = 0x87, //!< 当前状态不允许
    CMD_Fault1             = 0x88, //!< 设备->上位机：故障上报（4 字节故障码）
    CMD_ALIGN_DONE         = 0x89, //!< 设备->上位机：0x09 仅对齐完成
} SerialCmd_t;

/* ------------------- 对外接口 ------------------- */
void SerialCom_Init(void);

/** @brief 协议模块周期处理：RX 恢复、TX 超时/发送环、延迟分发、故障上报 */
void SerialCom_Process(void);

/**
 * @brief 发送一帧（组帧 + 入发送环）
 * @return true=已入发送环；false=参数错误/发送环满
 * @note 可在任务上下文调用；0x18 遥测和命令应答共用该接口。
 */
bool UART_SendResponse(uint8_t cmd, const uint8_t *data, uint8_t len);

/** @brief 按当前 g_telemetry_mask 打包并发出一帧 0x18 遥测 */
void UART_SendTelemetryReport(void);

extern volatile uint16_t g_telemetry_mask;
extern volatile bool g_telemetry_enable;   // 主动上报使能标志
extern bool FOC_Debug;

#endif //UART_H
