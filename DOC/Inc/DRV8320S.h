/**
 * @file DRV8320S.h
 * @brief DRV8320S (SPI版本) 三相栅极驱动器驱动库 - HAL风格
 * @note  适用于无电流采样放大器的DRV8320S，基于STM32 HAL库
 */

#ifndef DRV8320S_H
#define DRV8320S_H

#include "main.h"
#include <stdint.h>
#include "UART.h"

/*==================== 寄存器地址 ====================*/
#define DRV8320S_REG_FAULT1       0x00
#define DRV8320S_REG_FAULT2       0x01
#define DRV8320S_REG_DRIVER_CTRL  0x02
#define DRV8320S_REG_GATE_DRIVE_HS 0x03
#define DRV8320S_REG_GATE_DRIVE_LS 0x04
#define DRV8320S_REG_OCP_CTRL     0x05

/*==================== 枚举类型 ====================*/
typedef enum {
    PWM_MODE_6X     = 0x00,
    PWM_MODE_3X     = 0x01,
    PWM_MODE_1X     = 0x02,
    PWM_MODE_INDEP  = 0x03
} PWM_Mode_t;

typedef enum {
    OCP_MODE_LATCH        = 0x00,
    OCP_MODE_AUTO_RETRY   = 0x01,
    OCP_MODE_REPORT_ONLY  = 0x02,
    OCP_MODE_DISABLED     = 0x03
} OCP_Mode_t;

typedef enum {
    TDRIVE_500NS = 0x00,
    TDRIVE_1US   = 0x01,
    TDRIVE_2US   = 0x02,
    TDRIVE_4US   = 0x03
} TDRIVE_Time_t;

typedef enum {
    DEADTIME_50NS  = 0x00,
    DEADTIME_100NS = 0x01,
    DEADTIME_200NS = 0x02,
    DEADTIME_400NS = 0x03
} DeadTime_t;

typedef enum {
    OCP_DEG_2US = 0x00,
    OCP_DEG_4US = 0x01,
    OCP_DEG_6US = 0x02,
    OCP_DEG_8US = 0x03
} OCP_Deglitch_t;

typedef enum {
    VDS_LVL_0_06V = 0x00,
    VDS_LVL_0_13V = 0x01,
    VDS_LVL_0_20V = 0x02,
    VDS_LVL_0_26V = 0x03,
    VDS_LVL_0_31V = 0x04,
    VDS_LVL_0_45V = 0x05,
    VDS_LVL_0_53V = 0x06,
    VDS_LVL_0_60V = 0x07,
    VDS_LVL_0_68V = 0x08,
    VDS_LVL_0_75V = 0x09,
    VDS_LVL_0_94V = 0x0A,
    VDS_LVL_1_13V = 0x0B,
    VDS_LVL_1_30V = 0x0C,
    VDS_LVL_1_50V = 0x0D,
    VDS_LVL_1_70V = 0x0E,
    VDS_LVL_1_88V = 0x0F
} VDS_Level_t;

/*==================== 初始化配置结构体 ====================*/
/* 该结构体通过串口协议按内存布局整体传输，必须按 1 字节压缩打包：
 * 保证与上位机布局一致，并使指针强转访问时免受对齐问题影响 */
#pragma pack(push, 1)
typedef struct {
    PWM_Mode_t      pwmMode;        /*!< PWM 输入模式 */
    uint8_t         idriveHsSource; /*!< 高侧源电流档位 0~15 */
    uint8_t         idriveHsSink;   /*!< 高侧灌电流档位 0~15 */
    uint8_t         idriveLsSource; /*!< 低侧源电流档位 0~15 */
    uint8_t         idriveLsSink;   /*!< 低侧灌电流档位 0~15 */
    TDRIVE_Time_t   tDrive;         /*!< TDRIVE 时间 */
    DeadTime_t      deadTime;       /*!< 数字死区时间 */
    OCP_Mode_t      ocpMode;        /*!< OCP 动作模式 */
    OCP_Deglitch_t  ocpDeglitch;    /*!< OCP 消隐时间 */
    VDS_Level_t     vdsLevel;       /*!< VDS 过流阈值 */
    uint8_t         retryTime;      /*!< 自动重试时间: 0=4ms, 1=50us */
    uint8_t         enableCpuv;     /*!< 使能电荷泵欠压保护: 1=使能, 0=禁用 */
    uint8_t         enableGdf;      /*!< 使能栅极故障检测: 1=使能, 0=禁用 */
    uint8_t         enableOtwReport;/*!< 过热警告是否触发 nFAULT: 1=触发, 0=仅寄存器 */
} DRV8320S_InitTypeDef;
#pragma pack(pop)

/*==================== 故障状态结构体 ====================*/
typedef struct {
    uint8_t fault;      uint8_t vds_ocp;    uint8_t gdf;
    uint8_t uvlo;       uint8_t otsd;       uint8_t vds_ha;
    uint8_t vds_la;     uint8_t vds_hb;     uint8_t vds_lb;
    uint8_t vds_hc;     uint8_t vds_lc;     uint8_t otw;
    uint8_t cpuv;       uint8_t vgs_ha;     uint8_t vgs_la;
    uint8_t vgs_hb;     uint8_t vgs_lb;     uint8_t vgs_hc;
    uint8_t vgs_lc;
} DRV8320S_Fault_t;

/*==================== 句柄结构体 ====================*/
typedef struct {
    SPI_HandleTypeDef *hspi;      /*!< SPI 句柄（已初始化为16位模式） */
    GPIO_TypeDef      *csPort;    /*!< CS 引脚端口 */
    uint16_t           csPin;     /*!< CS 引脚号 */
    GPIO_TypeDef      *enablePort;/*!< ENABLE 引脚端口 */
    uint16_t           enablePin; /*!< ENABLE 引脚号 */
    /* 私有成员，用户无需修改 */
    uint8_t            isInitialized; /*!< 初始化标志 */
} DRV8320S_HandleTypeDef;

/*==================== 对外接口函数 ====================*/

/**
 * @brief 控制DRV8320S使能引脚（高电平使能，低电平禁用）
 * @param hdrv   DRV8320S句柄
 * @param enable 1: 使能芯片, 0: 禁用芯片
 * @retval HAL_OK: 成功; HAL_ERROR: 参数错误
 */
HAL_StatusTypeDef HAL_DRV8320S_Enable(DRV8320S_HandleTypeDef *hdrv, uint8_t enable);

/**
 * @brief 初始化DRV8320S，配置所有参数
 * @param hdrv   DRV8320S句柄（需先设置hspi、csPort、csPin）
 * @param config 配置结构体指针，包含所有运行参数
 * @retval HAL_OK: 成功; HAL_ERROR: 失败
 */
HAL_StatusTypeDef HAL_DRV8320S_Init(DRV8320S_HandleTypeDef *hdrv, const DRV8320S_InitTypeDef *config);

/**
 * @brief 验证SPI通信是否正常
 * @param hdrv DRV8320S句柄
 * @retval HAL_OK: 正常; HAL_ERROR: 异常
 */
HAL_StatusTypeDef HAL_DRV8320S_CheckCommunication(DRV8320S_HandleTypeDef *hdrv);

/**
 * @brief 清除锁存的故障
 * @param hdrv DRV8320S句柄
 */
HAL_StatusTypeDef HAL_DRV8320S_ClearFaults(DRV8320S_HandleTypeDef *hdrv);

/**
 * @brief 读取并解析故障状态
 * @param hdrv  DRV8320S句柄
 * @param fault 输出故障结构体
 */
HAL_StatusTypeDef HAL_DRV8320S_GetFault(DRV8320S_HandleTypeDef *hdrv, DRV8320S_Fault_t *fault);

/**
 * @brief 单独设置PWM模式（运行时动态修改）
 * @param hdrv DRV8320S句柄
 * @param mode PWM模式
 */
HAL_StatusTypeDef HAL_DRV8320S_SetPWMMode(DRV8320S_HandleTypeDef *hdrv, PWM_Mode_t mode);

/**
 * @brief 单独设置高侧IDRIVE
 * @param hdrv       DRV8320S句柄
 * @param sourceVal  源电流档位 0~15
 * @param sinkVal    灌电流档位 0~15
 */
HAL_StatusTypeDef HAL_DRV8320S_SetIDRIVE_HS(DRV8320S_HandleTypeDef *hdrv, uint8_t sourceVal, uint8_t sinkVal);

/**
 * @brief 单独设置低侧IDRIVE
 * @param hdrv       DRV8320S句柄
 * @param sourceVal  源电流档位 0~15
 * @param sinkVal    灌电流档位 0~15
 */
HAL_StatusTypeDef HAL_DRV8320S_SetIDRIVE_LS(DRV8320S_HandleTypeDef *hdrv, uint8_t sourceVal, uint8_t sinkVal);

/**
 * @brief 单独设置TDRIVE
 * @param hdrv DRV8320S句柄
 * @param time TDRIVE时间
 */
HAL_StatusTypeDef HAL_DRV8320S_SetTDRIVE(DRV8320S_HandleTypeDef *hdrv, TDRIVE_Time_t time);

/**
 * @brief 单独设置死区时间
 * @param hdrv     DRV8320S句柄
 * @param deadTime 死区时间
 */
HAL_StatusTypeDef HAL_DRV8320S_SetDeadTime(DRV8320S_HandleTypeDef *hdrv, DeadTime_t deadTime);

/**
 * @brief 单独设置OCP模式
 * @param hdrv  DRV8320S句柄
 * @param mode  OCP模式
 */
HAL_StatusTypeDef HAL_DRV8320S_SetOCPMode(DRV8320S_HandleTypeDef *hdrv, OCP_Mode_t mode);

/**
 * @brief 单独设置OCP消隐时间
 * @param hdrv      DRV8320S句柄
 * @param deglitch  消隐时间
 */
HAL_StatusTypeDef HAL_DRV8320S_SetOCPDeglitch(DRV8320S_HandleTypeDef *hdrv, OCP_Deglitch_t deglitch);

/**
 * @brief 单独设置VDS阈值
 * @param hdrv   DRV8320S句柄
 * @param level  VDS阈值
 */
HAL_StatusTypeDef HAL_DRV8320S_SetVDSLevel(DRV8320S_HandleTypeDef *hdrv, VDS_Level_t level);

/**
 * @brief 单独设置自动重试时间
 * @param hdrv      DRV8320S句柄
 * @param retryMode 0=4ms, 1=50us
 */
HAL_StatusTypeDef HAL_DRV8320S_SetRetryTime(DRV8320S_HandleTypeDef *hdrv, uint8_t retryMode);

/**
 * @brief 使能/禁用电荷泵欠压保护
 * @param hdrv   DRV8320S句柄
 * @param enable 1=使能, 0=禁用
 */
HAL_StatusTypeDef HAL_DRV8320S_EnableChargePumpUVLO(DRV8320S_HandleTypeDef *hdrv, uint8_t enable);

/**
 * @brief 使能/禁用栅极故障检测
 * @param hdrv   DRV8320S句柄
 * @param enable 1=使能, 0=禁用
 */
HAL_StatusTypeDef HAL_DRV8320S_EnableGateFaultDetect(DRV8320S_HandleTypeDef *hdrv, uint8_t enable);

/**
 * @brief 设置过热警告是否报告到nFAULT
 * @param hdrv   DRV8320S句柄
 * @param enable 1=触发nFAULT, 0=仅寄存器
 */
HAL_StatusTypeDef HAL_DRV8320S_SetOTWReport(DRV8320S_HandleTypeDef *hdrv, uint8_t enable);

/**
 * @brief 锁定寄存器（防止意外修改）
 * @param hdrv DRV8320S句柄
 */
HAL_StatusTypeDef HAL_DRV8320S_LockRegisters(DRV8320S_HandleTypeDef *hdrv);

/**
 * @brief 解锁寄存器
 * @param hdrv DRV8320S句柄
 */
HAL_StatusTypeDef HAL_DRV8320S_UnlockRegisters(DRV8320S_HandleTypeDef *hdrv);

/**
 * @brief 读取 DRV8320S 当前关键配置（从硬件寄存器）
 * @param hdrv   DRV8320S句柄
 * @param config 输出配置结构体指针
 * @retval HAL_OK: 成功; HAL_ERROR: 失败
 */
HAL_StatusTypeDef HAL_DRV8320S_GetConfig(DRV8320S_HandleTypeDef *hdrv, DRV8320S_Config_t *config);

extern DRV8320S_HandleTypeDef hdrv8320s;

#endif /* DRV8320S_H */