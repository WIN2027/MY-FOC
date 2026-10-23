#include "DRV8320S.h"

DRV8320S_HandleTypeDef hdrv8320s;

/* 辅助函数：SPI 读写 16 位 (MCU 作为 master，DRV8320S 作为 slave) */
static HAL_StatusTypeDef DRV8320S_SPI_Transceive(DRV8320S_HandleTypeDef *hdrv, uint16_t txData, uint16_t *rxData)
{
    if (hdrv == NULL || hdrv->hspi == NULL) return HAL_ERROR;
    HAL_GPIO_WritePin(hdrv->csPort, hdrv->csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(hdrv->hspi, (uint8_t*)&txData, (uint8_t*)rxData, 1, 100);
    HAL_GPIO_WritePin(hdrv->csPort, hdrv->csPin, GPIO_PIN_SET);
    HAL_Delay(1);
    return ret;
}

/* 写寄存器：regAddr 4 位，data 低 11 位有效 */
static HAL_StatusTypeDef DRV8320S_WriteReg(DRV8320S_HandleTypeDef *hdrv, uint8_t regAddr, uint16_t data)
{
    if (regAddr > 0x0F) return HAL_ERROR;
    uint16_t txWord = (0 << 15) | ((regAddr & 0x0F) << 11) | (data & 0x07FF);
    uint16_t rxWord;
    return DRV8320S_SPI_Transceive(hdrv, txWord, &rxWord);
}

/* 读寄存器 */
static HAL_StatusTypeDef DRV8320S_ReadReg(DRV8320S_HandleTypeDef *hdrv, uint8_t regAddr, uint16_t *data)
{
    if (regAddr > 0x0F || data == NULL) return HAL_ERROR;
    uint16_t txWord = (1 << 15) | ((regAddr & 0x0F) << 11);
    uint16_t rxWord;
    HAL_StatusTypeDef ret = DRV8320S_SPI_Transceive(hdrv, txWord, &rxWord);
    if (ret == HAL_OK) *data = rxWord & 0x07FF;
    return ret;
}

/* 初始化：保存SPI句柄和CS引脚，配置所有参数 */
HAL_StatusTypeDef HAL_DRV8320S_Init(DRV8320S_HandleTypeDef *hdrv, const DRV8320S_InitTypeDef *config)
{
    if (hdrv == NULL || config == NULL || hdrv->hspi == NULL) return HAL_ERROR;
    hdrv->isInitialized = 0;

    /* 1. 验证通信 */
    if (HAL_DRV8320S_CheckCommunication(hdrv) != HAL_OK) return HAL_ERROR;

    /* 2. 按顺序写入配置 */
    if (HAL_DRV8320S_SetPWMMode(hdrv, config->pwmMode) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetIDRIVE_HS(hdrv, config->idriveHsSource, config->idriveHsSink) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetIDRIVE_LS(hdrv, config->idriveLsSource, config->idriveLsSink) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetTDRIVE(hdrv, config->tDrive) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetDeadTime(hdrv, config->deadTime) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetOCPMode(hdrv, config->ocpMode) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetOCPDeglitch(hdrv, config->ocpDeglitch) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetVDSLevel(hdrv, config->vdsLevel) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetRetryTime(hdrv, config->retryTime) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_EnableChargePumpUVLO(hdrv, config->enableCpuv) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_EnableGateFaultDetect(hdrv, config->enableGdf) != HAL_OK) return HAL_ERROR;
    if (HAL_DRV8320S_SetOTWReport(hdrv, config->enableOtwReport) != HAL_OK) return HAL_ERROR;

    hdrv->isInitialized = 1;
    return HAL_OK;
}

/* 简单验证：读 Fault1 寄存器，若读回 0x7FF 则认为无响应 */
HAL_StatusTypeDef HAL_DRV8320S_CheckCommunication(DRV8320S_HandleTypeDef *hdrv)
{
    uint16_t status;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT1, &status) != HAL_OK) return HAL_ERROR;
    if (status == 0x7FF) return HAL_ERROR;
    return HAL_OK;
}

/**
 * @brief 控制DRV8320S使能引脚
 * @param hdrv   DRV8320S句柄
 * @param enable 1: 使能芯片, 0: 禁用芯片
 * @retval HAL_OK: 成功; HAL_ERROR: 句柄或引脚无效
 */
HAL_StatusTypeDef HAL_DRV8320S_Enable(DRV8320S_HandleTypeDef *hdrv, uint8_t enable)
{
    if (hdrv == NULL || hdrv->enablePort == NULL) return HAL_ERROR;
    GPIO_PinState pinState = (enable != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(hdrv->enablePort, hdrv->enablePin, pinState);
    return HAL_OK;
}

/* ========== 参数配置接口实现 ========== */

HAL_StatusTypeDef HAL_DRV8320S_SetPWMMode(DRV8320S_HandleTypeDef *hdrv, PWM_Mode_t mode)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_DRIVER_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x03 << 5);
    reg |= ((mode & 0x03) << 5);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_DRIVER_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetIDRIVE_HS(DRV8320S_HandleTypeDef *hdrv, uint8_t sourceVal, uint8_t sinkVal)
{
    if (sourceVal > 15 || sinkVal > 15) return HAL_ERROR;
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x0F << 4);
    reg &= ~(0x0F << 0);
    reg |= ((sourceVal & 0x0F) << 4);
    reg |= (sinkVal & 0x0F);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetIDRIVE_LS(DRV8320S_HandleTypeDef *hdrv, uint8_t sourceVal, uint8_t sinkVal)
{
    if (sourceVal > 15 || sinkVal > 15) return HAL_ERROR;
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_LS, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x0F << 4);
    reg &= ~(0x0F << 0);
    reg |= ((sourceVal & 0x0F) << 4);
    reg |= (sinkVal & 0x0F);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_GATE_DRIVE_LS, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetTDRIVE(DRV8320S_HandleTypeDef *hdrv, TDRIVE_Time_t time)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_LS, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x03 << 8);
    reg |= ((time & 0x03) << 8);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_GATE_DRIVE_LS, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetDeadTime(DRV8320S_HandleTypeDef *hdrv, DeadTime_t deadTime)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_OCP_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x03 << 8);
    reg |= ((deadTime & 0x03) << 8);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_OCP_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetOCPMode(DRV8320S_HandleTypeDef *hdrv, OCP_Mode_t mode)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_OCP_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x03 << 6);
    reg |= ((mode & 0x03) << 6);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_OCP_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetOCPDeglitch(DRV8320S_HandleTypeDef *hdrv, OCP_Deglitch_t deglitch)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_OCP_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x03 << 4);
    reg |= ((deglitch & 0x03) << 4);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_OCP_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetVDSLevel(DRV8320S_HandleTypeDef *hdrv, VDS_Level_t level)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_OCP_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x0F << 0);
    reg |= (level & 0x0F);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_OCP_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetRetryTime(DRV8320S_HandleTypeDef *hdrv, uint8_t retryMode)
{
    if (retryMode > 1) return HAL_ERROR;
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_OCP_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(1 << 10);
    reg |= ((retryMode & 0x01) << 10);
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_OCP_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_EnableChargePumpUVLO(DRV8320S_HandleTypeDef *hdrv, uint8_t enable)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_DRIVER_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    if (enable) {
        reg &= ~(1 << 9);  // DIS_CPUV = 0
    } else {
        reg |= (1 << 9);   // DIS_CPUV = 1
    }
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_DRIVER_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_EnableGateFaultDetect(DRV8320S_HandleTypeDef *hdrv, uint8_t enable)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_DRIVER_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    if (enable) {
        reg &= ~(1 << 8);  // DIS_GDF = 0
    } else {
        reg |= (1 << 8);   // DIS_GDF = 1
    }
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_DRIVER_CTRL, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_SetOTWReport(DRV8320S_HandleTypeDef *hdrv, uint8_t enable)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_DRIVER_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    if (enable) {
        reg |= (1 << 7);   // OTW_REP = 1
    } else {
        reg &= ~(1 << 7);  // OTW_REP = 0
    }
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_DRIVER_CTRL, reg);
}

/* 故障查询与清除 */
HAL_StatusTypeDef HAL_DRV8320S_GetFaultStatus1(DRV8320S_HandleTypeDef *hdrv, uint16_t *status)
{
    return DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT1, status);
}

HAL_StatusTypeDef HAL_DRV8320S_GetFaultStatus2(DRV8320S_HandleTypeDef *hdrv, uint16_t *status)
{
    return DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT2, status);
}

HAL_StatusTypeDef HAL_DRV8320S_GetFault(DRV8320S_HandleTypeDef *hdrv, DRV8320S_Fault_t *fault)
{
    if (fault == NULL) return HAL_ERROR;
    uint16_t reg1, reg2;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT1, &reg1) != HAL_OK) return HAL_ERROR;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT2, &reg2) != HAL_OK) return HAL_ERROR;
    fault->fault    = (reg1 >> 10) & 0x01;
    fault->vds_ocp  = (reg1 >> 9) & 0x01;
    fault->gdf      = (reg1 >> 8) & 0x01;
    fault->uvlo     = (reg1 >> 7) & 0x01;
    fault->otsd     = (reg1 >> 6) & 0x01;
    fault->vds_ha   = (reg1 >> 5) & 0x01;
    fault->vds_la   = (reg1 >> 4) & 0x01;
    fault->vds_hb   = (reg1 >> 3) & 0x01;
    fault->vds_lb   = (reg1 >> 2) & 0x01;
    fault->vds_hc   = (reg1 >> 1) & 0x01;
    fault->vds_lc   = (reg1 >> 0) & 0x01;
    fault->otw      = (reg2 >> 7) & 0x01;
    fault->cpuv     = (reg2 >> 6) & 0x01;
    fault->vgs_ha   = (reg2 >> 5) & 0x01;
    fault->vgs_la   = (reg2 >> 4) & 0x01;
    fault->vgs_hb   = (reg2 >> 3) & 0x01;
    fault->vgs_lb   = (reg2 >> 2) & 0x01;
    fault->vgs_hc   = (reg2 >> 1) & 0x01;
    fault->vgs_lc   = (reg2 >> 0) & 0x01;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DRV8320S_ClearFaults(DRV8320S_HandleTypeDef *hdrv)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_DRIVER_CTRL, &reg) != HAL_OK) return HAL_ERROR;
    reg |= (1 << 0);  // set CLR_FLT bit
    HAL_StatusTypeDef ret = DRV8320S_WriteReg(hdrv, DRV8320S_REG_DRIVER_CTRL, reg);
    /* CLR_FLT 位会自动清零，无需再次写入清除 */
    return ret;
}

/* 锁寄存器功能 */
HAL_StatusTypeDef HAL_DRV8320S_LockRegisters(DRV8320S_HandleTypeDef *hdrv)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x07 << 8);
    reg |= (0x06 << 8);  // 110b 锁存
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_UnlockRegisters(DRV8320S_HandleTypeDef *hdrv)
{
    uint16_t reg;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, &reg) != HAL_OK) return HAL_ERROR;
    reg &= ~(0x07 << 8);
    reg |= (0x03 << 8);  // 011b 解锁
    return DRV8320S_WriteReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, reg);
}

HAL_StatusTypeDef HAL_DRV8320S_GetConfig(DRV8320S_HandleTypeDef *hdrv, DRV8320S_Config_t *config)
{
    if (hdrv == NULL || config == NULL) return HAL_ERROR;

    uint16_t reg_val;

    // 读取 Driver Control 寄存器
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_DRIVER_CTRL, &reg_val) != HAL_OK) return HAL_ERROR;
    config->pwm_mode      = (reg_val >> 5) & 0x03;
    config->cpuv_enabled  = ((reg_val >> 9) & 0x01) ? 0 : 1;   // DIS_CPUV=0 表示使能
    config->gdf_enabled   = ((reg_val >> 8) & 0x01) ? 0 : 1;   // DIS_GDF=0 表示使能
    config->otw_report    = (reg_val >> 7) & 0x01;

    // 读取高侧驱动寄存器
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_HS, &reg_val) != HAL_OK) return HAL_ERROR;
    config->idrive_hs_source = (reg_val >> 4) & 0x0F;
    config->idrive_hs_sink   = reg_val & 0x0F;

    // 读取低侧驱动寄存器 (同时包含 TDRIVE)
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_GATE_DRIVE_LS, &reg_val) != HAL_OK) return HAL_ERROR;
    config->idrive_ls_source = (reg_val >> 4) & 0x0F;
    config->idrive_ls_sink   = reg_val & 0x0F;
    config->tdrive           = (reg_val >> 8) & 0x03;

    // 读取 OCP 控制寄存器
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_OCP_CTRL, &reg_val) != HAL_OK) return HAL_ERROR;
    config->dead_time     = (reg_val >> 8) & 0x03;
    config->ocp_mode      = (reg_val >> 6) & 0x03;
    config->ocp_deglitch  = (reg_val >> 4) & 0x03;
    config->vds_level     = reg_val & 0x0F;
    config->retry_mode    = (reg_val >> 10) & 0x01;

    // 读取故障寄存器（用于诊断）
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT1, &config->fault1) != HAL_OK) return HAL_ERROR;
    if (DRV8320S_ReadReg(hdrv, DRV8320S_REG_FAULT2, &config->fault2) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}