#ifndef ADC_H
#define ADC_H

/**
 * @file ADC.h
 * @brief 应用层 ADC 适配接口：FOC_ADC 滤波模块 + 双 ADC 就绪标志
 * @note  ADC1_Filter()/ADC2_Filter() 输入原始 ADC 码值，返回相电流 (A)。
 *        内部处理链：3 点中值 -> 一阶 IIR -> (code - midpoint) * scale，
 *        由可移植模块 FOC_ADC 实现；本文件只保留工程侧的参数与遥测兼容接口。
 */

#include <stdint.h>
#include <stdbool.h>

/* IIR 系数，与旧工程一致：0.7 -> 48 kHz 下约 10.6 kHz 截止 */
#ifndef ADC_FILTER_ALPHA
#define ADC_FILTER_ALPHA      0.7f
#endif

/* 零电流中点与码值->电流系数（沿用旧工程标定值） */
#ifndef ADC_FILTER_MIDPOINT
#define ADC_FILTER_MIDPOINT   2048.0f
#endif
#ifndef ADC_FILTER_CURRENT_SCALE
#define ADC_FILTER_CURRENT_SCALE 0.0179f
#endif

/* ------------------- API 函数 ------------------- */

/** @brief 初始化两路滤波器（含模块自检） */
void ADC_Init(void);

/** @brief 输入 ADC1 原始码值，返回 U 相电流 (A) */
float ADC1_Filter(uint16_t raw);

/** @brief 输入 ADC2 原始码值，返回 V 相电流 (A) */
float ADC2_Filter(uint16_t raw);

/* ------------------- 遥测/状态兼容接口 ------------------- */

/** @brief 最近一次滤波后的 ADC1 码值（用于现有遥测打包） */
extern float ADC1_Prev;

/** @brief 最近一次滤波后的 ADC2 码值（用于现有遥测打包） */
extern float ADC2_Prev;

extern volatile bool ADC1_Ready;
extern volatile bool ADC2_Ready;

/** @brief FOC_ADC 模块自检结果；false 时应检查构建/参数 */
extern volatile bool ADC_SelfTest_OK;

#endif /* ADC_H */
