#ifndef ADC_H
#define ADC_H

/**
 * @file ADC.h
 * @brief 限幅 + 一阶低通 IIR 滤波模块（仅处理原始ADC码值）
 * @note  由外部中断服务程序调用 ADC_Update() 传入最新码值，
 *        模块内部实现尖峰限幅与一阶 IIR 滤波。
 */

#include <stdint.h>
#include <stdbool.h>

/* ------------------- API 函数 ------------------- */

void ADC_Init();
float ADC1_Filter(uint16_t raw);
float ADC2_Filter(uint16_t raw);


extern float ADC1_Prev;
extern float ADC2_Prev;
extern volatile bool ADC1_Ready;
extern volatile bool ADC2_Ready;

#endif /* ADC_H */