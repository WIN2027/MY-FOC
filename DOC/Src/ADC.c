#include "ADC.h"
#include <stdlib.h>   // 为 abs() 提供原型
#include "stm32g4xx_hal.h"

/**
 * @file ADC.c
 * @brief 限幅 + 一阶低通 IIR 滤波实现
 */


__attribute__((section(".ccmram_data")))volatile bool ADC1_Ready = false;
__attribute__((section(".ccmram_data")))volatile bool ADC2_Ready = false;

__attribute__((section(".ccmram_data")))uint16_t ADC1_BUF[3];
__attribute__((section(".ccmram_data")))uint16_t ADC2_BUF[3];

__attribute__((section(".ccmram_data")))float ADC1_Prev;
__attribute__((section(".ccmram_data")))float ADC2_Prev;


void ADC_Init()
{
    ADC1_BUF[0] = 2048;ADC1_BUF[1] = 2048;ADC1_BUF[2] = 2048;
    ADC2_BUF[0] = 2048;ADC2_BUF[1] = 2048;ADC2_BUF[2] = 2048;
}

__attribute__((section(".ccmram_text")))float ADC1_Filter(uint16_t raw)
{

    // 更新历史窗口（循环移位）
    ADC1_BUF[0] = ADC1_BUF[1];
    ADC1_BUF[1] = ADC1_BUF[2];
    ADC1_BUF[2] = raw;

    // ---------- 中值滤波（内联排序，三个数） ----------
    uint16_t a = ADC1_BUF[0], b = ADC1_BUF[1], c = ADC1_BUF[2];
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    const float median = (float)b;   // 中值

    // ---------- 一阶 IIR 低通 ----------
    const float new_out = 0.7f * median + 0.3f * ADC1_Prev;
    ADC1_Prev = new_out;
    return new_out;
}
__attribute__((section(".ccmram_text")))float ADC2_Filter(uint16_t raw)
{

    // 更新历史窗口（循环移位）
    ADC2_BUF[0] = ADC2_BUF[1];
    ADC2_BUF[1] = ADC2_BUF[2];
    ADC2_BUF[2] = raw;

    // ---------- 中值滤波（内联排序，三个数） ----------
    uint16_t a = ADC2_BUF[0], b = ADC2_BUF[1], c = ADC2_BUF[2];
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    const float median = (float)b;   // 中值

    // ---------- 一阶 IIR 低通 ----------
    const float new_out = 0.7f * median + 0.3f * ADC2_Prev;
    ADC2_Prev = new_out;
    return new_out;
}