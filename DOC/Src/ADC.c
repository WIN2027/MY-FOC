#include "ADC.h"
#define ADC_FILTER_DATA_CCM
#include "FOC_ADC.h"

/**
 * @file ADC.c
 * @brief 应用层 ADC 适配：把 FOC_ADC 可移植模块接到现有工程
 *
 * 与旧实现的区别：
 *   - 滤波/换算由 FOC_ADC 模块完成，ADC1_Filter()/ADC2_Filter() 直接返回安培值；
 *   - ADC1_Prev/ADC2_Prev 仍保存“滤波后的 ADC 码值”，保持遥测打包代码不变；
 *   - 模块实例放 CCM，函数仍在 Flash，避免引入 CCM 取指配置。
 *
 * 模块来源：模块/FOC_ADC（v1.3，commit 4f93491），FOC_ADC.c/h 内容未修改。
 */

/* ------------------- 对外状态（保持旧工程符号名） ------------------- */
ADC_FILTER_DATA volatile bool ADC1_Ready = false;
ADC_FILTER_DATA volatile bool ADC2_Ready = false;
ADC_FILTER_DATA volatile bool ADC_SelfTest_OK = false;

ADC_FILTER_DATA float ADC1_Prev = 0.0f;
ADC_FILTER_DATA float ADC2_Prev = 0.0f;

/* ------------------- FOC_ADC 模块实例 ------------------- */
ADC_FILTER_DATA ADC_Filter_t s_adc1_filter;
ADC_FILTER_DATA ADC_Filter_t s_adc2_filter;

void ADC_Init(void)
{
    ADC1_Ready = false;
    ADC2_Ready = false;

    ADC_Filter_Init(&s_adc1_filter, ADC_FILTER_ALPHA, ADC_FILTER_MIDPOINT, ADC_FILTER_CURRENT_SCALE);
    ADC_Filter_Init(&s_adc2_filter, ADC_FILTER_ALPHA, ADC_FILTER_MIDPOINT, ADC_FILTER_CURRENT_SCALE);

    /* 首拍遥测码值 = 中点，保持与旧 ADC_Init() 的 2048 初值一致 */
    ADC1_Prev = ADC_Filter_GetFilteredCode(&s_adc1_filter);
    ADC2_Prev = ADC_Filter_GetFilteredCode(&s_adc2_filter);

#if ADC_FILTER_ENABLE_SELFTEST
    ADC_SelfTest_OK = ADC_SelfTest();
#else
    ADC_SelfTest_OK = true;
#endif
}

// ADC_FILTER_TEXT float ADC1_Filter(uint16_t raw)
// {
//     const float current = ADC_Filter_Update(&s_adc1_filter, raw);
//     ADC1_Prev = ADC_Filter_GetFilteredCode(&s_adc1_filter);
//     return current;
// }
//
// ADC_FILTER_TEXT float ADC2_Filter(uint16_t raw)
// {
//     const float current = ADC_Filter_Update(&s_adc2_filter, raw);
//     ADC2_Prev = ADC_Filter_GetFilteredCode(&s_adc2_filter);
//     return current;
// }
