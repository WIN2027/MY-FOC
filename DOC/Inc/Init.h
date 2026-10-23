#ifndef INIT_H
#define INIT_H

#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern HRTIM_HandleTypeDef hhrtim1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern CORDIC_HandleTypeDef hcordic;

extern volatile uint16_t ADC1_DMA_BUF;
extern volatile uint16_t ADC2_DMA_BUF;



void Peripheral_Init(void);

#endif //INIT_H