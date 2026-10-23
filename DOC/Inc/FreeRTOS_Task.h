#ifndef FreeRTOS_Task_H
#define FreeRTOS_Task_H

#include "cmsis_os2.h"

extern osThreadId_t SpeedLoop_TaskHandle;
extern osThreadId_t FOC_TaskHandle;
extern osThreadId_t UART_TaskHandle;
extern osThreadId_t Telemetry_TaskHandle;
extern osMessageQueueId_t UART_Tx_Queue;

void FreeRTOS_TaskInit(void);

#endif //FreeRTOS_Task_H