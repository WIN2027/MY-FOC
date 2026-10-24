#ifndef FreeRTOS_Task_H
#define FreeRTOS_Task_H

#include "cmsis_os2.h"

extern osThreadId_t UART_TaskHandle;
extern osThreadId_t Telemetry_TaskHandle;

void FreeRTOS_TaskInit(void);

#endif //FreeRTOS_Task_H
