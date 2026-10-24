#include "FreeRTOS_Task.h"

#include "cmsis_os.h"
#include "stm32g4xx_hal.h"
#include "UART.h"

osThreadId_t UART_TaskHandle;
osThreadId_t Telemetry_TaskHandle;

static const osThreadAttr_t UART_TaskHandle_Attr = {
    .name = "UART_TaskHandle",
    .stack_size = 1536,
    .priority = osPriorityNormal,
};

static const osThreadAttr_t Telemetry_TaskHandle_Attr = {
    .name = "Telemetry_TaskHandle",
    .stack_size = 1024,
    .priority = osPriorityNormal,
};

/**
 * @brief 串口协议周期任务
 * @note  协议解析在 UART 中断中完成，业务回调（含 DRV8320S 阻塞 SPI/
 *        HAL_Delay）在本任务中执行；周期 1 ms。
 */
static void UART_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        SerialCom_Process();
        osDelay(1);
    }
}

/**
 * @brief 遥测任务：TIM4 每约 4.17 ms 置位一次，按掩码打包 0x18 帧
 */
static void Telemetry_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        (void)osThreadFlagsWait(0x01U, osFlagsWaitAny, osWaitForever);
        UART_SendTelemetryReport();
    }
}

void FreeRTOS_TaskInit(void)
{
    UART_TaskHandle      = osThreadNew(UART_Task, NULL, &UART_TaskHandle_Attr);
    Telemetry_TaskHandle = osThreadNew(Telemetry_Task, NULL, &Telemetry_TaskHandle_Attr);
}
