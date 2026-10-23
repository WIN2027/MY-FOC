#include "FreeRTOS_Task.h"

#include <math.h>

#include "ADC.h"
#include "cmsis_os.h"
#include "stm32g4xx_hal.h"
#include "FOC.h"
#include "UART.h"
#include "Init.h"


osThreadId_t FOC_TaskHandle;
osThreadId_t UART_TaskHandle;
osThreadId_t Telemetry_TaskHandle;
osMessageQueueId_t UART_Tx_Queue;

static const osThreadAttr_t UART_TaskHandle_Attr = {.name = "UART_TaskHandle",.stack_size = 1024,.priority = osPriorityNormal};
static const osThreadAttr_t Telemetry_TaskHandle_Attr = {.name = "Telemetry_TaskHandle",.stack_size = 1024,.priority = osPriorityNormal};

static void UART_Task(void *pvParameters)
{
    // ReSharper disable once CppDFAEndlessLoop
    while (1)
    {
        const uint32_t flags = osThreadFlagsWait(BIT_RX_READY | BIT_TX_QUEUE,osFlagsWaitAny, osWaitForever);

        if (flags & BIT_RX_READY)
        {
            // 处理接收到的帧
            // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            SerialCom_ProcessFrame(DMA_Rx_Buf, Rx_Len);
            // 重新启动DMA接收
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, DMA_Rx_Buf, DMA_RX_BUF_SIZE);
            // 重启会使能HT中断，必须再次关闭，防止半帧被当作完整帧解析
            __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
        }
        if (flags & BIT_TX_QUEUE) {
            // 处理发送队列中的所有消息
            UART_TxMsg_t msg;
            while (osMessageQueueGet(UART_Tx_Queue, &msg, NULL, 0U) == osOK)
            {
                // 等待上一次DMA发送完成
                while (Tx_Busy) { /* 忙等待，但这里只在任务中，不会长时间阻塞 */ }
                // 启动DMA发送
                Tx_Busy = true;
                HAL_UART_Transmit_DMA(&huart3, msg.data, msg.len);
                // 注意：发送完成后中断回调会置 Tx_Busy = false
            }
        }
    }
}

static void Telemetry_Task(void *pvParameters)
{
    // ReSharper disable once CppDFAEndlessLoop
    while (1)
    {
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

        FOC_Telemetry_t tele;
        FOC_GetTelemetry(&tele);

        // 打包遥测数据（根据掩码）
        uint8_t buffer[64];
        uint8_t len = 0;
        // 以下为示例打包逻辑，您可根据实际需要实现
        if (g_telemetry_mask & 0x0001)
        {
            const uint16_t Tamp = (uint16_t)(ADC1_Prev * 16.0f);
            buffer[len++] = (uint8_t)(Tamp & 0xFF);
            buffer[len++] = (uint8_t)((Tamp >> 8) & 0xFF);
        }   // ADC1源码
        if (g_telemetry_mask & 0x0002)
        {
            const uint16_t Tamp = (uint16_t)(ADC2_Prev * 16.0f);
            buffer[len++] = (uint8_t)(Tamp & 0xFF);
            buffer[len++] = (uint8_t)((Tamp>> 8) & 0xFF);
        }   // ADC2源码
        if (g_telemetry_mask & 0x0004)
        {
            buffer[len++] = (uint8_t)(tele.IU & 0xFF);
            buffer[len++] = (uint8_t)((tele.IU >> 8) & 0xFF);
        }   // 三相电流
        if (g_telemetry_mask & 0x0008)
        {
            buffer[len++] = (uint8_t)(tele.IV & 0xFF);
            buffer[len++] = (uint8_t)((tele.IV >> 8) & 0xFF);
        }   // 三相电流
        if (g_telemetry_mask & 0x0010)
        {
            buffer[len++] = (uint8_t)(tele.IW & 0xFF);
            buffer[len++] = (uint8_t)((tele.IW >> 8) & 0xFF);
        }   // 三相电流
        if (g_telemetry_mask & 0x0020)
        {
            buffer[len++] = (uint8_t)((int16_t)(pi_d.actual*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_d.actual*100.0f) >> 8) & 0xFF);
        }   // D轴电流
        if (g_telemetry_mask & 0x0040)
        {
            buffer[len++] = (uint8_t)((int16_t)(pi_q.actual *100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_q.actual*100.0f) >> 8) & 0xFF);
        }   // Q轴电流
        if (g_telemetry_mask & 0x0080)
        {
            buffer[len++] = (uint8_t)(smo.speed_rpm & 0xFF);
            buffer[len++] = (uint8_t)((smo.speed_rpm >> 8) & 0xFF);
        }   // 转速
        if (g_telemetry_mask & 0x0100)
        {
            buffer[len++] = (uint8_t)(tele.foc_dwt & 0xFF);
            buffer[len++] = (uint8_t)((tele.foc_dwt >> 8) & 0xFF);
        }   // 计算周期
        if (g_telemetry_mask & 0x0200)
        {
            buffer[len++] = (uint8_t)(PWM.PWM_U & 0xFF);
            buffer[len++] = (uint8_t)((PWM.PWM_U >> 8) & 0xFF);
            buffer[len++] = (uint8_t)(PWM.PWM_V & 0xFF);
            buffer[len++] = (uint8_t)((PWM.PWM_V >> 8) & 0xFF);
            buffer[len++] = (uint8_t)(PWM.PWM_W & 0xFF);
            buffer[len++] = (uint8_t)((PWM.PWM_W >> 8) & 0xFF);
        }   //三相占空比
        if (g_telemetry_mask & 0x0400)
        {
            buffer[len++] = (uint8_t)((int16_t)(pi_d.out*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_d.out*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_d.error*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_d.error*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_d.integral*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_d.integral*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_d.actual *100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_d.actual*100.0f) >> 8) & 0xFF);
        }   //D轴PI
        if (g_telemetry_mask & 0x0800)
        {
            // buffer[len++] = (uint8_t)((int16_t)(TestA*100.0f) & 0xFF);
            // buffer[len++] = (uint8_t)(((int16_t)(TestA*100.0f) >> 8) & 0xFF);
            // buffer[len++] = (uint8_t)((int16_t)(TestB*100.0f) & 0xFF);
            // buffer[len++] = (uint8_t)(((int16_t)(TestB*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_q.out*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_q.out*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_q.error*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_q.error*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_q.integral*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_q.integral*100.0f) >> 8) & 0xFF);
            buffer[len++] = (uint8_t)((int16_t)(pi_q.actual *100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(pi_q.actual*100.0f) >> 8) & 0xFF);
        }   //Q轴PI
        if (g_telemetry_mask & 0x1000)
        {
            buffer[len++] = (uint8_t)((int16_t)(smo.v_est*100.0f) & 0xFF);
            buffer[len++] = (uint8_t)(((int16_t)(smo.v_est*100.0f) >> 8) & 0xFF);
        }   //反电动势

        // ... 继续其他项，注意 float 占4字节，需使用联合或指针转换

        // 发送上报帧（命令码 CMD_TELEMETRY_REPORT）
        UART_SendResponse(CMD_TELEMETRY_REPORT, buffer, len);
    }
}

void FreeRTOS_TaskInit(void)
{

    UART_Tx_Queue = osMessageQueueNew(10, sizeof(UART_TxMsg_t), NULL);   // 队列深度5

    // FOC_TaskHandle = osThreadNew(FOC_Task, NULL, &FOC_TaskHandle_Attr);
    UART_TaskHandle = osThreadNew(UART_Task, NULL, &UART_TaskHandle_Attr);
    Telemetry_TaskHandle = osThreadNew(Telemetry_Task, NULL, &Telemetry_TaskHandle_Attr);
}

