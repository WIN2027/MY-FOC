#include "main.h"
#include "cmsis_os.h"
#include "Init.h"
#include "FreeRTOS_Task.h"


int main(void)
{

    Peripheral_Init();
    osKernelInitialize();
    FreeRTOS_TaskInit();
    osKernelStart();

    // ReSharper disable once CppDFAEndlessLoop
    while (1){}

}