#include "system_stm32f10x.h"
#include "task.h"
#include "timProcessing.h"
#include "sensorProcessing.h"

int main(void)
{
    SystemInit();
    clockInit();
    task_nrf24l01();
    while(1)
    {
    }
}
