#include "system_stm32f10x.h"
#include "task.h"

int main(void)
{
    SystemInit();
    task_nrf24l01();
    while(1)
    {
    }
}
