/**********************************************
 * 中断服务函数 (除RTC外, RTC_IRQHandler在main.c中)
 **********************************************/
#include "config.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void)
{
    while(1)
    {
    }
}

void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while(1)
    {
    }
}
