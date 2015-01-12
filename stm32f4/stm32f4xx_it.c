//-----------------------------------------------------------------------------
/*

Interrupt Handlers

*/
//-----------------------------------------------------------------------------

#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"

#include "rtc.h"

//-----------------------------------------------------------------------------

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1);
}

void MemManage_Handler(void)
{
    while (1);
}

void BusFault_Handler(void)
{
    while (1);
}

void UsageFault_Handler(void)
{
    while (1);
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    uint32_t ticks = HAL_GetTick();

    if ((ticks & (RTC_MILLISECONDS - 1)) == 0) {
      rtc_isr();
    }

    HAL_IncTick();
}

//-----------------------------------------------------------------------------
