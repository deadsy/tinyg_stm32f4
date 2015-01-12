//-----------------------------------------------------------------------------
/*

TinyG RTC functions

*/
//-----------------------------------------------------------------------------

#include <inttypes.h>

#include "tinyg.h"
#include "config.h"
#include "report.h"
#include "gpio.h"
#include "stepper.h"
#include "rtc.h"

//-----------------------------------------------------------------------------

rtClock_t rtc;

//-----------------------------------------------------------------------------
// Called at system startup

void rtc_init(void)
{
  rtc.clock_ticks = 0;
  rtc.magic_end = MAGICNUM;
}

//-----------------------------------------------------------------------------
// Called every RTC_MILLISECONDS from the SysTick_Handler ISR

void rtc_isr(void)
{
  // rtc callbacks
  gpio_rtc_callback(); // switch debouncing
  rpt_status_report_rtc_callback(); // status report timing
  st_disable_motors_rtc_callback(); // stepper disable timer

  rtc.clock_ticks ++;
}

//-----------------------------------------------------------------------------
