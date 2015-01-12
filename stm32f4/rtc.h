//-----------------------------------------------------------------------------

#ifndef STM32F4_RTC_H
#define STM32F4_RTC_H

//-----------------------------------------------------------------------------

// Increment the rtc ticks every RTC_MILLISECONDS, must be a power of 2
#define RTC_MILLISECONDS 8

typedef struct rtClock {
  volatile uint32_t clock_ticks;
  uint16_t magic_end;
} rtClock_t;

extern rtClock_t rtc;

//-----------------------------------------------------------------------------

void rtc_init(void);
void rtc_isr(void);

//-----------------------------------------------------------------------------

#endif // STM32F4_RTC_H

//-----------------------------------------------------------------------------
