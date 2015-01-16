//-----------------------------------------------------------------------------
/*

GPIO Pins for the STM32F4 Discovery Board

Pin Assignments for STM32F4 Discovery Board
* = CNC function

PA0 = push button
PA1 = system_reset
PA2 = uart_tx *
PA3 = uart_rx *
PA4 = codec
PA5 = accel
PA6 = accel
PA7 = accel
PA8
PA9 = usb
PA10 = usb
PA11 = usb (not on header)
PA12 = usb (not on header)
PA13 = swd
PA14 = swd
PA15 =

PB0 = gs0_stepx *
PB1 = gs0_stepy *
PB2 = gs0_stepz *
PB3 = swd
PB4 = gs1_stepx *
PB5 = gs1_stepy *
PB6 = codec
PB7 = gs1_stepz *
PB8 = gs2_stepx *
PB9 = codec
PB10 = mic
PB11 = gs2_stepy *
PB12 = gs2_stepz *
PB13 = gs0_enable *
PB14 = gs1_enable *
PB15 = gs2_enable *

PC0 = usb
PC1 = limit_switch_x *
PC2 = limit_switch_y *
PC3 = mic
PC4 = codec
PC5 = limit_switch_z *
PC6
PC7 = codec
PC8
PC9
PC10 = codec
PC11
PC12 = codec
PC13
PC14 = osc_in
PC15 = osc_out

PD0 = gs0_dirnx *
PD1 = gs0_dirny *
PD2 = gs0_dirnz *
PD3 = gs1_dirnx *
PD4 = gs1_dirny
PD5 = usb
PD6 =
PD7 = gs1_dirnz *
PD8 = gs2_dirnx *
PD9 = gs2_dirny *
PD10 = gs2_dirnz *
PD11
PD12 = led
PD13 = led
PD14 = led
PD15 = led

PE0 = accel
PE1 = accel
PE2
PE3 = accel
PE4
PE5
PE6
PE7
PE8
PE9
PE10
PE11
PE12
PE13
PE14
PE15

PH0 = ph0_osc_in
PH1 = ph1_osc_out

*/
//-----------------------------------------------------------------------------

#ifndef GPIO_PINS_H
#define GPIO_PINS_H

//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

//-----------------------------------------------------------------------------
// port numbers

#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5
#define PORTG 6
#define PORTH 7
#define PORTI 8

//-----------------------------------------------------------------------------
// gpio macros

#define GPIO_NUM(port, pin) ((port << 4) | (pin))
#define GPIO_PORT(n) (n >> 4)
#define GPIO_PIN(n) (n & 0xf)
#define GPIO_BIT(n) (1 << GPIO_PIN(n))
#define GPIO_BASE(n) ((GPIO_TypeDef  *)(GPIOA_BASE + (GPIO_PORT(n) * 0x400)))

//-----------------------------------------------------------------------------
// gpio assignments

// standard board GPIO
#define LED_GREEN       GPIO_NUM(PORTD, 12)
#define LED_AMBER       GPIO_NUM(PORTD, 13)
#define LED_RED         GPIO_NUM(PORTD, 14)
#define LED_BLUE        GPIO_NUM(PORTD, 15)
#define PUSH_BUTTON     GPIO_NUM(PORTA, 0) // 0 = open, 1 = pressed

// all step bits must be on the same port
#define STEP_X          GPIO_NUM(PORTB, 0) // GS0
#define STEP_Y          GPIO_NUM(PORTB, 1) // GS0
#define STEP_Z          GPIO_NUM(PORTB, 2) // GS0
#define STEP_A          GPIO_NUM(PORTB, 4) // GS1
#define STEP_B          GPIO_NUM(PORTB, 5) // GS1
#define STEP_C          GPIO_NUM(PORTB, 7) // GS1
#define GS2_STEP_X      GPIO_NUM(PORTB, 8) // GS2
#define GS2_STEP_Y      GPIO_NUM(PORTB, 11) // GS2
#define GS2_STEP_Z      GPIO_NUM(PORTB, 12) // GS2

// all direction bits must be on the same port
#define DIRN_X          GPIO_NUM(PORTD, 0) // GS0
#define DIRN_Y          GPIO_NUM(PORTD, 1) // GS0
#define DIRN_Z          GPIO_NUM(PORTD, 2) // GS0
#define DIRN_A          GPIO_NUM(PORTD, 3) // GS1
#define DIRN_B          GPIO_NUM(PORTD, 4) // GS1
#define DIRN_C          GPIO_NUM(PORTD, 7) // GS1
#define GS2_DIRN_X      GPIO_NUM(PORTD, 8) // GS2
#define GS2_DIRN_Y      GPIO_NUM(PORTD, 9) // GS2
#define GS2_DIRN_Z      GPIO_NUM(PORTD, 10) // GS2

// grbl shield enables
#define GS0_ENABLE      GPIO_NUM(PORTB, 13) // GS0
#define GS1_ENABLE      GPIO_NUM(PORTB, 14) // GS1
#define GS2_ENABLE      GPIO_NUM(PORTB, 15) // GS2

// limit switches
#define LIMIT_X         GPIO_NUM(PORTC, 1)
#define LIMIT_Y         GPIO_NUM(PORTC, 2)
#define LIMIT_Z         GPIO_NUM(PORTC, 5)

// serial port
#define UART_TX         GPIO_NUM(PORTA, 2)
#define UART_RX         GPIO_NUM(PORTA, 3)

//-----------------------------------------------------------------------------
// generic api functions

static inline void gpio_clr(int n)
{
    GPIO_BASE(n)->BSRRH = GPIO_BIT(n);
}

static inline void gpio_set(int n)
{
    GPIO_BASE(n)->BSRRL = GPIO_BIT(n);
}

static inline void gpio_toggle(int n)
{
    GPIO_BASE(n)->ODR ^= GPIO_BIT(n);
}

static inline int gpio_rd(int n)
{
    return (GPIO_BASE(n)->IDR >> GPIO_PIN(n)) & 1;
}

static inline int gpio_rd_inv(int n)
{
    return (~(GPIO_BASE(n)->IDR) >> GPIO_PIN(n)) & 1;
}

//-----------------------------------------------------------------------------
// The input gpios are spread out across several ports. We read and pack them into a
// single uint32_t and debounce them together.

static inline uint32_t debounce_input(void)
{
    // pack the gpio inputs to be debounced into the uint32_t debounce state
    return ((gpio_rd(LIMIT_X) << 0) |
            (gpio_rd(LIMIT_Y) << 1) |
            (gpio_rd(LIMIT_Z) << 2) |
            (gpio_rd(PUSH_BUTTON) << 3));
}

//-----------------------------------------------------------------------------

#endif // GPIO_PINS_H

//-----------------------------------------------------------------------------
