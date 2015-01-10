//-----------------------------------------------------------------------------

#ifndef STM32F4
#define STM32F4

//-----------------------------------------------------------------------------

#include "delay.h"

//-----------------------------------------------------------------------------
#define buffer_t uint_fast8_t
#define flags_t uint16_t

//-----------------------------------------------------------------------------

#define _FDEV_ERR (-1)

//-----------------------------------------------------------------------------
typedef struct rtClock {
    volatile uint32_t clock_ticks; // RTC tick counter
    uint16_t magic_end;
} rtClock_t;

rtClock_t rtc;

void rtc_init(void);

#define RTC_MILLISECONDS 10 // interrupt on every 10 RTC ticks (~10 ms)

//-----------------------------------------------------------------------------

void cli(void);
void sei(void);

//-----------------------------------------------------------------------------

void xio_enable_rs485_rx(void);
void xio_reset_usb_rx_buffers(void);
buffer_t xio_get_usb_rx_free(void);

enum {
    XIO_DEV_USB,
    XIO_DEV_RS485,
    XIO_DEV_PGM,
};

//-----------------------------------------------------------------------------

void xio_init(void);
//void xio_reset_working_flags(xioDev_t *d);
FILE *xio_open(const uint8_t dev, const char *addr, const flags_t flags);
int xio_ctrl(const uint8_t dev, const flags_t flags);
int xio_gets(const uint8_t dev, char *buf, const int size);
int xio_getc(const uint8_t dev);
int xio_putc(const uint8_t dev, const char c);
int xio_set_baud(const uint8_t dev, const uint8_t baud_rate);

// generic functions (private, but at virtual level)
//int xio_ctrl_generic(xioDev_t *d, const flags_t flags);

//void xio_open_generic(uint8_t dev, x_open_t x_open,
//                                   x_ctrl_t x_ctrl,
//                                   x_gets_t x_gets,
//                                   x_getc_t x_getc,
//                                   x_putc_t x_putc,
//                                   x_flow_t x_flow);

//void xio_fc_null(xioDev_t *d);          // NULL flow control callback
//void xio_fc_usart(xioDev_t *d);         // XON/XOFF flow control callback

// std devices
//void xio_init_stdio(void);              // set std devs & do startup prompt
void xio_set_stdin(const uint8_t dev);
void xio_set_stdout(const uint8_t dev);
void xio_set_stderr(const uint8_t dev);

// assertions
uint8_t xio_assertions(uint8_t *value);

//-----------------------------------------------------------------------------

#define XIO_BAUD_115200 0

#define XIO_BLOCK       ((uint16_t)1<<0)                // enable blocking reads
#define XIO_NOBLOCK     ((uint16_t)1<<1)                // disable blocking reads
#define XIO_XOFF        ((uint16_t)1<<2)                // enable XON/OFF flow control
#define XIO_NOXOFF      ((uint16_t)1<<3)                // disable XON/XOFF flow control
#define XIO_ECHO        ((uint16_t)1<<4)                // echo reads from device to stdio
#define XIO_NOECHO      ((uint16_t)1<<5)                // disable echo
#define XIO_CRLF        ((uint16_t)1<<6)                // convert <LF> to <CR><LF> on writes
#define XIO_NOCRLF      ((uint16_t)1<<7)                // do not convert <LF> to <CR><LF> on writes
#define XIO_IGNORECR    ((uint16_t)1<<8)                // ignore <CR> on reads
#define XIO_NOIGNORECR  ((uint16_t)1<<9)                // don't ignore <CR> on reads
#define XIO_IGNORELF    ((uint16_t)1<<10)               // ignore <LF> on reads
#define XIO_NOIGNORELF  ((uint16_t)1<<11)               // don't ignore <LF> on reads
#define XIO_LINEMODE    ((uint16_t)1<<12)               // special <CR><LF> read handling
#define XIO_NOLINEMODE  ((uint16_t)1<<13)               // no special <CR><LF> read handling

//-----------------------------------------------------------------------------

#define NUL (char)0x00  //  ASCII NUL char (0) (not "NULL" which is a pointer)
#define STX (char)0x02  // ^b - STX
#define ETX (char)0x03  // ^c - ETX
#define ENQ (char)0x05  // ^e - ENQuire
#define BEL (char)0x07  // ^g - BEL
#define BS  (char)0x08  // ^h - backspace
#define TAB (char)0x09  // ^i - character
#define LF  (char)0x0A  // ^j - line feed
#define VT  (char)0x0B  // ^k - kill stop
#define CR  (char)0x0D  // ^m - carriage return
#define XON (char)0x11  // ^q - DC1, XON, resume
#define XOFF (char)0x13 // ^s - DC3, XOFF, pause
#define SYN (char)0x16  // ^v - SYN - Used for queue flush
#define CAN (char)0x18  // ^x - Cancel, abort
#define ESC (char)0x1B  // ^[ - ESC(ape)
#define SP  (char)0x20  // ' ' Space character
#define DEL (char)0x7F  // DEL(ete)

//-----------------------------------------------------------------------------

#define PROGMEM
#define PSTR(x) (x)
#define PGM_P const char *
#define PGMFILE (const char *)
#define PGM_FLAGS (XIO_BLOCK | XIO_CRLF | XIO_LINEMODE)

#define printf_P printf
#define fprintf_P fprintf
#define sprintf_P sprintf
#define strcpy_P strcpy
#define strncpy_P strncpy

static inline uint32_t pgm_read_word(const void *ptr) {
    return *(uint32_t*)ptr;
}
static inline uint8_t pgm_read_byte(const void *ptr) {
    return *(uint8_t*)ptr;
}
static inline float pgm_read_float(const void *ptr) {
    return *(float*)ptr;
}

//-----------------------------------------------------------------------------
uint16_t EEPROM_ReadBytes(const uint16_t address, int8_t *buf, const uint16_t size);
uint16_t EEPROM_WriteBytes(const uint16_t address, const int8_t *buf, const uint16_t size);

//-----------------------------------------------------------------------------

#endif // STM32F4
//-----------------------------------------------------------------------------
