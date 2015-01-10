//-----------------------------------------------------------------------------

#ifndef STM32F4
#define STM32F4

//-----------------------------------------------------------------------------
#define buffer_t uint_fast8_t
#define flags_t uint16_t

//-----------------------------------------------------------------------------
#define NUL 0

//-----------------------------------------------------------------------------
typedef struct rtClock {
    volatile uint32_t clock_ticks; // RTC tick counter
    uint16_t magic_end;
} rtClock_t;

rtClock_t rtc;

void rtc_init(void);

//-----------------------------------------------------------------------------

void cli(void);
void sei(void);

//-----------------------------------------------------------------------------
void xio_init(void);

uint8_t xio_assertions(uint8_t *value);

//-----------------------------------------------------------------------------

void xio_reset_usb_rx_buffers(void);
buffer_t xio_get_usb_rx_free(void);
#define XIO_DEV_USB 0

//-----------------------------------------------------------------------------

int xio_ctrl(const uint8_t dev, const flags_t flags);
int xio_set_baud(const uint8_t dev, const uint8_t baud_rate);
void xio_set_stdin(const uint8_t dev);
void xio_set_stdout(const uint8_t dev);
void xio_set_stderr(const uint8_t dev);
int xio_gets(const uint8_t dev, char *buf, const int size);

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

double square(double square);

//-----------------------------------------------------------------------------

#define PROGMEM
#define PSTR(x) (x)
#define PGM_P const char *
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
