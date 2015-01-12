//-----------------------------------------------------------------------------

#ifndef STM32F4
#define STM32F4

//-----------------------------------------------------------------------------

#include "delay.h"
#include "xio.h"
#include "rtc.h"

//-----------------------------------------------------------------------------

void cli(void);
void sei(void);

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
