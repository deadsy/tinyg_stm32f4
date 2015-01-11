//-----------------------------------------------------------------------------
/*

XIO Functions

*/
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdio.h>

#include "xio.h"

//-----------------------------------------------------------------------------

void xio_init(void)
{
}

FILE *xio_open(const uint8_t dev, const char *addr, const flags_t flags)
{
    return 0;
}

int xio_ctrl(const uint8_t dev, const flags_t flags)
{
    return 0;
}

int xio_gets(const uint8_t dev, char *buf, const int size)
{
    return 0;
}

int xio_getc(const uint8_t dev)
{
    return 0;
}

int xio_putc(const uint8_t dev, const char c)
{
    return 0;
}

int xio_set_baud(const uint8_t dev, const uint8_t baud_rate)
{
    return 0;
}

void xio_set_stdin(const uint8_t dev)
{
}

void xio_set_stdout(const uint8_t dev)
{
}

void xio_set_stderr(const uint8_t dev)
{
}

uint8_t xio_assertions(uint8_t *value)
{
    return 0;
}

//-----------------------------------------------------------------------------

void xio_enable_rs485_rx(void)
{
}

void xio_reset_usb_rx_buffers(void)
{
}

buffer_t xio_get_usb_rx_free(void)
{
    return 0;
}

//-----------------------------------------------------------------------------
