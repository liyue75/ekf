#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "common_nmea.h"
#include "gps.h"
#include "periph/uart.h"

char *nmea_vaprintf(const char *fmt, va_list ap)
{
    va_list ap_copy;
    va_copy(ap_copy, ap);
    int len = vsnprintf(NULL, 0, fmt, ap_copy);
    va_end(ap_copy);
    if (len <= 0) {
        return NULL;
    }
    char *s = (char *)malloc(len + 6);
    if (s == NULL) {
        return NULL;
    }
    if (vsnprintf(s, len + 5, fmt, ap) < len) {
        free(s);
        return NULL;
    }
    uint8_t cs = 0;
    const uint8_t *b = (const uint8_t *)s + 1;
    while (*b) {
        cs ^= *b++;
    }
    snprintf(s +len, 6, "*%02X\r\n", (unsigned)cs);
    return s;
}

bool nmea_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s == NULL) {
        return false;
    }
    size_t len = strlen(s);
    uart_write(GPS_UART_DEV, (uint8_t *)s, len);
    free(s);
    return true;
}
