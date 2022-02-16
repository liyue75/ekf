#ifndef COMMON_NMEA_H_
#define COMMON_NMEA_H_

#include <stdbool.h>
#include <stdarg.h>

char *nmea_vaprintf(const char *fmt, va_list ap);
bool nmea_printf(const char *fmt, ...);

#endif // COMMON_NMEA_H_
