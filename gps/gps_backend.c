#include <time.h>
#include <math.h>
#include "gps.h"
#include "gps_backend.h"
#include "definitions.h"
#include "uart_device.h"
#include "fusion_math.h"

uint32_t _last_itow;
uint64_t _pseudo_itow;
uint32_t _last_ms;
uint32_t _last_rate_ms;
uint16_t _rate_counter;

/* static time_t mktime(const struct tm*t) */
/* { */
/*     time_t epoch = 0; */
/*     int n; */
/*     int mon[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}, y, m, i; */
/*     const unsigned MINUTE = 60; */
/*     const unsigned HOUR = 60 * MINUTE; */
/*     const unsigned DAY = 24 * HOUR; */
/*     const unsigned YEAR = 365 * DAY; */
/*     if (t->tm_year < 70) { */
/*         return (time_t)-1; */
/*     } */
/*     n = t->tm_year + 1900 - 1; */
/*     epoch = (t->tm_year - 70) * YEAR + */
/*         ((n / 4 - n / 100 + n / 400) - (1969 / 4 - 1969 / 100 + 1969 / 400)) * DAY; */
/*     y = t->tm_year + 1900; */
/*     m = 0; */
/*     for (i = 0; i < t->tm_mon; i++) { */
/*         epoch += mon[m] * DAY; */
/*         if (m == 1 && y % 4 == 0 && (y % 100 !=0 || y % 400 == 0)) { */
/*             epoch += DAY; */
/*         } */
/*         if (++m > 11) { */
/*             m = 0; */
/*             y++; */
/*         } */
/*     } */
/*     epoch  += (t->tm_mday - 1) * DAY; */
/*     epoch += t->tm_hour * HOUR + t->tm_min * MINUTE + t->tm_sec; */

/*     return epoch; */
/* } */
extern gps_status_t _gps_state;

void set_uart_timestamp(uint16_t nbytes)
{
    _gps_state.uart_timestamp_ms = receive_time_constraint_us(nbytes) / 1000U;
}

void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds)
{
    struct tm tm = {};
    tm.tm_year = 100U + bcd_date % 100U;
    tm.tm_mon = ((bcd_date / 100U) % 100U) - 1;
    tm.tm_mday = bcd_date / 10000U;
    uint32_t v = bcd_milliseconds;
    uint16_t msec = v % 1000U; v /= 1000U;
    tm.tm_sec = v % 100U; v /= 100U;
    tm.tm_min = v % 100U; v /= 100U;
    tm.tm_hour = v % 100U;
    time_t unix_time = mktime(&tm);
    const uint32_t unix_to_GPS_secs = 315964800UL;
    const uint16_t leap_seconds_unix = GPS_LEAPSECONDS_MILLIS / 1000U;
    uint32_t ret = unix_time + leap_seconds_unix - unix_to_GPS_secs;
    _gps_state.time_week = ret / AP_SEC_PER_WEEK;
    _gps_state.time_week_ms = (ret % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC;
    _gps_state.time_week_ms += msec;
}

void fill_3d_velocity(void)
{
    float gps_heading = radians(_gps_state.ground_course);
    _gps_state.velocity.x = _gps_state.ground_speed * cosf(gps_heading);
    _gps_state.velocity.y = _gps_state.ground_speed * sinf(gps_heading);
    _gps_state.velocity.z = 0;
    _gps_state.have_vertical_velocity = false;
}
