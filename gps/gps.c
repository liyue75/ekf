#include <stddef.h>
#include "gps.h"
#include "periph/uart.h"
#include "thread.h"
#include "xtimer.h"
#include "ringbuffer.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static char gps_rx_buffer[GPS_UART_RX_BUFFER_SIZE];
ringbuffer_t gps_buffer;
uint64_t _receive_timestamp[2];
uint8_t _receive_timestamp_idx;

#ifndef GPS_MAX_RATE_MS
#define GPS_MAX_RATE_MS 200
#endif
#define GPS_BAUD_TIME_MS 1200
#define GPS_TIMEOUT_MS 4000u

const uint32_t baudrates[] = {9600, 115200, 4800, 19200, 38400, 57600, 230400, 460800};
static uint32_t baudrate;
const char initialisation_blob[] = "";
int8_t _type = GPS_TYPE_NMEA;
__attribute__((unused))static int8_t navfilter = GPS_ENGINE_AUTOMOTIVE;
__attribute__((unused))static int8_t auto_switch = 0;
__attribute__((unused))static int8_t min_dgps = 100;
__attribute__((unused))static int8_t sbas_mode = 2;
__attribute__((unused))static int8_t min_elevation = -100;
__attribute__((unused))static int8_t inject_to = 127;
__attribute__((unused))static int16_t sbp_logmask = -256;
__attribute__((unused))static int8_t raw_data = 0;
__attribute__((unused))static int8_t gnss_mode = 0;
__attribute__((unused))static int8_t save_config = 0;
__attribute__((unused))static int8_t auto_config = 1;

int16_t _rate_ms = 200; // 100:10hz 125:8hz 200:5hz. Raising the rate above 5hz usually provides little benefit
                       // and for some GPS(eg Ublox M9N) can severely impact performance
__attribute__((unused))static vector3f_t antenna_offset = {0, 0, 0};
int16_t _delay_ms = 0; //set to zero to use the default delay for the detected GPS type;

gps_status_t _gps_state;

void gps_get_lag(float *lag_sec)
{
    *lag_sec = 0.07;
}

static void receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = xtimer_now64().ticks64;
    _receive_timestamp_idx ^= 1;
}

static void uart_cb(__attribute__((unused))void *arg, uint8_t data)
{
    ringbuffer_add_one(&gps_buffer, data);
    receive_timestamp_update();
}

uint64_t receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    uint32_t transport_time_us = (1000000UL * 10 / baudrate) * (nbytes + gps_buffer.avail);
    last_receive_us -= transport_time_us;
    return last_receive_us;
}

void gps_init(void)
{
    ringbuffer_init(&gps_buffer, gps_rx_buffer, sizeof(gps_rx_buffer));
    DEBUG("gps ring buffer size =  %d\n", sizeof(gps_rx_buffer));
    uart_init(GPS_UART_DEV, GPS_UART_BAUD, uart_cb, NULL);
    baudrate = GPS_UART_BAUD;
    if (_rate_ms <= 0 || _rate_ms > GPS_MAX_RATE_MS) {
        _rate_ms = GPS_MAX_RATE_MS;
    }
}
