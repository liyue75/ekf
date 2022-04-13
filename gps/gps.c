#include <stddef.h>
#include <string.h>
#include "gps.h"
#include "periph/uart.h"
#include "thread.h"
#include "uart_device.h"
#include "xtimer.h"
#include "ringbuffer.h"
#include "mutex.h"
#include "nmea.h"
#include "sky_traq.h"
#include "board_led.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define ENABLE_GPS_LOG 0

#ifndef GPS_MAX_RATE_MS
#define GPS_MAX_RATE_MS 200
#endif
#define GPS_BAUD_TIME_MS 1200
#define GPS_TIMEOUT_MS 4000u

static char gps_rx_buffer[GPS_UART_RX_BUFFER_SIZE];
ringbuffer_t gps_buffer;
uint64_t _receive_timestamp[2];
uint8_t _receive_timestamp_idx;
static bool force_disable_gps = false;

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
__attribute__((unused))static int16_t delay_ms = 0; //set to zero to use the default delay for the detected GPS type;

static gps_timing_t _gps_timing;
gps_status_t _gps_state;
static mutex_t rsem;
static bool drivers = false;
//static detect_state_t detect_state;

bool gps_get_lag(float *lag_sec)
{
    /* if (delay_ms > 0) { */
    /*     lag_sec = 0.001f * delay_ms; */
    /* } */
    *lag_sec = 0.2f;
    return true;
}

static void receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = xtimer_now64().ticks64;
    _receive_timestamp_idx ^= 1;
}

static void uart_cb(__attribute__((unused))void *arg, uint8_t data)
{
    /* if (ringbuffer_full(&gps_buffer)) { */
    /*     led_on(LED_2); */
    /* } */
    ringbuffer_add_one(&gps_buffer, data);
    receive_timestamp_update();
}

uint64_t receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    uint32_t transport_time_us = (1000000UL * 10 / baudrate) * (nbytes + gps_buffer.avail);
    //MY_LOG("nbytes: %d trans: %ld", nbytes, transport_time_us / 1000);
    last_receive_us -= transport_time_us;
    return last_receive_us;
}

static void set_skytra_gps_mode(void)
{
    ringbuffer_remove(&gps_buffer, sizeof(gps_rx_buffer));
    if (skytra_cfg_rate(5)) {
        MY_LOG("Config gps rate 5hz\n");
    }
    xtimer_usleep(100000);
    ringbuffer_remove(&gps_buffer, sizeof(gps_rx_buffer));
    if (skytra_cfg_msg_type()) {
        MY_LOG("Config gps msg type\n");
    }
}

void gps_init(void)
{
    ringbuffer_init(&gps_buffer, gps_rx_buffer, sizeof(gps_rx_buffer));
    //DEBUG("gps ring buffer size =  %d\n", sizeof(gps_rx_buffer));
    uart_init(GPS_UART_DEV, GPS_UART_BAUD, uart_cb, NULL);
    baudrate = GPS_UART_BAUD;
    if (_rate_ms <= 0 || _rate_ms > GPS_MAX_RATE_MS) {
        _rate_ms = GPS_MAX_RATE_MS;
    }
    mutex_init(&rsem);
    set_skytra_gps_mode();
}

uint32_t gps_last_message_time_ms(void)
{
    return _gps_timing.last_message_time_ms;
}

uint32_t gps_last_message_delta_time_ms(void)
{
    return _gps_timing.delta_time_ms;
}

gps_status_enum_t gps_status(void)
{
    return _gps_state.status;
}

vector3f_t gps_velocity(void)
{
    return _gps_state.velocity;
}

bool gps_have_vertical_velocity(void)
{
    return _gps_state.have_vertical_velocity;
}

bool gps_speed_accuracy(float *sacc)
{
    if (_gps_state.have_speed_accuracy) {
        *sacc = _gps_state.speed_accuracy;
        return true;
    }
    return false;
}

bool gps_horizontal_accuracy(float *hacc)
{
    if (_gps_state.have_horizontal_accuracy) {
        *hacc = _gps_state.horizontal_accuracy;
        return true;
    }
    return false;
}

bool gps_vertical_accuracy(float *vacc)
{
    if (_gps_state.have_vertical_accuracy) {
        *vacc = _gps_state.vertical_accuracy;
        return true;
    }
    return false;
}

uint8_t gps_num_sats(void)
{
    return _gps_state.num_sats;
}

uint16_t gps_get_hdop(void)
{
    return _gps_state.hdop;
}

location_t *gps_location(void)
{
    return &_gps_state.location;
}

float gps_ground_speed(void)
{
    return _gps_state.ground_speed;
}

float gps_ground_course(void)
{
    return _gps_state.ground_course;
}

uint32_t gps_last_fix_time_ms(void)
{
    return _gps_timing.last_fix_time_ms;
}

static void detect_instance(void)
{
    //detect_state_t *dstate = &detect_state;
    const uint32_t now = xtimer_now().ticks32 / 1000;
    _gps_state.hdop = GPS_UNKNOWN_DOP;
    _gps_state.vdop = GPS_UNKNOWN_DOP;
    _gps_state.status = NO_FIX;
    drivers = true;
    _gps_timing.last_message_time_ms = now;
    _gps_timing.delta_time_ms = GPS_TIMEOUT_MS;
    ringbuffer_remove(&gps_buffer, sizeof(gps_rx_buffer));
    //led_off(LED_2);
}

static uint32_t last_write_gps_ms;
__attribute__((unused))static void write_gps(void)
{
    uint32_t now = xtimer_now().ticks32 / 1000;
    if (now - last_write_gps_ms > 1000) {
        last_write_gps_ms = now;
        location_t *loc = gps_location();
        MY_LOG("status: %d loc: %ld %ld %ld\nsats: %d hdop: %d\ndelta: %ld\nspd: %f course: %f\n",
               gps_status(),
               loc->lng, loc->lat, loc->alt,
               gps_num_sats(), gps_get_hdop(), gps_last_message_delta_time_ms(),
               gps_ground_speed(), gps_ground_course());
    }
}

static void update_instance(void)
{
    if (!drivers) {
        detect_instance();
        return;
    }
    bool result= gps_read();
    uint32_t tnow = xtimer_now().ticks32 / 1000;
    bool data_should_be_logged = false;
    if (!result) {
        if (tnow - _gps_timing.last_message_time_ms > GPS_TIMEOUT_MS) {
            //MY_LOG("GPS timeout\n");
            memset((void *)&_gps_state, 0, sizeof(_gps_state));
            _gps_state.hdop = GPS_UNKNOWN_DOP;
            _gps_state.vdop = GPS_UNKNOWN_DOP;
            _gps_timing.delta_time_ms = GPS_TIMEOUT_MS;
            _gps_timing.last_message_time_ms = tnow;
            _gps_state.status = NO_FIX;
        }
    } else {
        if (_gps_state.uart_timestamp_ms != 0) {
            //MY_LOG("tnow: %ld ups.uart timestam: %ld\n", tnow, _gps_state.uart_timestamp_ms);
            tnow = _gps_state.uart_timestamp_ms;
            _gps_state.uart_timestamp_ms = 0;
        }
        _gps_timing.delta_time_ms = tnow - _gps_timing.last_message_time_ms;
        _gps_timing.last_message_time_ms = tnow;
        if (_gps_state.status >= GPS_OK_FIX_2D && !force_disable_gps) {
            _gps_timing.last_fix_time_ms = tnow;
        }
        data_should_be_logged = true;
    }
    if (data_should_be_logged) {
        const uint16_t gps_max_delta_ms = 245;
        gps_timing_t *t = &_gps_timing;
        if (t->delta_time_ms > gps_max_delta_ms) {
            t->delayed_count++;
        } else {
            t->delayed_count = 0;
        }
        if (t->delta_time_ms < 2000) {
            if (t->average_delta_ms <= 0) {
                t->average_delta_ms = 200;//t->delta_time_ms;
            } else {
                t->average_delta_ms = 0.98f * t->average_delta_ms + 0.02f * t->delta_time_ms;
            }
        }
    }
    if (data_should_be_logged && ENABLE_GPS_LOG) {
        write_gps();
    }
}

void gps_update(void)
{
    mutex_lock(&rsem);
    update_instance();
    mutex_unlock(&rsem);
}
