#include <stdlib.h>
#include "nmea.h"
#include "common.h"
#include "xtimer.h"
#include "gps.h"
#include "ringbuffer.h"
#include "math.h"
#include "fusion_math.h"
#include "gps_backend.h"
#include "common_nmea.h"

#define DIGIT_TO_VAL(x) (x - '0')
#define hexdigit(x) ((x)>9?'A'+((x) - 10):'0'+(x))

static struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint32_t itow;
    int32_t fields[8];
} phd;

extern ringbuffer_t gps_buffer;
static uint8_t parity;
static bool is_checksum_term;
static char term[15];
static uint8_t sentence_type;
static uint8_t term_number;
static uint8_t term_offset;
static uint16_t sentence_length = 0;
static bool gps_data_good;
static bool sentence_done;

static int32_t new_time;
static int32_t new_date;
static int32_t new_latitude;
static int32_t new_longitude;
static int32_t new_altitude;
static int32_t new_speed;
static int32_t new_course;
static float new_gps_yaw;
static uint16_t new_hdop;
static uint8_t new_satellite_count;
static uint8_t new_quality_indicator;

static uint32_t last_RMC_ms;
static uint32_t last_GGA_ms;
static uint32_t last_VTG_ms;
static uint32_t last_HDT_THS_ms;
static uint32_t last_PHD_12_ms;
static uint32_t last_PHD_26_ms;
static uint32_t last_fix_ms;

extern gps_status_t _gps_state;
extern int16_t _rate_ms;
extern int8_t _type;

static bool have_new_message(void)
{
    if (last_RMC_ms == 0 ||
    last_GGA_ms == 0) {
        return false;
    }
    uint32_t now = xtimer_now().ticks32 / 1000;
    if (now - last_RMC_ms > 150 ||
    now - last_GGA_ms > 150) {
        return false;
    }
    if (last_VTG_ms != 0 &&
    now - last_VTG_ms > 150) {
        return false;
    }
    if (last_PHD_12_ms != 0 &&
        now - last_PHD_12_ms > 150 &&
        now - last_PHD_12_ms < 1000) {
        return false;
    }
    if (last_PHD_26_ms !=0 &&
        now - last_PHD_26_ms > 150 &&
        now - last_PHD_26_ms < 1000) {
        return false;
    }
    if (last_VTG_ms != 0) {
        last_VTG_ms = 1;
    }
    if (now - last_HDT_THS_ms > 300) {
        _gps_state.have_gps_yaw = false;
    }
    const int32_t dt_ms = now - last_fix_ms;
    if (labs(dt_ms - _rate_ms) > 50 &&
        _type == GPS_TYPE_ALLSTAR) {
        nmea_printf("$PHD,06,42,UUUUTTTT,BB,0,%u,55,0,%u,0,0,0",
                    1000U / _rate_ms,
                    _rate_ms);
    }
    last_fix_ms = now;
    last_GGA_ms = 1;
    last_RMC_ms = 1;
    return true;
}

static bool term_complete(void)
{
    if (is_checksum_term) {
        sentence_done = true;
        uint8_t nibble_high = 0;
        uint8_t nibble_low = 0;
        if (!hex_to_uint8(term[0], &nibble_high) || !hex_to_uint8(term[1], &nibble_low)) {
            return false;
        }
        const uint8_t checksum = (nibble_high) << 4u | nibble_low;
        if (checksum == parity) {
            if (gps_data_good) {
                uint32_t now = xtimer_now().ticks32 / 1000;
                switch (sentence_type) {
                    case GPS_SENTENCE_RMC:
                        last_RMC_ms = now;
                        _gps_state.location.lat = new_latitude;
                        _gps_state.location.lng = new_longitude;
                        _gps_state.ground_speed = new_speed * 0.01f;
                        _gps_state.ground_course = wrap_360(new_course * 0.01f);
                        make_gps_time(new_date, new_time * 10);
                        set_uart_timestamp(sentence_length);
                        _gps_state.last_gps_time_ms = now;
                        if (last_PHD_12_ms == 0 ||
                        now - last_PHD_12_ms > 1000) {
                            fill_3d_velocity();
                        }
                        break;
                    case GPS_SENTENCE_GGA:
                        last_GGA_ms = now;
                        _gps_state.location.alt = new_altitude;
                        _gps_state.location.lat = new_latitude;
                        _gps_state.location.lng = new_longitude;
                        _gps_state.num_sats = new_satellite_count;
                        _gps_state.hdop = new_hdop;
                        switch (new_quality_indicator) {
                            case 0:
                                _gps_state.status = NO_FIX;
                                break;
                            case 1:
                                _gps_state.status = GPS_OK_FIX_3D;
                                break;
                            case 2:
                                _gps_state.status = GPS_OK_FIX_3D_DGPS;
                                break;
                            case 3:
                                _gps_state.status = GPS_OK_FIX_3D;
                                break;
                            case 4:
                                _gps_state.status = GPS_OK_FIX_3D_RTK_FIXED;
                                break;
                            case 5:
                                _gps_state.status = GPS_OK_FIX_3D_RTK_FLOAT;
                                break;
                            case 6:
                                _gps_state.status = NO_FIX;
                                break;
                            default:
                                _gps_state.status = GPS_OK_FIX_3D;
                                break;
                        }
                        break;
                    case GPS_SENTENCE_VTG:
                        last_VTG_ms = now;
                        _gps_state.ground_speed = new_speed * 0.01f;
                        _gps_state.ground_course = wrap_360(new_course * 0.01f);
                        if (last_PHD_12_ms == 0 ||
                        now - last_PHD_12_ms > 1000) {
                            fill_3d_velocity();
                        }
                        break;
                    case GPS_SENTENCE_HDT:
                    case GPS_SENTENCE_THS:
                        last_HDT_THS_ms = now;
                        _gps_state.gps_yaw = wrap_360(new_gps_yaw * 0.01f);
                        _gps_state.have_gps_yaw = true;
                        _gps_state.gps_yaw_time_ms = xtimer_now().ticks32 / 1000;
                        _gps_state.gps_yaw_configured = true;
                        break;
                    case GPS_SENTENCE_PHD:
                        if (phd.msg_id == 12) {
                            _gps_state.velocity.x = phd.fields[0] * 0.01;
                            _gps_state.velocity.y = phd.fields[1] * 0.01;
                            _gps_state.velocity.z = phd.fields[2] * 0.01;
                            _gps_state.have_vertical_velocity = true;
                            last_PHD_12_ms = now;
                        } else if (phd.msg_id == 26) {
                            _gps_state.horizontal_accuracy = MAX(phd.fields[0], phd.fields[1])
                                * 0.01;
                            _gps_state.have_horizontal_accuracy = true;
                            _gps_state.vertical_accuracy = phd.fields[2] * 0.01;
                            _gps_state.have_vertical_accuracy = true;
                            _gps_state.speed_accuracy = MAX(phd.fields[3], phd.fields[4]) *
                                0.01;
                            _gps_state.have_speed_accuracy = true;
                            last_PHD_26_ms = now;
                        }
                }
            } else {
                switch (sentence_type) {
                    case GPS_SENTENCE_RMC:
                    case GPS_SENTENCE_GGA:
                        _gps_state.status = NO_FIX;
                        break;
                    case GPS_SENTENCE_THS:
                        _gps_state.have_gps_yaw = false;
                        break;
                }
            }
            return have_new_message();
        }
        return false;
    }
    return true;
}

static bool decode(char c)
{
    bool valid_sentence = false;
    sentence_length++;
    switch (c) {
        case ',':
            parity ^= c;
            __attribute__((fallthrough));
        case '\r':
        case '\n':
        case '*':
            if (sentence_done) {
                return false;
            }
            if (term_offset < sizeof(term)) {
                term[term_offset] = 0;
                valid_sentence = term_complete();
            }
            ++term_number;
            term_offset = 0;
            is_checksum_term = c == '*';
            return valid_sentence;
        case '$':
            term_number = term_offset = 0;
            parity = 0;
            sentence_type = GPS_SENTENCE_OTHER;
            is_checksum_term = false;
            gps_data_good = false;
            sentence_length = 1;
            sentence_done = false;
            return valid_sentence;
    }
    if (term_offset < sizeof(term) - 1) {
        term[term_offset++] = c;
    }
    if (!is_checksum_term) {
        parity ^= c;
    }
    return true;
}

bool gps_read(void)
{
    int16_t c;
    bool parsed = false;
    while((c = ringbuffer_get_one(&gps_buffer)) != -1) {
        if (decode(c)) {
            parsed = true;
        }
    }
    return parsed;
}
