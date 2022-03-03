#include <stdlib.h>
#include <string.h>
#include <ctype.h>
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

static int32_t parse_decimal_100(const char *p)
{
    char *endptr = NULL;
    long ret = 100 * strtol(p, &endptr, 10);
    int sign = ret < 0 ? -1 : 1;
    if (ret >= (long)INT32_MAX) {
        return INT32_MAX;
    }
    if (ret <= (long)INT32_MIN) {
        return INT32_MIN;
    }
    if (endptr == NULL || *endptr != '.') {
        return ret;
    }
    if (isdigit((int)endptr[1])) {
        ret += sign * 10 * DIGIT_TO_VAL(endptr[1]);
        if (isdigit((int)endptr[2])) {
            ret += sign * DIGIT_TO_VAL(endptr[2]);
            if (isdigit((int)endptr[3])) {
                ret += sign * (DIGIT_TO_VAL(endptr[3]) >= 5);
            }
        }
    }
    return ret;
}

static uint32_t parse_degrees(void)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;
    for (p = term; *p && isdigit((int)*p); p++) {
        ;
    }
    q = term;
    while ((p - q) > 2  && *q) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    while (p > q && *q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (*q && isdigit((int)*q)) {
            frac_min += DIGIT_TO_VAL(*q) * frac_scale;
            q ++;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t)(frac_min * (1.0e7f / 60.0f));
    return ret;
}

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
    if (term_number == 0) {
        if (strcmp(term, "PHD") == 0) {
            sentence_type = GPS_SENTENCE_PHD;
            return false;
        }
        if (term[0] < 'A' || term[0] > 'Z' ||
            term[1] < 'A' || term[1] > 'Z') {
            sentence_type = GPS_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &term[2];
        if (strcmp(term_type, "RMC") == 0) {
            sentence_type = GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {
            sentence_type = GPS_SENTENCE_GGA;
        } else if (strcmp(term_type, "HDT") == 0) {
            sentence_type = GPS_SENTENCE_HDT;
            gps_data_good = true;
        } else if (strcmp(term_type, "THS") == 0) {
            sentence_type = GPS_SENTENCE_THS;
        } else if (strcmp(term_type, "VTG") == 0) {
            sentence_type = GPS_SENTENCE_VTG;
            gps_data_good = true;
        } else {
            sentence_type = GPS_SENTENCE_OTHER;
        }
        return false;
    }
    if (sentence_type != GPS_SENTENCE_OTHER && term[0]) {
        switch (sentence_type + term_number) {
            case GPS_SENTENCE_RMC + 2:
                gps_data_good = term[0] == 'A';
                break;
            case GPS_SENTENCE_GGA + 6:
                gps_data_good = term[0] > '0';
                new_quality_indicator = term[0] - '0';
                break;
            case GPS_SENTENCE_THS + 2:
                gps_data_good = term[0] == 'A';
                break;
            case GPS_SENTENCE_VTG + 9:
                gps_data_good = term[0] != 'N';
                break;
            case GPS_SENTENCE_GGA + 7:
                new_satellite_count = atol(term);
                break;
            case GPS_SENTENCE_GGA + 8:
                new_hdop = (uint16_t)parse_decimal_100(term);
                break;
            case GPS_SENTENCE_RMC + 1:
            case GPS_SENTENCE_GGA + 1:
                new_time = parse_decimal_100(term);
                break;
            case GPS_SENTENCE_RMC + 9:
                new_date = atol(term);
                break;
            case GPS_SENTENCE_RMC + 3:
            case GPS_SENTENCE_GGA + 2:
                new_latitude = parse_degrees();
                break;
            case GPS_SENTENCE_RMC + 4:
            case GPS_SENTENCE_GGA + 3:
                if (term[0] == 'S') {
                    new_latitude = -new_latitude;
                }
                    break;
            case GPS_SENTENCE_RMC + 5:
            case GPS_SENTENCE_GGA + 4:
                new_longitude = parse_degrees();
                break;
            case GPS_SENTENCE_RMC + 6:
            case GPS_SENTENCE_GGA + 5:
                if (term[0] == 'W') {
                    new_longitude = -new_longitude;
                }
                break;
            case GPS_SENTENCE_GGA + 9:
                new_altitude = parse_decimal_100(term);
                break;
            case GPS_SENTENCE_RMC + 7:
            case GPS_SENTENCE_VTG + 5:
                new_speed = (parse_decimal_100(term) * 514) / 1000;
                break;
            case GPS_SENTENCE_HDT + 1:
                new_gps_yaw = parse_decimal_100(term);
                break;
            case GPS_SENTENCE_THS + 1:
                new_gps_yaw = parse_decimal_100(term);
                break;
            case GPS_SENTENCE_RMC + 8:
            case GPS_SENTENCE_VTG + 1:
                new_course = parse_decimal_100(term);
                break;
            case GPS_SENTENCE_PHD + 1:
                phd.msg_class = atol(term);
                break;
            case GPS_SENTENCE_PHD + 2:
                phd.msg_id = atol(term);
                if (phd.msg_class == 1 && (phd.msg_id == 12 || phd.msg_id == 26)) {
                    gps_data_good = true;
                }
                break;
            case GPS_SENTENCE_PHD + 5:
                phd.itow = strtoul(term, NULL, 10);
                break;
            case GPS_SENTENCE_PHD + 6 ... GPS_SENTENCE_PHD + 11:
                phd.fields[term_number - 6] = atol(term);
                break;
        }
    }
    return false;
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
