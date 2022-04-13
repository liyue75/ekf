#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include "location.h"
#include "vector3f.h"

#define GPS_LEAPSECONDS_MILLIS 18000ULL
#define GPS_UART_DEV UART_DEV(1)
#define GPS_UART_BAUD 115200
#define GPS_UART_RX_BUFFER_SIZE 512
#define GPS_UNKNOWN_DOP UINT16_MAX

typedef enum {
GPS_TYPE_NONE = 0,
GPS_TYPE_AUTO = 1,
GPS_TYPE_UBLOX = 2,
GPS_TYPE_NMEA = 5,
GPS_TYPE_ALLSTAR = 20
} gps_type_t;

typedef enum {
NO_GPS,
NO_FIX,
GPS_OK_FIX_2D,
GPS_OK_FIX_3D,
GPS_OK_FIX_3D_DGPS,
GPS_OK_FIX_3D_RTK_FLOAT,
GPS_OK_FIX_3D_RTK_FIXED
} gps_status_enum_t;

typedef enum {
GPS_ENGINE_NONE = -1,
GPS_ENGINE_PORTABLE = 0,
GPS_ENGINE_STATIONARY = 2,
GPS_ENGINE_PEDESTRIAN = 3,
GPS_ENGINE_AUTOMOTIVE = 4,
GPS_ENGINE_SEA = 5,
GPS_ENGINE_AIRBORNE_1G = 6,
GPS_ENGINE_AIRBORNE_2G,
GPS_ENGINE_AIRBORNE_4G
} gps_engine_setting_t;

typedef enum {
GPS_ROLE_NORMAL,
GPS_ROLE_MB_BASE,
GPS_ROLE_MB_ROVER
} gps_role_t;

typedef struct {
    gps_status_enum_t status;
    uint32_t time_week_ms;
    uint16_t time_week;
    location_t location;
    float ground_speed;
    float ground_course;
    float gps_yaw;
    uint32_t gps_yaw_time_ms;
    bool gps_yaw_configured;
    uint16_t hdop;
    uint16_t vdop;
    uint8_t num_sats;
    vector3f_t velocity;
    float speed_accuracy;
    float horizontal_accuracy;
    float vertical_accuracy;
    float gps_yaw_accuracy;
    bool have_vertical_velocity;
    bool have_speed_accuracy;
    float have_horizontal_accuracy;
    bool have_vertical_accuracy;
    bool have_gps_yaw;
    bool have_gps_yaw_accuracy;
    uint32_t last_gps_time_ms;
    uint32_t uart_timestamp_ms;
    uint32_t lagged_sample_count;

    float rel_pos_heading;
    float rel_pos_length;
    float rel_pos_d;
    float acc_heading;
    uint32_t rel_pos_heading_ts;
} gps_status_t;

typedef struct {
    uint32_t last_fix_time_ms;
    uint32_t last_message_time_ms;
    uint16_t delta_time_ms;
    uint8_t delayed_count;
    float average_delta_ms;
} gps_timing_t;

typedef struct {
    uint8_t step;
    uint8_t ck;
} nmea_detect_state_t;

typedef struct {
    uint32_t last_baud_change_ms;
    uint8_t current_baud;
    bool auot_detected_baud;
    nmea_detect_state_t nmea_detect_state;
} detect_state_t;

bool gps_get_lag(float *lag_sec);
void gps_init(void);
bool gps_read(void);
uint64_t receive_time_constraint_us(uint16_t nbytes);
uint32_t gps_last_message_time_ms(void);
uint32_t gps_last_message_delta_time_ms(void);
gps_status_enum_t gps_status(void);
vector3f_t gps_velocity(void);
bool gps_have_vertical_velocity(void);
bool gps_speed_accuracy(float *sacc);
bool gps_horizontal_accuracy(float *hacc);
bool gps_vertical_accuracy(float *vacc);
uint8_t gps_num_sats(void);
uint16_t gps_get_hdop(void);
location_t *gps_location(void);
float gps_ground_speed(void);
float gps_ground_course(void);
uint32_t gps_last_fix_time_ms(void);
void gps_update(void);
#endif // GPS_H_
