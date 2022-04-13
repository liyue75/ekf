#ifndef DAL_H_
#define DAL_H_

#include <stdint.h>
#include "vector3f.h"
#include "ahrs.h"

typedef struct {
    uint64_t time_us;
    uint32_t time_flying_ms;
    uint8_t _end;
} log_rfrh_t;

typedef struct {
    uint8_t frame_types;
    uint8_t core_slow;
    uint8_t _end;
} log_rfrf_t;

typedef struct {
    int32_t lat;
    int32_t lng;
    int32_t alt;
    float EAS2TAS;
    uint32_t available_memory;
    vector3f_t ahrs_trim;
    uint8_t vehicle_class;
    uint8_t ekf_type;
    uint8_t armed:1;
    uint8_t unused:1;
    uint8_t fly_forward:1;
    uint8_t ahrs_airspeed_sensor_enabled:1;
    uint8_t opticalflow_enabled:1;
    uint8_t wheelencoder_enabled:1;
    uint8_t takeoff_expected:1;
    uint8_t touchdown_expected:1;
    uint8_t _end;
} log_rfrn_t;

// Replay data structure - Intertial sensor header
typedef struct {
    uint16_t loop_rate_hz;
    uint8_t primary_gyro;
    uint8_t primary_accel;
    float loop_delta_t;
    uint8_t accel_count;
    uint8_t gyro_count;
    uint8_t _end;
} log_rish_t;

typedef struct {
    vector3f_t delta_velocity;
    vector3f_t delta_angle;
    float delta_velocity_dt;
    float delta_angle_dt;
    uint8_t use_accel:1;
    uint8_t use_gyro:1;
    uint8_t get_delta_velocity_ret:1;
    uint8_t get_delta_angle_ret:1;
    uint8_t instance:1;
    uint8_t _end;
} log_risi_t;

typedef struct {
    uint8_t event;
    uint8_t _end;
} log_rev2_t;

typedef struct {
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint8_t _end;
} log_rso2_t;

typedef struct {
    float airspeed;
    float uncertainty;
    uint8_t _end;
} log_rwa2_t;

typedef log_rev2_t log_rev3_t;
typedef log_rso2_t log_rso3_t;
typedef log_rwa2_t log_rwa3_t;

typedef struct {
    float yawangle;
    float yawangleerr;
    uint32_t timestamp_ms;
    uint8_t type;
    uint8_t _end;
} log_rey3_t;

typedef struct {
    uint8_t primary;
    uint8_t num_instance;
    uint8_t _end;
} log_rbrh_t;

typedef struct {
    uint32_t last_update_ms;
    float altitude;
    bool healthy;
    uint8_t instance;
    uint8_t _end;
} log_rbri_t;

typedef struct {
    uint8_t num_sensors;
    uint8_t primary_sensor;
    uint8_t _end;
} log_rgph_t;

typedef struct {
    vector3f_t antenna_offset;
    float lag_sec;
    uint8_t have_vertical_velocity:1;
    uint8_t horizontal_accuracy_returncode:1;
    uint8_t vertical_accuracy_returncode:1;
    uint8_t get_lag_returncode:1;
    uint8_t speed_accuracy_returncode:1;
    uint8_t gps_yaw_deg_returncode:1;
    uint8_t status;
    uint8_t num_sats;
    uint8_t instance;
    uint8_t _end;
} log_rgpi_t;

typedef struct {
    uint32_t last_message_time_ms;
    vector3f_t velocity;
    float sacc;
    float yaw_deg;
    float yaw_accuracy_deg;
    uint32_t yaw_deg_time_ms;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    float hacc;
    float vacc;
    uint16_t hdop;
    uint8_t instance;
    uint8_t _end;
} log_rgpj_t;

typedef struct {
    float declination;
    bool available;
    uint8_t count;
    bool auto_declination_enabled;
    uint8_t num_enabled;
    bool learn_offsets_enabled;
    bool consistent;
    uint8_t first_usable;
    uint8_t _end;
} log_rmgh_t;

typedef struct {
    uint32_t last_update_usec;
    vector3f_t offsets;
    vector3f_t field;
    bool use_for_yaw;
    bool healthy;
    bool have_scale_factor;
    uint8_t instance;
    uint8_t _end;
} log_rmgi_t;

typedef struct {
    float delang;
    float deltime;
    float timestamp_ms;
    vector3f_t posoffset;
    float radius;
    uint8_t _end;
} log_rwoh_t;

void dal_start_frame(void);
bool dal_get_fly_forward(void);
vehicle_class_t dal_get_vehicle_class(void);
location_t *dal_get_home(void);
bool dal_ekf_low_time_remaining(void);
uint32_t dal_millis(void);
uint64_t dal_micros64(void);

#endif // DAL_H_
