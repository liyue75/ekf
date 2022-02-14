#include <math.h>
#include "ekf3_core.h"
#include "location.h"
#include "inertial_sensor.h"
#include "fusion_math.h"
#include "gps.h"
#include "ekf_buffer.h"
#include "ahrs.h"
#include "board_led.h"

#define ENABLE_DEBUG 1
#include "debug.h"

source_set_t _source_set = {
XY_GPS,
XY_GPS,
Z_BARO,
Z_GPS,
YAW_COMPASS,
};

static uint32_t first_init_time_ms;
static uint32_t last_init_fail_report_ms;
float _dt_ekf_avg;
static uint8_t imu_buffer_length;
static uint8_t obs_buffer_length;

extern bool fly_forward;
extern vehicle_class_t vehicle_class;
extern uint16_t mag_delay_ms;
extern uint8_t sensor_interval_min_ms;

__attribute__((unused))static float gps_noise_scaler;
__attribute__((unused))static Matrix24 p;
ekf_imu_buffer_t _stored_imu;
ekf_ring_buffer_t _stored_gps;
ekf_ring_buffer_t _stored_mag;
ekf_imu_buffer_t _stored_output;

vector3f_t *_accel_pos_offset;

float _dt_imu_avg;
uint32_t _imu_sample_time_ms;
bool _on_ground = true;
bool _on_ground_not_moving;
bool _prev_on_ground;
vector3f_t _gyro_prev;
vector3f_t _accel_prev;
uint32_t _last_move_check_log_time_ms;
float _gyro_diff;
float _accel_diff;

vector3f_t _del_ang_corrected;
vector3f_t _del_vel_corrected;

state_elements_t _state_struct;
imu_elements_t _imu_data_delayed;
imu_elements_t _imu_data_new;
imu_elements_t _imu_data_down_sampled_new;
quaternionf_t _imu_quat_down_sample_new;
uint32_t _frames_since_predict;
bool _start_predict_enabled;
bool _run_updates;
ekf_timing_t _timing;

inactive_bias_t _inactive_bias;

void init_ekf3_core(void)
{
    init_location();
    first_init_time_ms = 0;
    last_init_fail_report_ms = 0;
}

bool ekf3_setup_core(void)
{
    if (get_ins_loop_rate_hz() > 0) {
        _dt_ekf_avg = 1.0f / get_ins_loop_rate_hz();
        _dt_ekf_avg = MAX(_dt_ekf_avg, EKF_TARGET_DT);
    }
    uint16_t max_time_delay_ms = MAX(mag_delay_ms, EKF_TARGET_DT_MS);
    float gps_delay_sec = 0;
    gps_get_lag(&gps_delay_sec);
    max_time_delay_ms = MAX(max_time_delay_ms, MIN(gps_delay_sec * 1000, 250));
    imu_buffer_length = (max_time_delay_ms / EKF_TARGET_DT_MS) + 1;
    uint16_t ekf_delay_ms = max_time_delay_ms + (int)(ceilf(max_time_delay_ms * 0.5f));
    DEBUG("ekf_delay ms = %d, imu_buffer_length = %d\n", ekf_delay_ms, imu_buffer_length);
    obs_buffer_length = ekf_delay_ms / sensor_interval_min_ms + 1;
    obs_buffer_length = MIN(obs_buffer_length, imu_buffer_length);
    DEBUG("obs buffer length = %d\n", obs_buffer_length);
    init_ekf_ring_buffer(&_stored_gps, sizeof(gps_elements_t), obs_buffer_length);
    init_ekf_imu_buffer(&_stored_imu, sizeof(imu_elements_t), imu_buffer_length);
    init_ekf_imu_buffer(&_stored_output, sizeof(output_elements_t), imu_buffer_length);
    return true;
}

static bool assume_zero_sideslip(void)
{
    return fly_forward && (vehicle_class != GROUND);
}

bool init_ekf3_core_filter_bootstrap(void)
{
    ekf3_core_update_sensor_selection();
    if (assume_zero_sideslip() /*&& gps_status < GPS_OK_FIX_3D*/) {
        led_on(LED_2);
        return false;
    }
    ekf3_core_read_imu_data();
    //ekf3_core_read_mag_data();
    ekf3_core_read_gps_data();
    return true;
}

source_yaw_t ekf3_get_yaw_source(void)
{
    if ((_source_set.yaw == YAW_COMPASS) /*&& (compass_enabled() == 0)*/) {
        return YAW_NONE;
    }
    return _source_set.yaw;
}

void ekf3_core_correct_delta_angle(vector3f_t *del_ang, float del_ang_dt)
{
    vector3f_t tmp = v3f_uniform_scale(&_inactive_bias.gyro_bias, del_ang_dt / _dt_ekf_avg);
    *del_ang = v3f_sub(del_ang, &tmp);
}

void ekf3_core_correct_delta_velocity(vector3f_t *del_vel, float del_vel_dt)
{
    vector3f_t tmp = v3f_uniform_scale(&_inactive_bias.accel_bias, del_vel_dt / _dt_ekf_avg);
    *del_vel = v3f_sub(del_vel, &tmp);
}
