#include <math.h>
#include <string.h>
#include "ekf3_core.h"
#include "location.h"
//#include "inertial_sensor.h"
#include "ins_dal.h"
#include "fusion_math.h"
//#include "gps.h"
#include "gps_dal.h"
#include "ekf_buffer.h"
#include "ahrs.h"
#include "board_led.h"
#include "ekf_source.h"
#include "nav_common.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static uint32_t first_init_time_ms;
static uint32_t last_init_fail_report_ms;
float _dt_ekf_avg;
static uint8_t imu_buffer_length;
static uint8_t obs_buffer_length;

extern bool fly_forward;
extern vehicle_class_t vehicle_class;
extern uint16_t mag_delay_ms;
extern uint8_t _sensor_interval_min_ms;

bool _bad_imu_data;
uint32_t _bad_imu_data_ms;
uint32_t _good_imu_data_ms;
uint32_t _vert_vel_var_clip_counter;

float _gps_noise_scaler;
Matrix24 _P;
Matrix24 _KH;
Matrix24 _KHP;
Matrix24 _nextP;
Vector28 _Kfusion;
ekf_imu_buffer_t _stored_imu;
ekf_ring_buffer_t _stored_gps;
ekf_ring_buffer_t _stored_mag;
ekf_imu_buffer_t _stored_output;

bool _in_flight;

vector3f_t _accel_pos_offset;
vector3f_t _earth_rate_ned;
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

state_var_t _s;

imu_elements_t _imu_data_delayed;
imu_elements_t _imu_data_new;
imu_elements_t _imu_data_down_sampled_new;
quaternionf_t _imu_quat_down_sample_new;
uint32_t _frames_since_predict;
bool _start_predict_enabled;
bool _run_updates;
ekf_timing_t _timing;
fault_status_t _fault_status;
mag_state_t _mag_state;

inactive_bias_t _inactive_bias;

extern location_t _location;
extern source_set_t _source_set;

float _vel_test_ratio;
float _pos_test_ratio;
float _hgt_test_ratio;
vector3f_t _mag_test_ratio;
bool _inhibit_wind_states;
bool _wind_state_aligned;
bool _inhibit_mag_states;
bool _last_inhibit_mag_status;
bool _need_mag_body_var_reset;
bool _need_earth_body_var_reset;
bool _inhibit_del_ang_bias_states;
bool _gps_in_use;

gps_elements_t _gps_data_new;
gps_elements_t _gps_data_delayed;
output_elements_t _output_data_new;
output_elements_t _output_data_delayed;
uint32_t _last_time_gps_received_ms;
gps_check_status_t _gps_check_status;
uint8_t _local_filter_time_step_ms;
float _gps_spd_accuracy;
float _gps_pos_accuracy;
float _gps_hgt_accuracy;
aiding_mode_t _pv_aiding_mode;
bool _use_gps_vert_vel;
bool _gps_good_to_align;
uint32_t _last_pre_align_gps_check_time_ms;
float _gps_drift_ne;
float _gps_vert_vel_filt;
float _gps_horiz_vel_filt;
location_t _gps_loc_prev;
uint32_t _last_gps_vel_fail_ms;
uint32_t _last_gps_vel_pass_ms;
bool _gps_spd_acc_pass;
bool _ekf_innovations_pass;
float _s_acc_filter_state1;
float _s_acc_filter_state2;
uint32_t _last_gps_check_time_ms;
uint32_t _last_innov_pass_time_ms;
uint32_t _last_innov_fail_time_ms;
bool _gps_accuracy_good;
vector3f_t _gps_vel_innov;
vector3f_t _gps_vel_var_innov;
uint32_t _gps_vel_innov_time_ms;
bool _gps_is_in_use;

vector3f_t _mag_test_ratio;
bool _consistent_mag_data;
uint32_t _mag_yaw_reset_timer_ms;
bool _motors_armed;
bool _mag_yaw_reset_request;

location_t _ekf_origin;
location_t *_public_origin;
bool _valid_origin;
double _ekf_gps_ref_hgt;
float _ekf_origin_hgt_var;

bool _have_table_earth_field;
vector3f_t _table_earth_field_ga;
float _table_declination;

uint8_t _mag_select_index;
bool _mag_field_learned;
bool _have_table_earth_field;
vector3f_t _table_earth_field_ga;
float _table_declination;

bool _inhibit_del_vel_bias_states;
bool _dvel_bias_axis_inhibit[3];
vector3f_t _dvel_bias_axis_var_prev;
uint8_t _state_index_lim;

extern float _gps_vert_vel_noise;
extern float _baro_alt_noise;

void init_ekf3_core(void)
{
    init_location(&_location);
    first_init_time_ms = 0;
    last_init_fail_report_ms = 0;
}

bool ekf3_setup_core(void)
{
    if (dal_ins_get_loop_rate_hz() > 0) {
        _dt_ekf_avg = 1.0f / dal_ins_get_loop_rate_hz();
        _dt_ekf_avg = MAX(_dt_ekf_avg, EKF_TARGET_DT);
    }
    uint16_t max_time_delay_ms = MAX(mag_delay_ms, EKF_TARGET_DT_MS);
    float gps_delay_sec = 0;
    dal_gps_get_lag(&gps_delay_sec);
    max_time_delay_ms = MAX(max_time_delay_ms, MIN(gps_delay_sec * 1000, 250));
    imu_buffer_length = (max_time_delay_ms / EKF_TARGET_DT_MS) + 1;
    uint16_t ekf_delay_ms = max_time_delay_ms + (int)(ceilf(max_time_delay_ms * 0.5f));
    DEBUG("ekf_delay ms = %d, imu_buffer_length = %d\n", ekf_delay_ms, imu_buffer_length);
    obs_buffer_length = ekf_delay_ms / _sensor_interval_min_ms + 1;
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
    /* if ((_source_set.yaw == YAW_COMPASS) && (compass_enabled() == 0)) { */
    /*     return YAW_NONE; */
    /* } */
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

void ekf3_core_calc_earth_rate_ned(vector3f_t *omega, int32_t latitude)
{
    float lat_rad = radians(latitude * 1.0e-7f);
    omega->x = earthRate * cosf(lat_rad);
    omega->y = 0;
    omega->z = earthRate * sinf(lat_rad);
}

void ekf3_core_zero_range(float *v, uint8_t n1, uint8_t n2)
{
    memset(&v[n1], 0, sizeof(float) * (1 + n2 - n1));
}

void ekf3_core_zero_rows(Matrix24 *cov_mat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = first; row <= last; row++) {
        ekf3_core_zero_range((float *)cov_mat + row * 24, 0, 23);
    }
}

void ekf3_core_zero_cols(Matrix24 *cov_mat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = 0; row <= 23; row++) {
        ekf3_core_zero_range((float *)cov_mat + row * 24, first, last);
    }
}

bool ekf3_core_get_llh(__attribute__((unused))location_t *loc)
{
    return true;
}

void ekf3_core_force_symmetry(void)
{
    for (uint8_t i = 1; i <= _state_index_lim ; i++) {
        for (uint8_t j = 0; j <= i - 1; j++) {
            float temp = 0.5f * (_P[i][j] + _P[j][i]);
            _P[i][j] = temp;
            _P[j][i] = temp;
        }
    }
}

void ekf3_core_constrain_variances(void)
{
    for (uint8_t i = 0; i <= 3 ; i++) {
        _P[i][i] = constrain_float(_P[i][i], 0.0, 1.0);
    }
    for (uint8_t i = 4; i <= 5; i++) {
        _P[i][i] = constrain_float(_P[i][i], VEL_STATE_MIN_VARIANCE, 1.0e3);
    }
    if (_bad_imu_data) {
        _P[6][6] = fmaxf(_P[6][6], sq(_gps_vert_vel_noise));
        _P[9][9] = fmaxf(_P[9][9], sq(_baro_alt_noise));
    } else if (_P[6][6] < VEL_STATE_MIN_VARIANCE) {
        _P[6][6] = VEL_STATE_MIN_VARIANCE;
        _vert_vel_var_clip_counter += EKF_TARGET_RATE_HZ;
        if (_vert_vel_var_clip_counter > VERT_VEL_VAR_CLIP_COUNT_LIM) {
           ekf3_core_zero_rows(&_P, 6, 6);
           ekf3_core_zero_cols(&_P, 6, 6);
           _P[6][6] = _gps_vert_vel_noise;
           _vert_vel_var_clip_counter = 0;
        }
    }
    for (uint8_t i =7; i <= 9; i++) {
        _P[i][i] = constrain_float(_P[i][i], POS_STATE_MIN_VARIANCE, 1.0e6);
    }
    if (!_inhibit_del_ang_bias_states) {
        for (uint8_t i = 10; i <= 12; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, sq(0.175 * _dt_ekf_avg));
        }
    } else {
        ekf3_core_zero_cols(&_P, 10, 12);
        ekf3_core_zero_rows(&_P, 10, 12);
    }
    const float min_state_var_target = 1e-11;
    if (!_inhibit_del_vel_bias_states) {
        const float min_safe_state_var = min_state_var_target * 0.1f;
        float max_state_var = min_safe_state_var;
        bool reset_required = false;
        for (uint8_t state_index = 13; state_index <= 15; state_index++) {
            if (_P[state_index][state_index] > max_state_var) {
                max_state_var = _P[state_index][state_index];
            } else if (_P[state_index][state_index] < min_safe_state_var) {
                reset_required = true;
            }
        }
        float min_allowed_state_var = fmaxf(0.01f * max_state_var, min_state_var_target);
        for (uint8_t state_index = 13; state_index <= 15; state_index++) {
            _P[state_index][state_index] = constrain_float(_P[state_index][state_index],
                                                           min_allowed_state_var,
                                                           sq(10.0f * _dt_ekf_avg));
        }
        if (reset_required) {
            float del_vel_bias_var[3];
            for (uint8_t i = 0; i <= 2; i++) {
                del_vel_bias_var[i] = _P[i + 13][i + 13];
            }
            ekf3_core_zero_cols(&_P, 13, 15);
            ekf3_core_zero_rows(&_P, 13, 15);
            for (uint8_t i = 0; i <= 2; i++) {
                _P[i + 13][i + 13] = del_vel_bias_var[i];
            }
        }
    } else {
        ekf3_core_zero_cols(&_P, 13, 15);
        ekf3_core_zero_rows(&_P, 13, 15);
        for (uint8_t i = 0; i <= 2; i++) {
            const uint8_t state_index = i + 13;
            _P[state_index][state_index] = fmaxf(_P[state_index][state_index], min_state_var_target);
        }
    }
    if (!_inhibit_mag_states) {
        for (uint8_t i = 16; i <= 18; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, 0.01f);
        }
        for (uint8_t i = 19; i <= 21; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, 0.01f);
        }
    } else {
        ekf3_core_zero_cols(&_P, 16, 21);
        ekf3_core_zero_rows(&_P, 16, 21);
    }
    if (!_inhibit_wind_states) {
        for (uint8_t i = 22; i <= 23; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, WIND_VEL_VARIANCE_MAX);
        }
    } else {
        ekf3_core_zero_cols(&_P, 22, 23);
        ekf3_core_zero_rows(&_P, 22, 23);

    }
}

void ekf3_core_reset_gyro_bias(void)
{
    v3f_zero(&_s.state_struct.gyro_bias);
    ekf3_core_zero_rows(&_P, 10, 12);
    ekf3_core_zero_cols(&_P, 10, 12);
    _P[10][10] = sq(radians(0.5 * _dt_imu_avg));
    _P[11][11] = _P[10][10];
    _P[12][12] = _P[10][10];
}
