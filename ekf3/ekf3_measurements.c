#include "ekf3_core.h"
#include "definitions.h"
#include "ekf_source.h"
//#include "inertial_sensor.h"
#include "ins_dal.h"
#include "fusion_math.h"
#include "quaternion.h"
#include "matrix3f.h"
#include "ekf_buffer.h"
//#include "gps.h"
#include "gps_dal.h"
#include "uart_device.h"
#include "ekf_source.h"
//#include "compass.h"
#include "uart_device.h"
#include "dal.h"
#include "compass_dal.h"
#include "nav_common.h"
#include "declination.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static uint8_t selected_gps;
extern bool _gps_is_in_use;
extern ekf_ring_buffer_t _stored_gps;
extern location_t _ekf_origin;
extern int8_t _origin_hgt_mode;
extern float _dt_imu_avg;
extern uint64_t _imu_sample_time_us;
extern uint32_t _imu_sample_time_ms;
extern bool _on_ground;
extern bool _on_ground_not_moving;
extern float _dt_ekf_avg;
extern bool _prev_on_ground;
extern vector3f_t _gyro_prev;
extern vector3f_t _accel_prev;
extern float _gyro_diff;
extern float _accel_diff;
extern state_var_t _s;
extern float _ognm_test_scale_factor;
extern imu_elements_t _imu_data_new;
extern imu_elements_t _imu_data_down_sampled_new;
extern quaternionf_t _imu_quat_down_sample_new;
extern vector3f_t _accel_pos_offset;
extern uint32_t _frames_since_predict;
extern bool _start_predict_enabled;
extern ekf_imu_buffer_t _stored_imu;
extern bool _run_updates;
extern imu_elements_t _imu_data_delayed;
extern ekf_timing_t _timing;
extern vector3f_t _del_ang_corrected;
extern vector3f_t _del_vel_corrected;
extern inactive_bias_t _inactive_bias;
//extern gps_timing_t _gps_timing;
//extern gps_status_t _gps_state;

extern uint32_t _last_time_gps_received_ms;
extern uint8_t _sensor_interval_min_ms;
extern gps_check_status_t _gps_check_status;
extern gps_elements_t _gps_data_new;
extern uint8_t _local_filter_time_step_ms;
extern float _gps_spd_accuracy;
extern float _gps_pos_accuracy;
extern float _gps_hgt_accuracy;
extern float _gps_horiz_vel_noise;
extern float _gps_horiz_pos_noise;
extern aiding_mode_t _pv_aiding_mode;
extern float _gps_noise_scaler;
extern bool _use_gps_vert_vel;
extern bool _valid_origin;
extern bool _in_flight;
extern bool _gps_good_to_align;
extern double _ekf_gps_ref_hgt;
extern float _ekf_origin_hgt_var;
extern bool _have_table_earth_field;
extern float _table_declination;
extern output_elements_t _output_data_new;
extern output_elements_t _output_data_delayed;
extern uint8_t _mag_select_index;

extern bool _have_table_earth_field;
extern int16_t _mag_ef_limit;
extern vector3f_t _table_earth_field_ga;
extern float _table_declination;

static void update_gps_selection(void)
{
    selected_gps = 0;
}

static void update_mag_selection(void)
{
    if (dal_compass_healthy() && dal_compass_use_for_yaw()) {
        DEBUG("compass healthy compass use_for_yaw");
        _mag_select_index = 0;
    }
}

void ekf3_core_update_sensor_selection(void)
{
    update_gps_selection();
    update_mag_selection();
}

static void update_movement_check(void)
{
    const source_yaw_t yaw_source = ekf3_get_yaw_source();
    const bool run_check = _on_ground && (yaw_source == YAW_GPS ||
                                          yaw_source == YAW_GPS_COMPASS_FALLBACK ||
                                          yaw_source == YAW_EXTNAV ||
    yaw_source == YAW_GSF || !ekf3_core_use_compass());
    if (!run_check) {
        _on_ground_not_moving = false;
        return;
    }
    __attribute__((unused))const float gyro_limit = radians(3.0f);
    const float gyro_diff_limit = 0.2f;
    __attribute__((unused))const float accel_limit = 1.0f;
    const float accel_diff_limit = 5.0f;
    __attribute__((unused))const float hysteresis_ratio = 0.7f;
    const float dt_ekf_avg_inv = 1.0f / _dt_ekf_avg;
    vector3f_t gyro_dt_bias = v3f_uniform_scale(&_s.state_struct.gyro_bias, dt_ekf_avg_inv);
    vector3f_t gyro_tmp = dal_ins_get_gyro();
    vector3f_t gyro = v3f_sub(&gyro_tmp, &gyro_dt_bias);
    vector3f_t accel_dt_bias = v3f_uniform_scale(&_s.state_struct.accel_bias, dt_ekf_avg_inv);
    vector3f_t accel_tmp = dal_ins_get_accel();
    vector3f_t accel = v3f_sub(&accel_tmp, &accel_dt_bias);
    if (!_prev_on_ground) {
        _gyro_prev = gyro;
        _accel_prev = accel;
        _on_ground_not_moving = false;
        _gyro_diff = gyro_diff_limit;
        _accel_diff = accel_diff_limit;
        return;
    }
    vector3f_t temp = v3f_sub(&gyro, &_gyro_prev);
    temp = v3f_uniform_scale(&temp, dt_ekf_avg_inv);
    _gyro_prev = gyro;
    _gyro_diff = _gyro_diff * 0.99f + v3f_length(&temp) * 0.01f;
    temp = v3f_sub(&accel, &_accel_prev);
    temp = v3f_uniform_scale(&temp, dt_ekf_avg_inv);
    _accel_prev = accel;
    _accel_diff = 0.99f * _accel_diff + 0.01f * v3f_length(&temp);

    const float gyro_length_ratio = v3f_length(&gyro) / gyro_limit;
    const float accel_length_ratio = (v3f_length(&accel)  - GRAVITY_MSS) / accel_limit;
    const float gyro_diff_ratio = _gyro_diff / gyro_diff_limit;
    const float accel_diff_ratio = _accel_diff / accel_diff_limit;
    if (_on_ground_not_moving) {
        if (gyro_length_ratio > _ognm_test_scale_factor ||
            fabsf(accel_length_ratio) > _ognm_test_scale_factor ||
            gyro_diff_ratio > _ognm_test_scale_factor ||
            accel_diff_ratio > _ognm_test_scale_factor) {
            _on_ground_not_moving = false;
        }
    } else if (gyro_length_ratio < _ognm_test_scale_factor * hysteresis_ratio &&
               fabsf(accel_length_ratio) < _ognm_test_scale_factor * hysteresis_ratio &&
               gyro_diff_ratio < _ognm_test_scale_factor * hysteresis_ratio &&
               accel_diff_ratio < _ognm_test_scale_factor * hysteresis_ratio) {
        _on_ground_not_moving = true;
    }

}

static bool read_delta_velocity(vector3f_t *d_vel, float *d_vel_dt)
{
    vector3f_t d_velf;
    float d_vel_dtf;
    dal_ins_get_delta_velocity(&d_velf, &d_vel_dtf);
    *d_vel = d_velf;
    *d_vel_dt = d_vel_dtf;
    *d_vel_dt = MAX(*d_vel_dt, 1.0e-4);
    return true;
}

static bool read_delta_angle(vector3f_t *d_ang, float *d_ang_dt)
{
    dal_ins_get_delta_angle(d_ang, d_ang_dt);
    return true;
}

static void update_timing_statistics(void)
{
    if (_timing.count == 0) {
        _timing.dt_imu_avg_max = _dt_imu_avg;
        _timing.dt_imu_avg_min = _dt_imu_avg;
        _timing.dt_ekf_avg_max = _timing.dt_ekf_avg_min = _dt_ekf_avg;
        _timing.del_ang_dt_max = _timing.del_ang_dt_min = _imu_data_delayed.del_ang_dt;
        _timing.del_vel_dt_max = _timing.del_vel_dt_min = _imu_data_delayed.del_vel_dt;
    } else {
        _timing.dt_imu_avg_max = MAX(_timing.dt_imu_avg_max, _dt_imu_avg);
        _timing.dt_imu_avg_min = MIN(_timing.dt_imu_avg_min, _dt_imu_avg);
        _timing.dt_ekf_avg_max = MAX(_timing.dt_ekf_avg_max, _dt_ekf_avg);
        _timing.dt_ekf_avg_min = MIN(_timing.dt_ekf_avg_min, _dt_ekf_avg);
        _timing.del_ang_dt_max = MAX(_timing.del_ang_dt_max, _imu_data_delayed.del_ang_dt);
        _timing.del_ang_dt_min = MIN(_timing.del_ang_dt_min, _imu_data_delayed.del_ang_dt);
        _timing.del_vel_dt_max = MAX(_timing.del_vel_dt_max, _imu_data_delayed.del_vel_dt);
        _timing.del_vel_dt_min = MIN(_timing.del_vel_dt_min, _imu_data_delayed.del_vel_dt);
    }
    _timing.count++;
}

void ekf3_core_read_imu_data(void)
{
    _dt_imu_avg = 0.02f * constrain_float(dal_ins_get_loop_delta_t(), 0.5f * _dt_imu_avg,
                                         2.0f * _dt_imu_avg) + 0.98f * _dt_imu_avg;
    _imu_sample_time_ms = _imu_sample_time_us / 1000;
    //uint8_t accel_active = 0, gyro_active = 0;
    //learn_inactive_biases();
    update_movement_check();
    read_delta_velocity(&_imu_data_new.del_vel, &_imu_data_new.del_vel_dt);
    vector3f_t tmp_pos = {0, 0, 0};
    _accel_pos_offset = tmp_pos;//dal_ins_get_imu_pos_offset();
    _imu_data_new.accel_index = 0;
    read_delta_angle(&_imu_data_new.del_ang, &_imu_data_new.del_ang_dt);
    _imu_data_new.del_ang_dt = MAX(_imu_data_new.del_ang_dt, 1.0e-4f);
    _imu_data_new.gyro_index = 0;
    _imu_data_new.time_ms = _imu_sample_time_ms;
    _imu_data_down_sampled_new.del_ang_dt += _imu_data_new.del_ang_dt;
    _imu_data_down_sampled_new.del_vel_dt += _imu_data_down_sampled_new.del_vel_dt;
    _imu_data_down_sampled_new.gyro_index = 0;
    _imu_data_down_sampled_new.accel_index = 0;
    quat_rotate_v(&_imu_quat_down_sample_new, &_imu_data_new.del_ang);
    quat_normalize(&_imu_quat_down_sample_new);
    matrix3f_t delta_rot_mat;
    quat_to_rotation_matrix(&_imu_quat_down_sample_new, &delta_rot_mat);
    vector3f_t tmp = m3f_multi_v(&delta_rot_mat, &_imu_data_new.del_vel);
    _imu_data_down_sampled_new.del_vel =  v3f_add(&_imu_data_down_sampled_new.del_vel, &tmp);
    _frames_since_predict++;
    if ((_imu_data_down_sampled_new.del_ang_dt >= (EKF_TARGET_DT - (_dt_imu_avg * 0.5f)) &&
        _start_predict_enabled) || (_imu_data_down_sampled_new.del_ang_dt >= (2.0f * EKF_TARGET_DT))) {
        quat_to_axis_angle(&_imu_quat_down_sample_new, &_imu_data_down_sampled_new.del_ang);
        _imu_data_down_sampled_new.time_ms = _imu_sample_time_ms;
        ekf_imu_buffer_push_youngest_element(&_stored_imu, &_imu_data_down_sampled_new);
        float dt_now = constrain_float(0.5f * (_imu_data_down_sampled_new.del_ang_dt +
                                               _imu_data_down_sampled_new.del_vel_dt),
                                       0.5f * _dt_ekf_avg,
                                       2.0f * _dt_ekf_avg);
        _dt_ekf_avg = 0.98f * _dt_ekf_avg + 0.02f * dt_now;
        v3f_zero(&_imu_data_down_sampled_new.del_ang);
        v3f_zero(&_imu_data_down_sampled_new.del_vel);
        _imu_data_down_sampled_new.del_ang_dt = 0.0f;
        _imu_data_down_sampled_new.del_vel_dt = 0.0f;
        _imu_quat_down_sample_new.q1 = 1.0f;
        _imu_quat_down_sample_new.q2 = _imu_quat_down_sample_new.q3 =
            _imu_quat_down_sample_new.q4 = 0.0f;
        _frames_since_predict = 0;
        _run_updates = true;
        ekf_imu_buffer_get_oldest_element(&_stored_imu, &_imu_data_delayed);
        float min_dt = 0.1f * _dt_ekf_avg;
        _imu_data_delayed.del_ang_dt = MAX(_imu_data_delayed.del_ang_dt, min_dt);
        _imu_data_delayed.del_vel_dt = MAX(_imu_data_delayed.del_vel_dt, min_dt);
        update_timing_statistics();
        _del_ang_corrected = _imu_data_delayed.del_ang;
        _del_vel_corrected = _imu_data_delayed.del_vel;
        ekf3_core_correct_delta_angle(&_del_ang_corrected, _imu_data_delayed.del_ang_dt);
        ekf3_core_correct_delta_velocity(&_del_vel_corrected, _imu_data_delayed.del_vel_dt);
    } else {
        _run_updates = false;
    }
}


static void get_earth_field_table(const location_t *loc)
{
    _table_earth_field_ga = get_earth_field_ga(loc);
    _table_declination = radians(get_declination(loc->lat * 10e-7, loc->lng * 10e-7));
    _have_table_earth_field = true;
}

void ekf3_core_read_gps_data(void)
{
    if(dal_gps_last_message_time_ms() - _last_time_gps_received_ms <= _sensor_interval_min_ms) {
        return;
    }
    if (dal_gps_status() < DAL_GPS_OK_FIX_3D) {
        _gps_check_status.bad_fix = true;
        MY_LOG("waiting for 3D fix\n");
        return;
    }
    _gps_check_status.bad_fix = false;
    const uint32_t second_last_gps_time_ms = _last_time_gps_received_ms;
    _last_time_gps_received_ms = dal_gps_last_message_time_ms();
    float gps_delay_sec = 0;
    dal_gps_get_lag(&gps_delay_sec);
    _gps_data_new.time_ms = _last_time_gps_received_ms - (uint32_t)(gps_delay_sec * 1000.0f);
    _gps_data_new.time_ms -= _local_filter_time_step_ms / 2;
    _gps_data_new.time_ms = MIN(MAX(_gps_data_new.time_ms, _imu_data_delayed.time_ms),
                                _imu_data_down_sampled_new.time_ms);
    _gps_data_new.sensor_idx = 0;
    _gps_data_new.vel = dal_gps_velocity();
    _gps_data_new.have_vz = dal_gps_have_vertical_velocity();
    _gps_data_new.corrected = false;
    float alpha = constrain_float(0.0002f * (_last_time_gps_received_ms -
                                             second_last_gps_time_ms), 0.0f, 1.0f);
    _gps_spd_accuracy *= (1.0f - alpha);
    float gps_spd_acc_raw;
    if (!dal_gps_speed_accuracy(&gps_spd_acc_raw)) {
        _gps_spd_accuracy = 0.0f;
    } else {
        _gps_spd_accuracy = MAX(_gps_spd_accuracy, gps_spd_acc_raw);
        _gps_spd_accuracy = MIN(_gps_spd_accuracy, 50.0f);
        _gps_spd_accuracy = MAX(_gps_spd_accuracy, _gps_horiz_vel_noise);
    }
    _gps_pos_accuracy *= (1.0f - alpha);
    float gps_pos_acc_raw;
    if (!dal_gps_horizontal_accuracy(&gps_pos_acc_raw)) {
        _gps_pos_accuracy = 0.0f;
    } else {
        _gps_pos_accuracy = MAX(_gps_pos_accuracy, gps_pos_acc_raw);
        _gps_pos_accuracy = MIN(_gps_pos_accuracy, 100.0f);
        _gps_pos_accuracy = MAX(_gps_pos_accuracy, _gps_horiz_pos_noise);
    }
    _gps_hgt_accuracy *= (1.0f - alpha);
    float gps_hgt_acc_raw;
    if (!dal_gps_vertical_accuracy(&gps_hgt_acc_raw)) {
        _gps_hgt_accuracy = 0.0f;
    } else {
        _gps_hgt_accuracy = MAX(_gps_hgt_accuracy, gps_hgt_acc_raw);
        _gps_hgt_accuracy = MIN(_gps_hgt_accuracy, 100.0f);
        _gps_hgt_accuracy = MAX(_gps_hgt_accuracy, 1.5f * _gps_horiz_pos_noise);
    }
    if (dal_gps_num_sats() >= 6 && (_pv_aiding_mode == AID_ABSOLUTE)) {
        _gps_noise_scaler = 1.0f;
    } else if (dal_gps_num_sats() == 5 && (_pv_aiding_mode == AID_ABSOLUTE)) {
        _gps_noise_scaler = 1.4;
    } else {
        _gps_noise_scaler = 2.0f;
    }
    if (_gps_data_new.have_vz && ekf_use_vel_z_source(Z_GPS)) {
        _use_gps_vert_vel = true;
    } else {
        _use_gps_vert_vel = false;
    }
    ekf3_core_calc_gps_good_to_align();
    ekf3_core_calc_gps_good_for_flight();
    const location_t *gpsloc = dal_gps_location();
    if (_gps_good_to_align && !_valid_origin) {
        location_t gpsloc_fieldelevation = *gpsloc;
        if (_in_flight) {
            gpsloc_fieldelevation.alt += (int32_t)(100.0f * _s.state_struct.position.z);
        }
        if (!ekf3_core_set_origin(&gpsloc_fieldelevation)) {
            return;
        }
        ekf3_core_align_mag_state_declination();
        _ekf_gps_ref_hgt = (double)0.01 * (double)gpsloc->alt +
            (double)_output_data_new.position.z;
        _ekf_origin_hgt_var = sq(_gps_hgt_accuracy);
    }
    if (_gps_good_to_align && !_have_table_earth_field) {
        if (dal_compass_have_scale_factor() &&
        dal_compass_auto_declination_enabled()) {
            get_earth_field_table(gpsloc);
            if (_mag_ef_limit > 0) {
                _s.state_struct.earth_magfield = _table_earth_field_ga;
            }
        }
    }
    if (_valid_origin) {
        _gps_data_new.lat = gpsloc->lat;
        _gps_data_new.lng = gpsloc->lng;
        if ((_origin_hgt_mode & (1 << 2)) == 0) {
            _gps_data_new.hgt = (float)(0.01 * (double)gpsloc->alt - _ekf_gps_ref_hgt);
        } else {
            _gps_data_new.hgt = 0.01 * (gpsloc->alt - _ekf_origin.alt);
        }
        ekf_ring_buffer_push(&_stored_gps, &_gps_data_new);
        _gps_is_in_use = true;
    }
}

float ekf3_core_mag_declination(void)
{
    if (_have_table_earth_field && _mag_ef_limit > 0) {
        return _table_declination;
    }
    if(!ekf3_core_use_compass()) {
        return 0;
    }
    return dal_compass_get_declination();
}
