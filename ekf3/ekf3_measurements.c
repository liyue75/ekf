#include "ekf3_core.h"
#include "definitions.h"
#include "ekf_source.h"
#include "inertial_sensor.h"
#include "fusion_math.h"
#include "quaternion.h"
#include "matrix3f.h"
#include "ekf_buffer.h"

static uint8_t selected_gps;

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
extern state_elements_t _state_struct;
extern float _ognm_test_scale_factor;
extern imu_elements_t _imu_data_new;
extern imu_elements_t _imu_data_down_sampled_new;
extern quaternionf_t _imu_quat_down_sample_new;
extern vector3f_t *_accel_pos_offset;
extern uint32_t _frames_since_predict;
extern bool _start_predict_enabled;
extern ekf_imu_buffer_t _stored_imu;
extern bool _run_updates;
extern imu_elements_t _imu_data_delayed;
extern ekf_timing_t _timing;
extern vector3f_t _del_ang_corrected;
extern vector3f_t _del_vel_corrected;
extern inactive_bias_t _inactive_bias;


static void update_gps_selection(void)
{
    selected_gps = 1;
}

void ekf3_core_update_sensor_selection(void)
{
    update_gps_selection();
    //update_mag_selection();
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
    vector3f_t gyro_dt_bias = v3f_uniform_scale(&_state_struct.gyro_bias, dt_ekf_avg_inv);
    vector3f_t gyro = v3f_sub(get_gyro(), &gyro_dt_bias);
    vector3f_t accel_dt_bias = v3f_uniform_scale(&_state_struct.accel_bias, dt_ekf_avg_inv);
    vector3f_t accel = v3f_sub(get_accel(), &accel_dt_bias);
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
    get_delta_velocity(&d_velf, &d_vel_dtf);
    *d_vel = d_velf;
    *d_vel_dt = d_vel_dtf;
    *d_vel_dt = MAX(*d_vel_dt, 1.0e-4);
    return true;
}

static bool read_delta_angle(vector3f_t *d_ang, float *d_ang_dt)
{
    get_delta_angle(d_ang, d_ang_dt);
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
    _dt_imu_avg = 0.02f * constrain_float(get_loop_delta_t(), 0.5f * _dt_imu_avg,
                                         2.0f * _dt_imu_avg) + 0.98f * _dt_imu_avg;
    _imu_sample_time_ms = _imu_sample_time_us / 1000;
    //uint8_t accel_active = 0, gyro_active = 0;
    update_movement_check();
    read_delta_velocity(&_imu_data_new.del_vel, &_imu_data_new.del_vel_dt);
    _accel_pos_offset = get_imu_pos_offset();
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

void ekf3_core_read_gps_data(void)
{

}
