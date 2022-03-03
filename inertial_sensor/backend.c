#include <stdbool.h>
#include <stddef.h>

#include "xtimer.h"
#include "mutex.h"

#include "tempcal.h"
#include "backend.h"
#include "accel_cal.h"
#include "fusion_math.h"
#include "definitions.h"
#include "inertial_sensor.h"
#include "lowpass_filter2p.h"
#include "uart_device.h"
#include "board_led.h"
#include "accel_cal.h"
//#define IMU_LOOP_RATE_HZ 8000
#define IMU_FAST_SAMPLING_RATE 0

extern bool _accel_calibrator_inited;
extern uint16_t _loop_rate;
static uint8_t fast_sampling_rate;
static uint16_t last_accel_filter_hz = 0;
static uint16_t last_gyro_filter_hz = 0;
extern uint16_t _accel_filter_cutoff;
extern uint16_t _gyro_filter_cutoff;
float _temperature;
//bool _tcal_learning;

static mutex_t backend_mutex;

extern bool _calibrating_accel;
extern bool _calibrating_gyro;
extern bool _acal_inited;

extern vector3f_t _accel_scale;
extern vector3f_t _accel_offset;
extern vector3f_t _gyro_offset;
extern vector3f_t _accel_pos;

extern uint16_t _sample_accel_count;
extern uint32_t _sample_accel_start_us;
extern uint64_t _accel_last_sample_us;
extern float _accel_raw_sample_rates;
extern uint16_t _sample_gyro_count;
extern uint32_t _sample_gyro_start_us;
extern uint64_t _gyro_last_sample_us;
extern float _gyro_raw_sample_rates;

extern vector3f_t _delta_velocity_acc;
extern float _delta_velocity_acc_dt;
extern float _delta_angle_acc_dt;
extern vector3f_t _delta_angle_acc;
extern vector3f_t _last_delta_angle;
extern vector3f_t _last_raw_gyro;

extern biquad_t _accel_filter;
extern biquad_t _gyro_filter;
extern vector3f_t _accel_filtered;
extern vector3f_t _gyro_filtered;
extern bool _new_accel_data;
extern bool _new_gyro_data;
extern vector3f_t _gyro;
extern vector3f_t _accel;
extern bool _accel_healthy;
extern bool _gyro_healthy;
extern vector3f_t _delta_velocity;
extern float _delta_velocity_dt;
extern bool _delta_velocity_valid;
extern vector3f_t _delta_angle;
extern float _delta_angle_dt;
extern bool _delta_angle_valid;

void imu_backend_init(void)
{
    fast_sampling_rate = IMU_FAST_SAMPLING_RATE;
    mutex_init(&backend_mutex);
}

uint8_t get_fast_sampling_rate(void)
{
    return (1 << fast_sampling_rate);
}

void rotate_and_correct_accel(vector3f_t *accel, Rotation_t orientation)
{
    rotate_vec(accel, orientation);
    /* if (_tcal_learning) { */
    /*     update_accel_learning(accel, _temperature); */
    /* } */
    if (!_calibrating_accel && (!_acal_inited || !acal_running())) {
        accel->x -= _accel_offset.x;
        accel->y -= _accel_offset.y;
        accel->z -= _accel_offset.z;
        accel->x *= _accel_scale.x;
        accel->y *= _accel_scale.y;
        accel->z *= _accel_scale.z;
    }
}

void rotate_and_correct_gyro(vector3f_t *gyro, Rotation_t orientation)
{
    rotate_vec(gyro, orientation);
    if (!_calibrating_gyro) {
       gyro->x -= _gyro_offset.x;
       gyro->y -= _gyro_offset.y;
       gyro->z -= _gyro_offset.z;
    }
}

void set_gyro_accel_orientation(Rotation_t *accel_orientation,
                                Rotation_t *gyro_orientation,
                                Rotation_t rotation)
{
    *accel_orientation = rotation;
    *gyro_orientation = rotation;
}

static bool sensors_converging(void)
{
    return xtimer_now().ticks32 < 30000000;
}

static void update_sensor_rate(uint16_t *count, uint32_t *start_us, float *rate_hz)
{
    uint32_t now = xtimer_now().ticks32;
    if (*start_us == 0) {
        *count = 0;
        *start_us = now;
    } else {
        *count = *count + 1;
        if (now - *start_us > 1000000UL) {
            float observed_rate_hz = *count * 1.0e6f / (now - *start_us);
            float filter_constant = 0.98f;
            float upper_limit = 1.05f;
            float lower_limit = 0.95f;
            if (sensors_converging()) {
                filter_constant = 0.8f;
                upper_limit = 2.0f;
                lower_limit = 0.5f;
            }
            observed_rate_hz = constrain_float(observed_rate_hz, *rate_hz * lower_limit,
                                               *rate_hz * upper_limit);
            *rate_hz = filter_constant * *rate_hz + (1 - filter_constant) * observed_rate_hz;
            *count = 0;
            *start_us = now;
        }
    }
}

void notify_new_accel_raw_sample(__attribute__((unused))vector3f_t *accel,
                                 __attribute__((unused))uint64_t sample_us,
                                 __attribute__((unused))bool fsync_set)
{
    float dt;
    update_sensor_rate(&_sample_accel_count, &_sample_accel_start_us,
                       &_accel_raw_sample_rates);
    uint64_t last_sample_us = _accel_last_sample_us;
    //if (sample_us != 0 && _accel_last_sample_us != 0) {
    //    dt = (sample_us - _accel_last_sample_us) * 1.0e-6f;
    //    _accel_last_sample_us = sample_us;
    //} else {
    dt = 1.0f / _accel_raw_sample_rates;
    _accel_last_sample_us = xtimer_now64().ticks64;
    sample_us = _accel_last_sample_us;
    //}
    calc_vibration_and_clipping(accel, dt);
    {
        mutex_lock(&backend_mutex);
        uint64_t now = xtimer_now64().ticks64;
        if (now - last_sample_us > 100000U) {
            v3f_zero(&_delta_velocity_acc);
            _delta_velocity_acc_dt = 0;
            dt = 0;
        }
        _delta_velocity_acc.x += accel->x * dt;
        _delta_velocity_acc.y += accel->y * dt;
        _delta_velocity_acc.z += accel->z * dt;
        _delta_velocity_acc_dt += dt;

        _accel_filtered = lpf2p_v3f_apply(accel, &_accel_filter);
        if (v3f_isnan(&_accel_filtered) || v3f_isinf(&_accel_filtered)) {
            lpf2p_v3f_reset(&_accel_filter);
            //MY_LOG("accel filtered is nan or inf\n");
            led_on(LED_1);
        }
        set_accel_peak_hold(&_accel_filtered);
        _new_accel_data = true;
        mutex_unlock(&backend_mutex);
    }
}

void notify_new_gyro_raw_sample(vector3f_t *gyro, __attribute__((unused))uint64_t sample_us)
{
    float dt;
    update_sensor_rate(&_sample_gyro_count, &_sample_gyro_start_us, &_gyro_raw_sample_rates);
    uint64_t last_sample_us = _gyro_last_sample_us;
    dt = 1.0f / _gyro_raw_sample_rates;
    _gyro_last_sample_us = xtimer_now64().ticks64;
    sample_us = _gyro_last_sample_us;
    vector3f_t delta_angle = {(gyro->x + _last_raw_gyro.x) * 0.5f * dt,
    (gyro->y + _last_raw_gyro.y) * 0.5f * dt,
    (gyro->z + _last_raw_gyro.z) * 0.5f * dt};
    vector3f_t delta_coning = {_delta_angle_acc.x + _last_delta_angle.x * (1.0f / 6.0f),
    _delta_angle_acc.y + _last_delta_angle.y * (1.0f / 6.0f),
    _delta_angle_acc.z + _last_delta_angle.z * (1.0f / 6.0f)};
    delta_coning = v3f_cross_product(&delta_coning, &delta_angle);
    delta_coning = v3f_uniform_scale(&delta_coning, 0.5f);
    {
        mutex_lock(&backend_mutex);
        uint64_t now = xtimer_now64().ticks64;
        if (now - last_sample_us > 100000U) {
            v3f_zero(&_delta_angle_acc);
            _delta_angle_acc_dt = 0;
            dt = 0;
            v3f_zero(&delta_angle);
        }
        vector3f_t tmp_delta = v3f_add(&delta_angle, &delta_coning);
        _delta_angle_acc = v3f_add(&_delta_angle_acc, &tmp_delta);
        _delta_angle_acc_dt += dt;
        _last_delta_angle = delta_angle;
        _last_raw_gyro = *gyro;
        vector3f_t gyro_filtered = *gyro;
        gyro_filtered = lpf2p_v3f_apply(&gyro_filtered, &_gyro_filter);
        if (v3f_isnan(&gyro_filtered) || v3f_isinf(&gyro_filtered) ) {
            lpf2p_v3f_reset(&_gyro_filter);
        } else {
            _gyro_filtered = gyro_filtered;
        }
        _new_gyro_data = true;

        mutex_unlock(&backend_mutex);
    }
}

static void publish_accel(vector3f_t *accel)
{
    _accel = *accel;
    _accel_healthy = true;
    _delta_velocity = _delta_velocity_acc;
    _delta_velocity_dt = _delta_velocity_acc_dt;
    _delta_velocity_valid = true;
    v3f_zero(&_delta_velocity_acc);
    _delta_velocity_acc_dt = 0;
    if (_accel_calibrator_inited && acal_cal_get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        vector3f_t cal_sample = _delta_velocity;
        //rotate_inverse(_board_orientation);
        acal_cal_new_sample(&cal_sample, _delta_velocity_dt);
    }

}

void backend_update_accel(void)
{
    mutex_lock(&backend_mutex);
    if (_new_accel_data) {
        publish_accel(&_accel_filtered);
        _new_accel_data = false;
    }
    if (last_accel_filter_hz != _accel_filter_cutoff) {
        lpf2p_set_cutoff_frequency(_accel_raw_sample_rates, _accel_filter_cutoff,
                                   &_accel_filter);
        last_accel_filter_hz = _accel_filter_cutoff;
    }
    mutex_unlock(&backend_mutex);
}

static void publish_gyro(vector3f_t * gyro)
{
    _gyro = *gyro;
    _gyro_healthy = true;
    _delta_angle = _delta_angle_acc;
    _delta_angle_dt = _delta_angle_acc_dt;
    _delta_angle_valid = true;
    v3f_zero(&_delta_angle_acc);
    _delta_angle_acc_dt = 0;
}

void backend_update_gyro(void)
{
    mutex_lock(&backend_mutex);
    if (_new_gyro_data) {
        publish_gyro(&_gyro_filtered);
        _new_gyro_data = false;
    }
    if (last_gyro_filter_hz != _gyro_filter_cutoff || sensors_converging()) {
        lpf2p_set_cutoff_frequency(_gyro_raw_sample_rates, _gyro_filter_cutoff,
                                   &_gyro_filter);
        last_gyro_filter_hz = _gyro_filter_cutoff;
    }
    mutex_unlock(&backend_mutex);
}

void backend_publish_temperature(float temperature)
{
    _temperature = temperature;
}
