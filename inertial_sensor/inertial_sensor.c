#include <math.h>
#include "inertial_sensor.h"
#include "xtimer.h"
#include "mpu9250.h"
#include "backend.h"
#include "vector3f.h"
#include "lowpass_filter_v3f.h"
#include "lowpass_filter2p.h"
#include "fusion_math.h"
#include "definitions.h"

#define ACCEL_TOT_MAX_OFFSET_CHANGE 4.0f
#define ACCEL_MAX_OFFSET 250.0f
#define ACCEL_VIBE_FLOOR_FILT_HZ 5.0f
#define ACCEL_VIBE_FILT_HZ 2.0f
#define ACCEL_PEAK_DETECT_TIMEOUT_MS 500

#define DEFAULT_GYRO_FILTER 4
#define DEFAULT_ACCEL_FILTER 10
#define DEFAULT_STILL_THRESH 0.1f

uint16_t _loop_rate;

float _loop_delta_t;
static float loop_delta_t_max;
//extern bool _tcal_learning;
uint32_t _accel_clip_count;

bool _calibrating_accel;
bool _calibrating_gyro;
void *_acal;

vector3f_t _accel_scale;
vector3f_t _accel_offset;
vector3f_t _gyro_offset;
vector3f_t _accel_pos;
float accel_max_abs_offsets;

static vector3f_t gyro;
static vector3f_t accel;
vector3f_t _delta_velocity;
vector3f_t _delta_angle;
bool _delta_velocity_valid;
bool _delta_angle_valid;
float _delta_velocity_dt;
float _delta_angle_dt;
vector3f_t _delta_velocity_acc;
float _delta_velocity_acc_dt;
float _delta_angle_acc_dt;
vector3f_t _delta_angle_acc;
vector3f_t _last_delta_angle;
vector3f_t _last_raw_gyro;

static lpf_v3f_t accel_vibe_floor_filter;
static lpf_v3f_t accel_vibe_filter;
biquad_t _accel_filter;
biquad_t _gyro_filter;
uint16_t _accel_filter_cutoff = DEFAULT_ACCEL_FILTER;
uint16_t _gyro_filter_cutoff = DEFAULT_GYRO_FILTER;
vector3f_t _accel_filtered;
vector3f_t _gyro_filtered;
bool _new_accel_data;
bool _new_gyro_data;
static bool _have_sample;

float _temp_sensitivity = 1.0f / 333.87; // degC/LSB
float _temp_zero = 21.0; //degC
biquad_t _temp_filter;
float _temp_filtered;

uint16_t _sample_accel_count;
uint32_t _sample_accel_start_us;
uint64_t _accel_last_sample_us;
float _accel_raw_sample_rates;
uint16_t _sample_gyro_count;
uint32_t _sample_gyro_start_us;
uint64_t _gyro_last_sample_us;
float _gyro_raw_sample_rates;

float _delta_time;
uint32_t _last_sample_usec;
uint32_t _next_sample_usec;
uint32_t _sample_period_usec;
static uint32_t _last_update_usec;

bool _accel_healthy;
bool _gyro_healthy;

static uint8_t accel_over_sampling;
static uint16_t accel_raw_sampling_multiplier;
static bool accel_cal_ok;
static uint8_t gyro_over_sampling;
static uint16_t gyro_raw_sampling_multiplier;
static bool gyro_cal_ok;

extern float _clip_limit;

typedef struct peak_hold_state {
    float accel_peak_hold_neg_x;
    uint32_t accel_peak_hold_neg_x_age;
} peak_hold_state_t;

static peak_hold_state_t peak_hold_state;

bool inertial_sensor_init(void)
{
    backend_init();
    if(!sensor_init())
        return false;
    sensor_start();
    //_tcal_learning = true;
    return true;
}

uint16_t get_ins_loop_rate_hz(void)
{
    return _loop_rate;
}

static void _init_gyro(void)
{

}

static void _save_gyro_calibration(void)
{

}
static void init_gyro(void)
{
    _init_gyro();
    _save_gyro_calibration();
}

void ins_init(uint16_t loop_rate)
{
    _loop_rate = loop_rate;
    _loop_delta_t = 1.0f / _loop_rate;
    loop_delta_t_max = 10 * _loop_delta_t;
    init_lpf2p_v3f(&_accel_filter);
    init_lpf2p_v3f(&_gyro_filter);
    init_lpf2p_v3f(&_temp_filter);
    lpf2p_set_cutoff_frequency(1000, 1, &_temp_filter);
    init_lpf_v3f(&accel_vibe_floor_filter);
    init_lpf_v3f(&accel_vibe_filter);
    set_cutoff_freq_v3f(ACCEL_VIBE_FLOOR_FILT_HZ, &accel_vibe_floor_filter);
    set_cutoff_freq_v3f(ACCEL_VIBE_FILT_HZ, &accel_vibe_floor_filter);
    inertial_sensor_init();
    vector3f_t tmp = {1, 1, 1};
    _accel_scale = tmp;
    init_gyro();
    _sample_period_usec = 1000 * 1000UL / _loop_rate;
    _delta_time = 0;
    _next_sample_usec = 0;
    _last_sample_usec = 0;
    _have_sample = false;
}

void calc_vibration_and_clipping(vector3f_t *accel, float dt)
{
    if (fabsf(accel->x) > _clip_limit ||
        fabsf(accel->y) > _clip_limit ||
        fabsf(accel->z) > _clip_limit) {
        _accel_clip_count++;
    }
    vector3f_t accel_filt = lpf_v3f_apply_dt(accel, dt, &accel_vibe_floor_filter);
    vector3f_t accel_diff = {accel->x - accel_filt.x,
    accel->y - accel_filt.y,
    accel->z - accel_filt.z};
    accel_diff.x *= accel_diff.x;
    accel_diff.y *= accel_diff.y;
    accel_diff.z *= accel_diff.z;
    lpf_v3f_apply_dt(&accel_diff, dt, &accel_vibe_filter);
}

void set_accel_peak_hold(const vector3f_t *accel)
{
    uint32_t now = xtimer_now().ticks32 / 1000;
    if (accel->x < peak_hold_state.accel_peak_hold_neg_x ||
    peak_hold_state.accel_peak_hold_neg_x_age <= now) {
        peak_hold_state.accel_peak_hold_neg_x = accel->x;
        peak_hold_state.accel_peak_hold_neg_x_age = now + ACCEL_PEAK_DETECT_TIMEOUT_MS;
    }
}

bool register_gyro(uint16_t raw_sample_rate_hz)
{
    _gyro_raw_sample_rates = raw_sample_rate_hz;
    gyro_over_sampling = 1;
    gyro_raw_sampling_multiplier = INT16_MAX / radians(2000);
    gyro_cal_ok = false;
    return true;
}

bool register_accel(uint16_t raw_sample_rate_hz)
{
    _accel_raw_sample_rates = raw_sample_rate_hz;
    accel_over_sampling = 1;
    accel_raw_sampling_multiplier = INT16_MAX / (16 * GRAVITY_MSS);
    accel_cal_ok = false;
    return true;
}

void notify_gyro_fifo_reset(void)
{
    _sample_gyro_count = 0;
    _sample_gyro_start_us = 0;
}

void notify_accel_fifo_reset(void)
{
    _sample_accel_count = 0;
    _sample_accel_start_us = 0;
}

void set_accel_oversampling(uint8_t n)
{
    accel_over_sampling = n;
}

void set_gyro_oversampling(uint8_t n)
{
    gyro_over_sampling = n;
}

void set_raw_sample_accel_multiplier(uint16_t mul)
{
    accel_raw_sampling_multiplier = mul;
}

uint32_t get_last_update_usec(void)
{
    return _last_update_usec;
}

float get_loop_delta_t(void)
{
    return _loop_delta_t;
}

vector3f_t* get_gyro(void)
{
    return &gyro;
}
vector3f_t* get_accel(void)
{
    return &accel;
}

static float get_delta_time(void)
{
    return MIN(_delta_time, loop_delta_t_max);
}

bool get_accel_health(void)
{
    return _accel_healthy;
}
bool get_gyro_health(void)
{
    return _gyro_healthy;
}

bool get_delta_velocity(vector3f_t *delta_velocity, float *delta_velocity_dt)
{
    if (_delta_velocity_valid) {
        *delta_velocity_dt = _delta_velocity_dt;
    } else {
        *delta_velocity_dt = get_delta_time();
    }
    *delta_velocity_dt = MIN(*delta_velocity_dt, loop_delta_t_max);
    if (_delta_velocity_valid) {
        *delta_velocity = _delta_velocity;
        return true;
    } else if (get_accel_health()) {
        *delta_velocity = v3f_uniform_scale(get_accel(), get_delta_time());
        return true;
    }
    return false;
}

bool get_delta_angle(vector3f_t *delta_angle, float *delta_angle_dt)
{
    if (_delta_angle_valid && _delta_angle_dt > 0) {
        *delta_angle_dt = _delta_angle_dt;
    } else {
        *delta_angle_dt = get_delta_time();
    }
    *delta_angle_dt = MIN(*delta_angle_dt, loop_delta_t_max);
    if (_delta_angle_valid) {
        *delta_angle = _delta_angle;
        return true;
    } else if (get_gyro_health()) {
        *delta_angle = v3f_uniform_scale(get_gyro(), get_delta_time());
    }
    return false;
}

vector3f_t *get_imu_pos_offset(void)
{
    return &_accel_pos;
}
