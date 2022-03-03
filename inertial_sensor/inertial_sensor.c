#include <string.h>
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
#include "board_led.h"
#include "uart_device.h"
#include "accel_cal.h"
#include "common.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define ACCEL_TOT_MAX_OFFSET_CHANGE 4.0f
#define ACCEL_MAX_OFFSET 250.0f
#define ACCEL_VIBE_FLOOR_FILT_HZ 5.0f
#define ACCEL_VIBE_FILT_HZ 2.0f
#define ACCEL_PEAK_DETECT_TIMEOUT_MS 500

#define DEFAULT_GYRO_FILTER 4
#define DEFAULT_ACCEL_FILTER 10
#define DEFAULT_STILL_THRESH 0.1f

#define GYRO_INIT_MAX_DIFF_DPS 0.1f

__attribute__((unused))static uint8_t gyro_count = 1;
__attribute__((unused))static uint8_t accel_count = 1;
__attribute__((unused))static uint8_t backend_count = 1;

static uint16_t loop_rate;
extern accel_cal_t _acal;
extern accel_calibrator_t _accel_calibrator;
extern bool _mpu9250_have_init;

float _loop_delta_t;
static float loop_delta_t_max;
//extern bool _tcal_learning;
uint32_t _accel_clip_count;

bool _calibrating_accel;
bool _calibrating_gyro;
bool _acal_inited = false;
bool _accel_calibrator_inited = false;

vector3f_t _accel_scale = {0, 0, 0};
vector3f_t _accel_offset;
vector3f_t _gyro_offset = {0, 0, 0};
vector3f_t _accel_pos = {0, 0, 0};
static float accel_max_abs_offsets;
float still_threshold = DEFAULT_STILL_THRESH;

vector3f_t _gyro;
vector3f_t _accel;
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
static bool _have_sample = false;

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

float _delta_time = 0;
uint32_t _last_sample_usec;
uint32_t _next_sample_usec = 0;
uint32_t _sample_period_usec = 0;
static uint32_t _last_update_usec;

bool _accel_healthy;
bool _gyro_healthy;
static uint32_t accel_startup_error_count;
static uint32_t gyro_startup_error_count;
bool _startup_error_counts_set;
uint32_t _startup_ms;
uint32_t _accel_error_count;
uint32_t _gyro_error_count;

static uint8_t accel_over_sampling;
static uint16_t accel_raw_sampling_multiplier;
static bool accel_cal_ok;
static uint8_t gyro_over_sampling;
static uint16_t gyro_raw_sampling_multiplier;
static bool gyro_cal_ok;
static int8_t gyro_cal_timing = GYRO_CAL_STARTUP_ONLY;
//static Rotation_t board_orientation;
//static uint8_t gyro_wait_mask;
//static uint8_t accel_wait_mask;

extern float _clip_limit;

typedef struct peak_hold_state {
    float accel_peak_hold_neg_x;
    uint32_t accel_peak_hold_neg_x_age;
} peak_hold_state_t;

static peak_hold_state_t peak_hold_state;

bool inertial_sensor_init(void)
{
    imu_backend_init();
    if (!_mpu9250_have_init) {
        DEBUG("mpu 9250 haven't inited**************\n");
        if(!mpu9250_init())
            return false;
    }
    imu_backend_start();
    //_tcal_learning = true;
    return true;
}

uint16_t get_ins_loop_rate_hz(void)
{
    return loop_rate;
}

static bool calibrating(void)
{
    return _calibrating_accel || _calibrating_gyro || (_acal_inited && acal_running());
}

static void wait_for_sample(void)
{
    if (_have_sample) {
        return;
    }
    uint32_t now = xtimer_now().ticks32;
    if (_next_sample_usec == 0 && _delta_time <= 0) {
        _last_sample_usec = now - _sample_period_usec;
        _next_sample_usec = now + _sample_period_usec;
        goto check_sample;
    }
    if (_next_sample_usec - now <= _sample_period_usec) {
        uint32_t wait_usec = _next_sample_usec - now;
        xtimer_usleep(wait_usec);
        //uint32_t now2 = xtimer_now().ticks32;
        //if (now2 + 100 < _next_sample_usec) {
        //    MY_LOG("short sleep\n");
        //}
        // else if (now2 > _next_sample_usec + 400)
        // MY_LOG("long sleep\n");
        _next_sample_usec += _sample_period_usec;
    } else if (now - _next_sample_usec < _sample_period_usec / 8) {
        _next_sample_usec += _sample_period_usec;
    } else {
        _next_sample_usec = now + _sample_period_usec;
    }
check_sample:
    while (true) {
        if (_new_gyro_data && _new_accel_data) {
            break;
        }
        xtimer_usleep(100);
    }
    now = xtimer_now().ticks32;
    _delta_time = (now - _last_sample_usec) * 1.0e-6f;
    _last_sample_usec = now;
    _have_sample = true;
}


void ins_update(void)
{
    wait_for_sample();
    {
        _gyro_healthy = false;
        _accel_healthy = false;
        _delta_velocity_valid = false;
        _delta_angle_valid = false;
        mpu9250_update();
        if (!_startup_error_counts_set) {
            accel_startup_error_count = _accel_error_count;
            gyro_startup_error_count = _gyro_error_count;
            if (_startup_ms == 0) {
                _startup_ms = xtimer_now().ticks32 / 1000;
            } else if (xtimer_now().ticks32 / 1000 - _startup_ms > 2000) {
                _startup_error_counts_set = true;
            }
        }
        if (_accel_error_count < accel_startup_error_count) {
            accel_startup_error_count = _accel_error_count;
        }
        if (_gyro_error_count < gyro_startup_error_count) {
            gyro_startup_error_count = _gyro_error_count;
        }
        bool have_zero_accel_error_count = false;
        bool have_zero_gyro_error_count = false;
        if (_accel_healthy && _accel_error_count <= accel_startup_error_count) {
            have_zero_accel_error_count = true;
        }
        if (_gyro_healthy && _gyro_error_count <= gyro_startup_error_count) {
            have_zero_gyro_error_count = true;
        }
        if (_gyro_healthy && _gyro_error_count > gyro_startup_error_count &&
        have_zero_gyro_error_count) {
            _gyro_healthy = false;
            led_on(LED_1);
        }
        if (_accel_healthy && _accel_error_count > accel_startup_error_count &&
        have_zero_accel_error_count) {
            _accel_healthy = false;
            led_on(LED_1);
        }
    }
    _last_update_usec = xtimer_now().ticks32;
    _have_sample = false;
}

static void _init_gyro(void)
{
    //uint8_t num_gyros = 1;
    vector3f_t last_average, best_avg;
    vector3f_t new_gyro_offset;
    float best_diff;
    bool converged;
    if (calibrating()) {
        return;
    }
    _calibrating_gyro = true;
    led_on(LED_3);
    DEBUG("init gyro ...\n\n");
    //board_orientation = ROTATION_NONE;
    v3f_zero(&_gyro_offset);
    v3f_zero(&new_gyro_offset);
    best_diff = -1.f;
    v3f_zero(&last_average);
    converged = false;
    for (uint8_t c = 0; c < 5; c++) {
        xtimer_usleep(5000);
        ins_update();
    }
    //uint8_t num_converged = 0;
    for (int16_t j = 0; j <= 30 * 4; j++) {
        vector3f_t gyro_sum, gyro_avg, gyro_diff;
        vector3f_t accel_start;
        float diff_norm;
        uint8_t i;
        memset(&diff_norm, 0, sizeof(float));
        DEBUG("*\n");
        v3f_zero(&gyro_sum);
        accel_start = *get_accel();
        for (i = 0; i < 50; i++) {
            ins_update();
            gyro_sum = v3f_add(&gyro_sum, get_gyro());
            xtimer_usleep(5000);
        }
        vector3f_t accel_diff = v3f_sub(get_accel(), &accel_start);
        if (v3f_length(&accel_diff) > 0.2f) {
            continue;
        }
        gyro_avg = v3f_div(&gyro_sum, i);
        gyro_diff = v3f_sub(&last_average, &gyro_avg);
        diff_norm = v3f_length(&gyro_diff);
        if (best_diff < 0) {
            best_diff = diff_norm;
            best_avg = gyro_avg;
        } else if (diff_norm < ToRad(GYRO_INIT_MAX_DIFF_DPS)) {
            vector3f_t tmp1 = v3f_uniform_scale(&gyro_avg, 0.5);
            vector3f_t tmp2 = v3f_uniform_scale(&last_average, 0.5);
            last_average = v3f_add(&tmp1, &tmp2);
            if (!converged || v3f_length(&last_average) < v3f_length(&new_gyro_offset)) {
                new_gyro_offset = last_average;
            }
            if (!converged) {
                converged = true;
                break;
            }
        } else if (diff_norm < best_diff) {
            best_diff = diff_norm;
            vector3f_t tmp1 = v3f_uniform_scale(&gyro_avg, 0.5);
            vector3f_t tmp2 = v3f_uniform_scale(&last_average, 0.5);
            best_avg = v3f_add(&tmp1, &tmp2);
        }
        last_average = gyro_avg;
    }
    DEBUG("\n");
    for (uint8_t i = 0; i < 3; i++) {
        led_on(LED_3);
        xtimer_usleep(100000);
        led_off(LED_3);
        xtimer_usleep(900000);
    }
    if (!converged) {
        DEBUG("gyro did not converge: diff = %f dps (expected < %f)\n",
              ToDeg(best_diff), GYRO_INIT_MAX_DIFF_DPS);
        _gyro_offset = best_avg;
        gyro_cal_ok = false;
    } else {
        DEBUG("gyro converge: offset = %f dps\n", v3f_length(&new_gyro_offset));
        gyro_cal_ok = true;
        _gyro_offset = new_gyro_offset;
    }
    _calibrating_gyro = false;
}

static void _save_gyro_calibration(void)
{

}

void init_gyro(void)
{
    _init_gyro();
    _save_gyro_calibration();
}

static gyro_calibration_timing_t gyro_calibration_timing(void)
{
    return gyro_cal_timing;
}

void ins_init(uint16_t _loop_rate)
{
    accel_max_abs_offsets = 3.5f;
    loop_rate = _loop_rate;
    _loop_delta_t = 1.0f / loop_rate;
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
    if (v3f_is_zero(&_accel_scale)) {
        vector3f_t tmp = {1, 1, 1};
        _accel_scale = tmp;
    }
    if (gyro_calibration_timing() != GYRO_CAL_NEVER) {
        init_gyro();
    }
    _sample_period_usec = 1000 * 1000UL / loop_rate;
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
    return &_gyro;
}
vector3f_t* get_accel(void)
{
    return &_accel;
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

vector3f_t get_imu_pos_offset(void)
{
    return _accel_pos;
}

static bool gyro_calibrated_ok(void)
{
    return gyro_cal_ok;
}

bool gyro_calibrated_ok_all(void)
{
   if (!gyro_calibrated_ok()) {
       return false;
   }
   return true;
}

void acal_init(void)
{
    if (_acal_inited == false) {
        _acal_inited = true;
        _acal.use_gcs_snoop = true;
        _acal.started = false;
        _acal.saving = false;
        acal_update_status();
    }
    if (_accel_calibrator_inited == false) {
        _accel_calibrator_inited = true;
        _accel_calibrator.conf_tolerance = ACCEL_CAL_TOLERANCE;
        _accel_calibrator.sample_buffer = NULL;
        acal_cal_clear();
    }
}
