#include <stddef.h>
#include <stdlib.h>
#include "accel_calibrator.h"
#include "matrix3f.h"
#include "board_led.h"
#include "uart_device.h"
#include "xtimer.h"
#include "definitions.h"
#include "fusion_math.h"
#include "matrix_alg.h"

#define ENABLE_DEBUG 1
#include "debug.h"

accel_calibrator_t _accel_calibrator;
static accel_cal_status_t calibrator_status;
static uint8_t samples_collected;
static uint8_t conf_num_samples;
static float conf_sample_time;
accel_cal_fit_type_t conf_fit_type;
static uint32_t last_samp_frag_collected_ms;
static float min_sample_dist;
static acal_cal_param_u param;
//static float min_sample_dist;
static float fitness;

static bool running(void)
{
    return calibrator_status == ACCEL_CAL_WAITING_FOR_ORIENTATION ||
        calibrator_status == ACCEL_CAL_COLLECTING_SAMPLE;
}

static void set_status(accel_cal_status_t status)
{
    switch (status) {
        case ACCEL_CAL_NOT_STARTED:
            calibrator_status = ACCEL_CAL_NOT_STARTED;
            samples_collected = 0;
            if (_accel_calibrator.sample_buffer != NULL) {
                free(_accel_calibrator.sample_buffer);
                _accel_calibrator.sample_buffer = NULL;
            }
            break;
        case ACCEL_CAL_WAITING_FOR_ORIENTATION:
            if (!running()) {
                samples_collected = 0;
                if (_accel_calibrator.sample_buffer == NULL) {
                    _accel_calibrator.sample_buffer = (cal_accel_sample_t *)calloc(
                        conf_num_samples, sizeof(cal_accel_sample_t));
                    if (_accel_calibrator.sample_buffer == NULL) {
                        set_status(ACCEL_CAL_FAILED);
                        MY_LOG("calloc calibrate accel sample failed\n");
                        led_on(LED_1);
                        break;
                    }
                }
            }
            if (samples_collected >= conf_num_samples) {
                break;
            }
            calibrator_status = ACCEL_CAL_WAITING_FOR_ORIENTATION;
            break;
        case ACCEL_CAL_COLLECTING_SAMPLE:
            if (calibrator_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
                break;
            }
            last_samp_frag_collected_ms = xtimer_now().ticks32 / 1000;
            calibrator_status = ACCEL_CAL_COLLECTING_SAMPLE;
            break;
        case ACCEL_CAL_SUCCESS:
            if (calibrator_status != ACCEL_CAL_COLLECTING_SAMPLE) {
                break;
            }
            calibrator_status = ACCEL_CAL_SUCCESS;
            break;
        case ACCEL_CAL_FAILED:
            if (calibrator_status == ACCEL_CAL_NOT_STARTED) {
                break;
            }
            calibrator_status = ACCEL_CAL_FAILED;
            break;
    }
}

static bool get_sample(uint8_t i, vector3f_t *s)
{
    if (i >= samples_collected) {
        return false;
    }
    *s = v3f_div(&_accel_calibrator.sample_buffer[i].delta_velocity,
                 _accel_calibrator.sample_buffer[i].delta_time);
    return true;
}

static bool accept_sample(vector3f_t *sample)
{
    if (_accel_calibrator.sample_buffer == NULL) {
        return false;
    }
    for (uint8_t i = 0; i < samples_collected; i++) {
        vector3f_t other_sample;
        get_sample(i, &other_sample);
        vector3f_t tmp = v3f_sub(&other_sample, sample);
        if (v3f_length(&tmp) < min_sample_dist) {
            return false;
        }
    }
    return true;
}

static float calc_residual(const vector3f_t *sample, const acal_cal_param_t *params)
{
    matrix3f_t M = {{params->diag.x, params->offdiag.x, params->offdiag.y},
                    {params->offdiag.x, params->diag.y, params->offdiag.z},
                    {params->offdiag.y, params->offdiag.z, params->diag.z}};
    vector3f_t tmp = v3f_add(sample, &params->offset);
    tmp = m3f_multi_v(&M, &tmp);
    return GRAVITY_MSS - v3f_length(&tmp);
}

static float calc_mean_squared_residuals(const acal_cal_param_t *params)
{
    if (samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples_collected; i++) {
        vector3f_t sample;
        get_sample(i, &sample);
        float resid = calc_residual(&sample, params);
        sum += sq(resid);
    }
    sum /= samples_collected;
    return sum;
}

static void calc_jacob(const vector3f_t *sample, const acal_cal_param_t *params, float *ret)
{
    if (conf_fit_type == ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID ||
        conf_fit_type == ACCEL_CAL_ELLIPSOID) {
        const vector3f_t *offset = &params->offset;
        const vector3f_t *diag = &params->diag;
        const vector3f_t *offdiag = &params->offdiag;
        matrix3f_t M = {{diag->x, offdiag->x, offdiag->y},
                        {offdiag->x, diag->y, offdiag->z},
                        {offdiag->y, offdiag->z, diag->z}};
        float A = (diag->x * (sample->x + offset->x)) + (offdiag->x * (sample->y + offset->y)) +
            (offdiag->y * (sample->z + offset->z));
        float B = (offdiag->x * (sample->x + offset->x)) + (diag->y * (sample->y + offset->y)) +
            (offdiag->z * (sample->z + offset->z));
        float C = (offdiag->y * (sample->x + offset->x)) + (offdiag->z * (sample->y + offset->y)) +
            (diag->z + (sample->z + offset->z));
        vector3f_t tmp = v3f_add(sample, offset);
        tmp = m3f_multi_v(&M, &tmp);
        float length = v3f_length(&tmp);
        ret[0] = -1.0f * (((diag->x * A) + (offdiag->x * B) + (offdiag->y * C)) / length);
        ret[1] = -1.0f * (((offdiag->x * A) + (diag->y * B) + (offdiag->z * C)) / length);
        ret[2] = -1.0f * (((offdiag->y * A) + (offdiag->z * B) + (diag->z * C)) / length);
        ret[3] = -1.0f * ((sample->x + offset->x) * A) / length;
        ret[4] = -1.0f * ((sample->y + offset->y) * B) / length;
        ret[5] = -1.0f * ((sample->z + offset->z) * C) / length;
        ret[6] = -1.0f * (((sample->y + offset->y) * A) + ((sample->x + offset->x) * B)) / length;
        ret[7] = -1.0f * (((sample->z + offset->z) * A) + ((sample->x + offset->x) * C)) / length;
        ret[8] = -1.0f * (((sample->z + offset->z) * B) + ((sample->y + offset->y) * C)) / length;
        return;
    }
}

static uint8_t get_num_params(void)
{
    switch (conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            return 6;
        case ACCEL_CAL_ELLIPSOID:
            return 9;
        default:
            return 6;
    }
}


static void run_fit(uint8_t max_iterations, float *fitness)
{
    if (_accel_calibrator.sample_buffer == NULL) {
        return;
    }
    *fitness = calc_mean_squared_residuals(&param.s);
    float min_fitness = *fitness;
    acal_cal_param_u fit_param = param;
    uint8_t num_iterations = 0;
    uint8_t num_params = get_num_params();
    while (num_iterations < max_iterations) {
        float JTJ[ACCEL_CAL_MAX_NUM_PARAMS * ACCEL_CAL_MAX_NUM_PARAMS] = {0};
        float JTFI[ACCEL_CAL_MAX_NUM_PARAMS] = {0};
        for (uint16_t k = 0; k < samples_collected; k++) {
            vector3f_t sample;
            get_sample(k, &sample);
            float jacob[ACCEL_CAL_MAX_NUM_PARAMS] = {0};
            calc_jacob(&sample, &fit_param.s, jacob);
            for (uint8_t i = 0; i < num_params; i++) {
                for (uint8_t j = 0; j < num_params; j++) {
                    JTJ[i * num_params + j] += jacob[i] * jacob[j];
                }
                JTFI[i] += jacob[i] * calc_residual(&sample, &fit_param.s);
            }
        }
        if (!mat_inverse(JTJ, JTJ, num_params)) {
           return;
        }
        for (uint8_t row = 0; row < num_params; row++) {
            for (uint8_t col=0; col < num_params; col++) {
                fit_param.a[row] -= JTFI[col] * JTJ[row*num_params+col];
            }
        }
        *fitness = calc_mean_squared_residuals(&fit_param.s);
        if (isnan(*fitness) || isinf(*fitness)) {
            return;
        }
        if (*fitness < min_fitness) {
            min_fitness = *fitness;
            param = fit_param;
        }
        num_iterations++;
    }
}

static bool accept_result(void)
{
    if (fabsf(param.s.offset.x) > GRAVITY_MSS ||
        fabsf(param.s.offset.y) > GRAVITY_MSS ||
        fabsf(param.s.offset.z) > GRAVITY_MSS ||
        param.s.diag.x < 0.8f || param.s.diag.x > 1.2f ||
        param.s.diag.y < 0.8f || param.s.diag.y > 1.2f ||
        param.s.diag.z < 0.8f || param.s.diag.z > 1.2f) {
        return false;
    } else {
        return true;
    }
}

void acal_cal_new_sample(__attribute__((unused))const vector3f_t* delta_velocity,
                     __attribute__((unused))float dt)
{
    if (calibrator_status != ACCEL_CAL_COLLECTING_SAMPLE) {
        return;
    }
    if (samples_collected >= conf_num_samples) {
        set_status(ACCEL_CAL_FAILED);
        return;
    }
    _accel_calibrator.sample_buffer[samples_collected].delta_velocity = v3f_add(
        &_accel_calibrator.sample_buffer[samples_collected].delta_velocity, delta_velocity);
    _accel_calibrator.sample_buffer[samples_collected].delta_time += dt;
    last_samp_frag_collected_ms = xtimer_now().ticks32 / 1000;

    if (_accel_calibrator.sample_buffer[samples_collected].delta_time > conf_sample_time) {
        led_off(LED_1); led_off(LED_2); led_off(LED_3);
        vector3f_t sample = v3f_div(&_accel_calibrator.sample_buffer[samples_collected].delta_velocity,
            _accel_calibrator.sample_buffer[samples_collected].delta_time);
        if (!accept_sample(&sample)) {
            set_status(ACCEL_CAL_FAILED);
            return;
        }
        samples_collected++;
        if (samples_collected >= conf_num_samples) {
            uint32_t start = xtimer_now().ticks32;
            run_fit(MAX_ITERATIONS, &fitness);
            uint32_t end = xtimer_now().ticks32;
            MY_LOG("cal fit time %ld\n", end - start);
            if (fitness < _accel_calibrator.conf_tolerance && accept_result()) {
                set_status(ACCEL_CAL_SUCCESS);
                led_on(LED_1); led_on(LED_2); led_on(LED_3);
            } else {
                set_status(ACCEL_CAL_FAILED);
            }
        } else {
            set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
        }
    }
}


void acal_cal_clear(void)
{
    set_status(ACCEL_CAL_NOT_STARTED);
}

accel_cal_status_t acal_cal_get_status(void)
{
    return calibrator_status;
}

static void start(accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time,
                    vector3f_t offset, vector3f_t diag, vector3f_t offdiag)
{
    if (calibrator_status == ACCEL_CAL_FAILED || calibrator_status == ACCEL_CAL_SUCCESS) {
        acal_cal_clear();
    }
    if (calibrator_status != ACCEL_CAL_NOT_STARTED) {
        return;
    }
    conf_num_samples = num_samples;
    conf_sample_time = sample_time;
    conf_fit_type = fit_type;
    const uint16_t faces = 2 * conf_num_samples - 4;
    const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    const float theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));
    min_sample_dist = GRAVITY_MSS * 2 * sinf(theta / 2);

    param.s.offset = offset;
    param.s.diag = diag;
    param.s.offdiag = offdiag;
    switch (conf_fit_type) {
        case ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID:
            if (conf_num_samples < 6) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
        case ACCEL_CAL_ELLIPSOID:
            if (conf_num_samples < 8) {
                set_status(ACCEL_CAL_FAILED);
                return;
            }
            break;
    }
    set_status(ACCEL_CAL_WAITING_FOR_ORIENTATION);
}

void acal_cal_start(accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time)
{
    vector3f_t offset = {0, 0, 0};
    vector3f_t diag = {1, 1, 1};
    vector3f_t offdiag = {0, 0, 0};
    start(fit_type, num_samples, sample_time, offset, diag, offdiag);
}

void acal_cal_collect_sample(void)
{
    set_status(ACCEL_CAL_COLLECTING_SAMPLE);
}

uint8_t acal_cal_get_num_samples_collected(void)
{
    return samples_collected;
}

void acal_cal_check_for_timeout(void)
{
    const uint32_t timeout = conf_sample_time * 2 * 1000 + 500;
    if (calibrator_status == ACCEL_CAL_COLLECTING_SAMPLE && (xtimer_now().ticks32 / 1000 -
    last_samp_frag_collected_ms > timeout)) {
        set_status(ACCEL_CAL_FAILED);
    }
 }

void acal_cal_get_calibration(vector3f_t *offset, vector3f_t *diag)
{
    offset->x = -param.s.offset.x;
    offset->y = -param.s.offset.y;
    offset->z = -param.s.offset.z;
    *diag = param.s.diag;
}
