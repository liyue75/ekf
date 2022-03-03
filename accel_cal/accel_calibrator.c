#include <stddef.h>
#include <stdlib.h>
#include "accel_calibrator.h"
#include "board_led.h"
#include "uart_device.h"
#include "xtimer.h"
#include "definitions.h"

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

static bool running(void)
{
    return calibrator_status == ACCEL_CAL_WAITING_FOR_ORIENTATION ||
        calibrator_status == ACCEL_CAL_COLLECTING_SAMPLE;
}

void acal_cal_new_sample(__attribute__((unused))const vector3f_t* delta_velocity,
                     __attribute__((unused))float dt)
{

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
