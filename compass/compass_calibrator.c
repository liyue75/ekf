#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fusion_math.h"
#include "xtimer.h"
#include "compass_calibrator.h"
#include "matrix3f.h"
#include "geodesic_grid.h"
#include "mutex.h"
#include "ahrs.h"
#include "uart_device.h"
#include "board_led.h"
#include "matrix_alg.h"
#include "gps.h"
#include "location.h"
#include "declination.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define FIELD_RADIUS_MIN 150
#define FIELD_RADIUS_MAX 950

#define COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(X) ((int16_t)constrain_float(roundf(X*8.0f), INT16_MIN, INT16_MAX))
#define COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(X) (X/8.0f)

compass_cal_state_t cal_state;
static uint8_t compass_idx;
static compass_cal_status_t status;
//values provided by caller
static float delay_start_sec;
__attribute__((unused))static bool retry;
__attribute__((unused))static float tolerance = 5.0;
__attribute__((unused))static uint16_t offset_max;

//behavioral state
__attribute__((unused))static uint32_t start_time_ms;
static uint8_t attempt;
static completion_mask_t completion_mask;
static compass_cal_compass_sample_t *sample_buffer;
static uint16_t samples_collected;
static uint16_t samples_thinned;

//fit state
static compass_cal_param_t params;
static int8_t roll;
static int8_t pitch;
static int8_t yaw;
static uint16_t fit_step;
static float fitness;
static float initial_fitness;
static float sphere_lambda;
static float ellipsoid_lambda;

//variables for orientation checking
Rotation_t orientation;
Rotation_t orig_orientation;
Rotation_t orientation_solution;
static bool is_external;
static bool check_orientation;
static bool fix_orientation;
static bool always_45_deg;
static float orientation_confidence;
static compass_cal_compass_sample_t last_sample;
static bool new_sample;

static float theta;
static mutex_t state_sem;
static mutex_t sample_sem;
static compass_cal_settings_t cal_settings;
static compass_cal_status_t requested_status;
static bool status_set_requested;
static compass_report_t cal_report;

compass_cal_state_t compass_calibrator_get_state(void)
{
    mutex_lock(&state_sem);
    compass_cal_state_t tmp = cal_state;
    mutex_unlock(&state_sem);
    return tmp;
}

void compass_calibrator_init(void)
{
    const uint16_t faces = (2 * COMPASS_CAL_NUM_SAMPLES - 4);
    const float a = (4.0f * M_PI / (3.0f * faces)) + M_PI / 3.0f;
    theta = 0.5f * acosf(cosf(a) / (1.0f - cosf(a)));
    mutex_init(&state_sem);
    mutex_init(&sample_sem);
}

static vector3f_t compass_sample_get(compass_cal_compass_sample_t *sample)
{
    vector3f_t tmp = {
    COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample->x),
    COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample->y),
    COMPASS_CAL_SAMPLE_SCALE_TO_FLOAT(sample->z)
};
    return tmp;
}

__attribute__((unused))static void compass_sample_set(const vector3f_t *in, compass_cal_compass_sample_t *sample)
{
    sample->x = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in->x);
    sample->y = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in->y);
    sample->z = COMPASS_CAL_SAMPLE_SCALE_TO_FIXED(in->z);
}

static float calc_residual(const vector3f_t *sample, const compass_cal_param_t *params)
{
    matrix3f_t softiron = {{params->diag.x, params->offdiag.x, params->offdiag.y},
                           {params->offdiag.x, params->diag.y, params->offdiag.z},
                           {params->offdiag.y, params->offdiag.z, params->diag.z}};
    vector3f_t tmp = v3f_add(sample, &params->offset);
    tmp = m3f_multi_v(&softiron, &tmp);
    float len_tmp = v3f_length(&tmp);
    return params->radius - len_tmp;
}

static float calc_mean_squared_residuals(const compass_cal_param_t *params)
{
    if (sample_buffer == NULL || samples_collected == 0) {
        return 1.0e30f;
    }
    float sum = 0;
    for (uint16_t i = 0; i < samples_collected; i++) {
        vector3f_t sample = compass_sample_get(&sample_buffer[i]);
        float resid = calc_residual(&sample, params);
        sum += sq(resid);
    }
    sum /= samples_collected;
    return sum;
}

static void initialize_fit(void)
{
    if (samples_collected != 0) {
        fitness = calc_mean_squared_residuals(&params);
    } else {
        fitness = 1.0e30f;
    }
    initial_fitness = fitness;
    sphere_lambda = 1.0f;
    ellipsoid_lambda = 1.0f;
    fit_step = 0;
}

static void reset_state(void)
{
    samples_collected = 0;
    samples_thinned = 0;
    params.radius = 200;
    vector3f_t tmp = {0, 0, 0};
    params.offset = tmp;
    vector3f_t tmp2 = {1, 1, 1};
    params.diag = tmp2;
    params.offdiag = tmp;
    params.scale_factor = 0;
    memset(completion_mask, 0, sizeof(completion_mask));
    initialize_fit();
}
static bool accept_sample(const vector3f_t *sample, uint16_t skip_index)
{
    if (sample_buffer == NULL) {
        return false;
    }
    //MY_LOG("theta: %f, radius: %f\n", theta, params.radius);
    float min_distance = params.radius * 2 * sinf(theta / 2);
    //MY_LOG("min_distance: %f\n", min_distance);
    for (uint16_t i = 0; i < samples_collected; i++) {
        //MY_LOG("samples collected:%d\n", samples_collected);
        if (i != skip_index) {
            vector3f_t tmp = compass_sample_get(&sample_buffer[i]);
            tmp = v3f_sub(sample, &tmp);
            float distance = v3f_length(&tmp);
            if (distance < min_distance) {
                return false;
            }
        }
    }
    return true;
}

static void update_completion_mask(vector3f_t *v)
{
    matrix3f_t softiron  ={{params.diag.x, params.offdiag.x, params.offdiag.y},
                           {params.offdiag.x, params.diag.y, params.offdiag.z},
                           {params.offdiag.y, params.offdiag.z, params.diag.z}};
    vector3f_t tmp = v3f_add(v, &params.offset);
    vector3f_t corrected = m3f_multi_v(&softiron, &tmp);
    int section = geodesic_grid_section(&corrected, true);
    //MY_LOG("section: %d\n", section);
    if (section < 0) {
        return;
    }
    completion_mask[section / 8] |= 1<<(section % 8);
}


static void update_completion_masks(void)
{
    memset(completion_mask, 0, sizeof(completion_mask));
    for (int i = 0; i < samples_collected; i++) {
        vector3f_t tmp = compass_sample_get(&sample_buffer[i]);
        update_completion_mask(&tmp);
    }
}

static void thin_samples(void)
{
    if (sample_buffer == NULL) {
        return;
    }
    samples_thinned = 0;
    for (uint16_t i = samples_collected - 1; i >= 1; i--) {
        uint16_t j = get_random16() %(i+1);
        compass_cal_compass_sample_t temp = sample_buffer[i];
        sample_buffer[i] = sample_buffer[j];
        sample_buffer[j] = temp;
    }
    for (uint16_t i = 0; i < samples_collected; i++) {
        vector3f_t tmp = compass_sample_get(&sample_buffer[i]);
        if(!accept_sample(&tmp, i)) {
            sample_buffer[i] = sample_buffer[samples_collected - 1];
            samples_collected--;
            samples_thinned++;
        }
    }
    update_completion_masks();
}

bool compass_calibrator_set_status(compass_cal_status_t status_tmp)
{
    if (status_tmp != COMPASS_CAL_NOT_STARTED && status == status_tmp) {
        return true;
    }
    switch (status_tmp) {
        case COMPASS_CAL_NOT_STARTED:
            reset_state();
            status = COMPASS_CAL_NOT_STARTED;
            if (sample_buffer != NULL) {
                free(sample_buffer);
                sample_buffer = NULL;
            }
            return true;
        case COMPASS_CAL_WAITING_TO_START:
            reset_state();
            status = COMPASS_CAL_WAITING_TO_START;
            compass_calibrator_set_status(COMPASS_CAL_RUNNING_STEP_ONE);
            return true;
        case COMPASS_CAL_RUNNING_STEP_ONE:
            //MY_LOG("status change to STEP ONE\n");
            if (status != COMPASS_CAL_WAITING_TO_START) {
                return false;
            }
            if (attempt == 1 && (xtimer_now().ticks32 / 1000 - start_time_ms) * 1.0e-3f <
            delay_start_sec) {
                return false;
            }
            if (sample_buffer == NULL) {
                //MY_LOG("create compass cal sample buffer\n");;
                sample_buffer = (compass_cal_compass_sample_t *)calloc(COMPASS_CAL_NUM_SAMPLES,
                                                                       sizeof(compass_cal_compass_sample_t));
            }
            if (sample_buffer != NULL) {
                initialize_fit();
                status = COMPASS_CAL_RUNNING_STEP_ONE;
                return true;
            }
            return false;
        case COMPASS_CAL_RUNNING_STEP_TWO:
            //MY_LOG("***status change to STEP TWO\n");
            if (status != COMPASS_CAL_RUNNING_STEP_ONE) {
                return false;
            }
            thin_samples();
            /* MY_LOG("samples collected: %d, samples thinned: %d\n", samples_collected, */
            /*        samples_thinned); */
            initialize_fit();
            status = COMPASS_CAL_RUNNING_STEP_TWO;
            return true;
        case COMPASS_CAL_SUCCESS:
            if (status != COMPASS_CAL_RUNNING_STEP_TWO) {
                return false;
            }
            if (sample_buffer != NULL) {
                free(sample_buffer);
                sample_buffer = NULL;
            }
            status = COMPASS_CAL_SUCCESS;
            return true;
        case COMPASS_CAL_FAILED:
            if (status == COMPASS_CAL_BAD_ORIENTATION ||
            status == COMPASS_CAL_BAD_RADIUS) {
                return false;
            }
            __attribute__((fallthrough));
        case COMPASS_CAL_BAD_ORIENTATION:
        case COMPASS_CAL_BAD_RADIUS:
            if (status == COMPASS_CAL_NOT_STARTED) {
                return false;
            }
            if (retry && compass_calibrator_set_status(COMPASS_CAL_WAITING_TO_START)) {
                attempt++;
                return true;
            }
            if (sample_buffer != NULL) {
                free(sample_buffer);
                sample_buffer = NULL;
            }
            status = status_tmp;
            return true;
        default:
            return false;
    }
}

void compass_calibrator_set_orientation(Rotation_t orientation, bool fix_orientation,
                                        bool always_45_deg)
{
    mutex_lock(&state_sem);
    cal_settings.check_orientation = true;
    cal_settings.orientation = orientation;
    cal_settings.orig_orientation = orientation;
    cal_settings.is_external = false;
    cal_settings.fix_orientation = fix_orientation;
    cal_settings.always_45_deg = always_45_deg;
    mutex_unlock(&state_sem);
}

__attribute__((unused))static void set_from_ahrs(__attribute__((unused))compass_cal_attitude_sample_t *att)
{
    const matrix3f_t *dcm = ahrs_get_dcm_rotation_body_to_ned();
    float roll_rad, pitch_rad, yaw_rad;
    m3f_to_euler(dcm, &roll_rad, &pitch_rad, &yaw_rad);
    roll = (int16_t)constrain_value(127 * (roll_rad / M_PI), -127, 127);
    pitch = (int16_t)constrain_value(127 * (pitch_rad / M_PI_2), -127, 127);
    yaw = (int16_t)constrain_value(127 * (yaw_rad / M_PI), -127, 127);
}

void compass_calibrator_new_sample(__attribute__((unused))const vector3f_t *sample)
{
    mutex_lock(&sample_sem);
    compass_sample_set(sample, &last_sample);
    set_from_ahrs(&last_sample.att);
    new_sample = true;
    mutex_unlock(&sample_sem);
}

bool compass_calibrator_running(void)
{
    mutex_lock(&state_sem);
    bool ret =  (cal_state.status == COMPASS_CAL_RUNNING_STEP_ONE ||
        cal_state.status == COMPASS_CAL_RUNNING_STEP_TWO);
    mutex_unlock(&state_sem);
    return ret;
}

static bool running(void)
{
    return status == COMPASS_CAL_RUNNING_STEP_ONE || status == COMPASS_CAL_RUNNING_STEP_TWO;
}

static bool fitting(void)
{
    return running() && (samples_collected == COMPASS_CAL_NUM_SAMPLES);
}

void compass_calibrator_start(bool retry, float delay, uint16_t offset_max, float tolerance)
{
    mutex_lock(&state_sem);
    if (running()) {
        mutex_unlock(&state_sem);
        return;
    }
    cal_settings.offset_max = offset_max;
    cal_settings.attempt = 1;
    cal_settings.retry = retry;
    cal_settings.delay_start_sec = delay;
    cal_settings.start_time_ms = xtimer_now().ticks32 / 1000;
    cal_settings.compass_idx = 0;
    cal_settings.tolerance = tolerance;
    requested_status = COMPASS_CAL_WAITING_TO_START;
    status_set_requested = true;
    mutex_unlock(&state_sem);
}

__attribute__((unused))static void pull_sample(void)
{
    compass_cal_compass_sample_t mag_sample;
    {
        mutex_lock(&sample_sem);
        if (!new_sample) {
            mutex_unlock(&sample_sem);
            return;
        }
        if (status == COMPASS_CAL_WAITING_TO_START) {
            compass_calibrator_set_status(COMPASS_CAL_RUNNING_STEP_ONE);
        }
        new_sample = false;
        mag_sample = last_sample;
        mutex_unlock(&sample_sem);
    }
    vector3f_t tmp = compass_sample_get(&mag_sample);
    if (running() && samples_collected < COMPASS_CAL_NUM_SAMPLES &&
        accept_sample(&tmp, UINT16_MAX)) {
        /* MY_LOG("softiron:\n%f %f %f\n%f %f %f\n%f %f %f\n", */
        /*        params.diag.x, params.offdiag.x, params.offdiag.y, */
        /*        params.offdiag.x, params.diag.y, params.offdiag.z, */
        /*        params.offdiag.y, params.offdiag.z, params.diag.z); */
        update_completion_mask(&tmp);
        sample_buffer[samples_collected] = mag_sample;
        samples_collected++;
        //MY_LOG("samples collected: %d\n", samples_collected);
    }
}

static void update_cal_settings(void)
{
    tolerance = cal_settings.tolerance;
    check_orientation = cal_settings.check_orientation;
    orientation = cal_settings.orientation;
    orig_orientation = cal_settings.orig_orientation;
    is_external = cal_settings.is_external;
    fix_orientation = cal_settings.fix_orientation;
    offset_max = cal_settings.offset_max;
    attempt = cal_settings.attempt;
    retry = cal_settings.retry;
    delay_start_sec = cal_settings.delay_start_sec;
    start_time_ms = cal_settings.start_time_ms;
    compass_idx = cal_settings.compass_idx;
    always_45_deg = cal_settings.always_45_deg;
}

static void update_cal_status(void)
{
    cal_state.status = status;
    cal_state.attempt = attempt;
    memcpy(cal_state.completion_mask, completion_mask, sizeof(completion_mask_t));
    cal_state.completion_pct = 0.0f;
    switch (status) {
        case COMPASS_CAL_NOT_STARTED:
        case COMPASS_CAL_WAITING_TO_START:
            cal_state.completion_pct = 0.0f;
            break;
        case COMPASS_CAL_RUNNING_STEP_ONE:
            cal_state.completion_pct = 33.3f * samples_collected / COMPASS_CAL_NUM_SAMPLES;
            break;
        case COMPASS_CAL_RUNNING_STEP_TWO:
            cal_state.completion_pct = 33.3f + 65.7f *
                ((float)(samples_collected - samples_thinned) / (COMPASS_CAL_NUM_SAMPLES - samples_thinned));
            break;
        case COMPASS_CAL_SUCCESS:
            cal_state.completion_pct = 100.0f;
            break;
        case COMPASS_CAL_FAILED:
        case COMPASS_CAL_BAD_ORIENTATION:
        case COMPASS_CAL_BAD_RADIUS:
            cal_state.completion_pct = 0.0f;
            break;
    }
}

static void update_cal_report(void)
{
    cal_report.status = status;
    cal_report.fitness = sqrtf(fitness);
    cal_report.ofs = params.offset;
    cal_report.diag = params.diag;
    cal_report.offdiag = params.offdiag;
    cal_report.scale_factor = params.scale_factor;
    cal_report.orientation_confidence = orientation_confidence;
    cal_report.original_orientation = orig_orientation;
    cal_report.orientation = orientation_solution;
    cal_report.check_orientation = check_orientation;
}

static void calc_initial_offset(void)
{
    v3f_zero(&params.offset);
    for (uint16_t k = 0; k < samples_collected; k++) {
        vector3f_t tmp = compass_sample_get(&sample_buffer[k]);
        params.offset = v3f_sub(&params.offset, &tmp);
    }
    params.offset = v3f_div(&params.offset, samples_collected);
}

static void calc_sphere_jacob(const vector3f_t *sample, const compass_cal_param_t *params,
                              float *ret)
{
    const vector3f_t *offset = &params->offset;
    const vector3f_t *diag = &params->diag;
    const vector3f_t *offdiag = &params->offdiag;
    matrix3f_t softiron = {{diag->x, offdiag->x, offdiag->y},
                           {offdiag->x, diag->y, offdiag->z},
                           {offdiag->y, offdiag->z, diag->z}};
    float A = (diag->x * (sample->x + offset->x)) +
        (offdiag->x * (sample->y + offset->y)) +
        (offdiag->y * (sample->z + offset->z));
    float B = (offdiag->x * (sample->x + offset->x)) +
        (diag->y * (sample->y + offset->y)) +
        (offdiag->z * (sample->z + offset->z));
    float C = (offdiag->y * (sample->x + offset->x)) +
        (offdiag->z * (sample->y + offset->y)) +
        (diag->z * (sample->z + offset->z));
    vector3f_t tmp = v3f_add(sample, offset);
    tmp = m3f_multi_v(&softiron, &tmp);
    float length = v3f_length(&tmp);
    ret[0] = 1.0f;
    ret[1] = -1.0f * (((diag->x * A) + (offdiag->x * B) + (offdiag->y * C)) / length);
    ret[2] = -1.0f * (((offdiag->x * A) + (diag->y * B) + (offdiag->z * C)) / length);
    ret[3] = -1.0f * (((offdiag->y * A) + (offdiag->z * B) + (diag->z * C)) / length);
}

__attribute__((unused))static void run_sphere_fit(void)
{
    MY_LOG("fit = %f\n", fitness);
    if (sample_buffer == NULL) {
        MY_LOG("run sphere fit: sample buffer = null\n");
        return;
    }
    const float lma_damping = 10.0f;
    float fitnesst = fitness;
    float fit1, fit2;
    compass_cal_param_t fit1_params, fit2_params;
    fit1_params = fit2_params = params;
    float JTJ[COMPASS_CAL_NUM_SPHERE_PARAMS * COMPASS_CAL_NUM_SPHERE_PARAMS] = {};
    float JTJ2[COMPASS_CAL_NUM_SPHERE_PARAMS * COMPASS_CAL_NUM_SPHERE_PARAMS] = {};
    float JTFI[COMPASS_CAL_NUM_SPHERE_PARAMS] = {};
    for (uint16_t k = 0; k < samples_collected; k++) {
        vector3f_t sample = compass_sample_get(&sample_buffer[k]);
        float sphere_jacob[COMPASS_CAL_NUM_SPHERE_PARAMS];
        calc_sphere_jacob(&sample, &fit1_params, sphere_jacob);
        for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_SPHERE_PARAMS; j++) {
                JTJ[i * COMPASS_CAL_NUM_SPHERE_PARAMS + j] += sphere_jacob[i] * sphere_jacob[j];
                JTJ2[i*COMPASS_CAL_NUM_SPHERE_PARAMS+j] += sphere_jacob[i] * sphere_jacob[j];
            }
            JTFI[i] += sphere_jacob[i] * calc_residual(&sample, &fit1_params);
        }
    }
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_SPHERE_PARAMS; i++) {
        JTJ[i*COMPASS_CAL_NUM_SPHERE_PARAMS+i] += sphere_lambda;
        JTJ2[i*COMPASS_CAL_NUM_SPHERE_PARAMS+i] += sphere_lambda / lma_damping;
    }
    if (!mat_inverse(JTJ, JTJ, 4)) {
        return;
    }
    if (!mat_inverse(JTJ2, JTJ2, 4)) {
        return;
    }
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_SPHERE_PARAMS; row++) {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_SPHERE_PARAMS; col++) {
            (&fit1_params.radius)[row] -= JTFI[col] * JTJ[row * COMPASS_CAL_NUM_SPHERE_PARAMS + col];
            (&fit2_params.radius)[row] -= JTFI[col] * JTJ2[row*COMPASS_CAL_NUM_SPHERE_PARAMS+col];
        }
    }
    fit1 = calc_mean_squared_residuals(&fit1_params);
    fit2 = calc_mean_squared_residuals(&fit2_params);
    if (fit1 > fitness && fit2 > fitness) {
        sphere_lambda *= lma_damping;
    } else if (fit2 < fitness && fit2 < fit1) {
        sphere_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitnesst = fit2;
    } else if (fit1 < fitness) {
        fitnesst = fit1;
    }
    if (!isnan(fitnesst) && fitnesst < fitness) {
        //MY_LOG("new sphere fit %f < old finess %f\n", fitnesst, fitness);
        fitness = fitnesst;
        params = fit1_params;
        update_completion_masks();
    }
}

static bool fit_acceptable(void)
{
    if (!isnan(fitness) &&
        params.radius > FIELD_RADIUS_MIN && params.radius < FIELD_RADIUS_MAX &&
        fabsf(params.offset.x) < offset_max && fabsf(params.offset.y) < offset_max &&
        fabsf(params.offset.z) < offset_max && params.diag.x > 0.2f &&
        params.diag.x < 5.0f && params.diag.y > 0.2f && params.diag.y < 5.0f &&
        params.diag.z > 0.2f && params.diag.z < 5.0f &&
        fabsf(params.offdiag.x) < 1.0f && fabsf(params.offdiag.y) < 1.0f &&
        fabsf(params.offdiag.z) < 1.0f) {
        return fitness <= sq(tolerance);
    }
    return false;
}

static bool fix_radius(void)
{
    if (gps_status() < GPS_OK_FIX_2D) {
        params.scale_factor = 0;
        return true;
    }
    const location_t *loc = gps_location();
    /* MY_LOG("gps location: %ld, %ld %ld\n", loc->lng, loc->lat, loc->alt); */
    /* location_t loc_tmp = {10, 302061700, 1201440700}; */
    /* const location_t *loc = &loc_tmp; */
    float intensity;
    float declination;
    float inclination;
    get_mag_field_ef(loc->lat * 1e-7f, loc->lng * 1e-7f, &intensity, &declination, &inclination);
    float expected_radius = intensity * 1000;
    float correction = expected_radius / params.radius;
    if (correction > COMPASS_MAX_SCALE_FACTOR || correction < COMPASS_MIN_SCALE_FACTOR) {
        MY_LOG("Mag bad radius %.0f expected %.0f\n", params.radius, expected_radius);
        compass_calibrator_set_status(COMPASS_CAL_BAD_RADIUS);
        return false;
    }
    params.scale_factor = correction;
    MY_LOG("mag scale factor: %f, expected %.2f\n", params.scale_factor, expected_radius);
    return true;
}

/* static bool calculate_orientation(void) */
/* { */
/*     if (!check_orientation) { */
/*         return true; */
/*     } */
/* } */

__attribute__((unused))static void calc_ellipsoid_jacob(const vector3f_t *sample,
                                 const compass_cal_param_t *params,
                                 float *ret)
{
    const vector3f_t *offset = &params->offset;
    const vector3f_t *diag = &params->diag;
    const vector3f_t *offdiag = &params->offdiag;
    matrix3f_t softiron = {{diag->x, offdiag->x, offdiag->y},
                           {offdiag->x, diag->y, offdiag->z},
                           {offdiag->y, offdiag->z, diag->z}};
    float A = (diag->x * (sample->x + offset->x)) + (offdiag->x * (sample->y + offset->y)) +
        (offdiag->y * (sample->z + offset->z));
    float B = (offdiag->x * (sample->x + offset->x)) + (diag->y * (sample->y + offset->y)) +
        (offdiag->z * (sample->z + offset->z));
    float C = (offdiag->y * (sample->x + offset->x)) + (offdiag->z * (sample->y + offset->y)) +
        (diag->z * (sample->z + offset->z));
    vector3f_t tmp = v3f_add(sample, offset);
    tmp = m3f_multi_v(&softiron, &tmp);
    float length = v3f_length(&tmp);
    ret[0] = -1.0f * (((diag->x * A) + (offdiag->x * B) + (offdiag->y * C)) / length);
    ret[1] = -1.0f * (((offdiag->x * A) + (diag->x * B) + (offdiag->z * C)) / length);
    ret[2] = -1.0f * (((offdiag->y * A) + (offdiag->z * B) + (diag->z * C)) / length);
    ret[3] = -1.0f * ((sample->x + offset->x) * A) / length;
    ret[4] = -1.0f * ((sample->y + offset->y) * B) / length;
    ret[5] = -1.0f * ((sample->z + offset->z) * C) / length;
    ret[6] = -1.0f * (((sample->y + offset->y) * A) + ((sample->x + offset->x) * B)) / length;
    ret[7] = -1.0f * (((sample->z + offset->z) * A) + ((sample->x + offset->x) * C)) / length;
    ret[8] = -1.0f * (((sample->z + offset->z) * B) + ((sample->y + offset->y) * C)) / length;
}

__attribute__((unused))static void run_ellipsoid_fit(void)
{
    if (sample_buffer == NULL) {
        return;
    }
    __attribute__((unused))const float lma_damping = 10.0f;
    __attribute__((unused))float fitnesst = fitness;
    __attribute__((unused))float fit1, fit2;
    __attribute__((unused))compass_cal_param_t fit1_params, fit2_params;
    fit1_params = fit2_params = params;
    __attribute__((unused))float JTJ[COMPASS_CAL_NUM_ELLIPSOID_PARAMS * COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = {};
    __attribute__((unused))float JTJ2[COMPASS_CAL_NUM_ELLIPSOID_PARAMS * COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = {};
    __attribute__((unused))float JTFI[COMPASS_CAL_NUM_ELLIPSOID_PARAMS] = {};
    for (uint16_t k = 0; k < samples_collected; k++) {
        vector3f_t sample = compass_sample_get(&sample_buffer[k]);
        float ellipsoid_jacob[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];
        calc_ellipsoid_jacob(&sample, &fit1_params, ellipsoid_jacob);
        for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
            for (uint8_t j = 0; j < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; j++) {
                JTJ[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
                JTJ2[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
            }
            JTFI[i] += ellipsoid_jacob[i] * calc_residual(&sample, &fit1_params);
        }
    }
    for (uint8_t i = 0; i < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; i++) {
        JTJ[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+i] += ellipsoid_lambda;
        JTJ2[i*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+i] += ellipsoid_lambda / lma_damping;
    }
    /* for (uint8_t i = 0; i < 9; i++) { */
    /*     for (uint8_t j = 0; j < 9; j++) { */
    /*         MY_LOG(" %.f", JTJ[i*9+j]); */
    /*     } */
    /*     MY_LOG("\n"); */
    /* } */
    if (!mat_inverse(JTJ, JTJ, 9)) {
        return;
    }
    /* MY_LOG("after inverse:\n"); */
    /* for (uint8_t i = 0; i < 9; i++) { */
    /*     for (uint8_t j = 0; j < 9; j++) { */
    /*         MY_LOG(" %f", JTJ[i*9+j]); */
    /*     } */
    /*     MY_LOG("\n"); */
    /* } */
    /* MY_LOG("\n"); */
    if (!mat_inverse(JTJ2, JTJ2, 9)) {
        return;
    }
    for (uint8_t row = 0; row < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; row++) {
        for (uint8_t col = 0; col < COMPASS_CAL_NUM_ELLIPSOID_PARAMS; col++) {
            ((float *)(&fit1_params.offset.x))[row] -= JTFI[col] * JTJ[row*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+col];
            ((float *)(&fit2_params.offset.x))[row] -= JTFI[col] * JTJ2[row*COMPASS_CAL_NUM_ELLIPSOID_PARAMS+col];
        }
    }
    fit1 = calc_mean_squared_residuals(&fit1_params);
    fit2 = calc_mean_squared_residuals(&fit2_params);
    if (fit1 > fitness && fit2 > fitness) {
        ellipsoid_lambda *= lma_damping;
    } else if (fit2 < fitness && fit2 < fit1) {
        ellipsoid_lambda /= lma_damping;
        fit1_params = fit2_params;
        fitnesst = fit2;
    } else if (fit1 < fitness) {
        fitnesst = fit1;
    }
    if (fitnesst < fitness) {
        fitness = fitnesst;
        params = fit1_params;
        update_completion_masks();
    }
}

void compass_calibrator_update(void)
{
    pull_sample();
    {
        mutex_lock(&state_sem);
        if (!running()) {
            //MY_LOG("compass cal !running\n");
            update_cal_settings();
        }
        if (status_set_requested) {
            //MY_LOG("status set requested\n");
            status_set_requested = false;
            compass_calibrator_set_status(requested_status);
        }
        update_cal_status();
        update_cal_report();
        mutex_unlock(&state_sem);
    }
    if (!fitting()) {
        return;
    }
    if (status == COMPASS_CAL_RUNNING_STEP_ONE) {
        led_on(LED_3);
        //MY_LOG("STEP ONE: fit step: %d\n", fit_step);
        if (fit_step >= 10) {
            if (float_is_equal(fitness, initial_fitness) || isnan(fitness)) {
                //MY_LOG("compass calibrator step one failed\n");
                compass_calibrator_set_status(COMPASS_CAL_FAILED);
            } else {
                //MY_LOG("STEP ONE: set compass cal status STEP TWO\n");
                compass_calibrator_set_status(COMPASS_CAL_RUNNING_STEP_TWO);
            }
        } else {
            if (fit_step == 0) {
                calc_initial_offset();
                //MY_LOG("STEP ONE: offset: %f %f %f\n", params.offset.x, params.offset.y, params.offset.z);
            }
            run_sphere_fit();
            fit_step++;
        }
    } else if (status == COMPASS_CAL_RUNNING_STEP_TWO) {
        led_on(LED_2);
        //MY_LOG("STEP TWO:fit step: %d\n", fit_step);
        if (fit_step >= 35) {
            //MY_LOG("fitness: %f\n", fitness);
            if (fit_acceptable() && fix_radius() /* && calculate_orientation() */) {
                compass_calibrator_set_status(COMPASS_CAL_SUCCESS);
                led_on(LED_1);
            } else {
                MY_LOG("fit not acceptable\n");
                led_off(LED_1); led_off(LED_2); led_off(LED_3);
                compass_calibrator_set_status(COMPASS_CAL_FAILED);
            }
        } else if (fit_step < 15) {
            run_sphere_fit();
            fit_step++;
        } else {
            run_ellipsoid_fit();
            fit_step++;
        }
    }
}

bool compass_calibrator_failed(void)
{
    bool ret = false;
    mutex_lock(&state_sem);
    ret = (cal_state.status == COMPASS_CAL_FAILED ||
           cal_state.status == COMPASS_CAL_BAD_ORIENTATION ||
           cal_state.status == COMPASS_CAL_BAD_RADIUS);
    mutex_unlock(&state_sem);
    return ret;
}

compass_report_t compass_calibrator_get_report(void)
{
    mutex_lock(&state_sem);
    compass_report_t tmp = cal_report;
    mutex_unlock(&state_sem);
    return tmp;
}

void compass_calibrator_stop(void)
{
    mutex_lock(&state_sem);
    requested_status = COMPASS_CAL_NOT_STARTED;
    status_set_requested = true;
    mutex_unlock(&state_sem);
}
