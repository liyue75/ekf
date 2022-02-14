#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "kernel_defines.h"
#include "xtimer.h"

#include "tempcal.h"
#include "lowpass_filter.h"
#include "fusion_math.h"
#include "polyfit.h"

#define TEMP_RANGE_MIN 10
#define CAL_TIMEOUT_MS (600U*1000U)
#define TEMP_REFERENCE 35.0

extern bool _initialized;

typedef struct learn_state {
    float last_temp;
    uint32_t last_sample_ms;
    vector3f_t sum;
    uint32_t sum_count;
    lpf_t temp_filter;
    polyfit_t pfit;
} learn_state_t;

__attribute__((unused))static float temp_min = 0;
static float temp_max = 70;
__attribute__((unused))vector3f_t accel_coeff[3] = {0};
__attribute__((unused))vector3f_t gyro_coeff[3] = {0};
__attribute__((unused))vector3f_t accel_tref;
__attribute__((unused))vector3f_t gyro_tref;

static bool learn = false;
static learn_state_t state[2];
static float start_temp;
static float start_tmax;
__attribute__((unused))static uint32_t last_save_ms;
static vector3f_t accel_start;

static void reset_cal(__attribute__((unused))float temperature)
{
    memset((void *)&state[0], 0, sizeof(state));
    start_tmax = temp_max;
    v3f_zero(&accel_start);
    for (uint8_t i = 0; i < ARRAY_SIZE(state); i++) {
        init_lpf(&state[i].temp_filter);
        set_cutoff_frequency(1000, 0.5, &state[i].temp_filter);
        reset(temperature, &state[i].temp_filter);
        state[i].last_temp = temperature;
    }
}

static void add_sample(vector3f_t *sample, float temperature, learn_state_t *st)
{
    temperature = lowpass_filter_apply(temperature, &st->temp_filter);
    st->sum.x += sample->x;
    st->sum.y += sample->y;
    st->sum.z += sample->z;
    st->sum_count++;
    uint32_t now = xtimer_now().ticks32 / 1000;
    if (st->sum_count < 100 ||
        temperature - st->last_temp < 0.5) {
        if (st->last_sample_ms != 0 &&
            temperature - start_temp >= TEMP_RANGE_MIN &&
            now - st->last_sample_ms > CAL_TIMEOUT_MS) {
            //finish_calibration(st->last_temp);
        }
        return;
    }
    st->sum.x /= st->sum_count;
    st->sum.y /= st->sum_count;
    st->sum.z /= st->sum_count;
    const float T = (temperature + st->last_temp) * 0.5;
    if (float_is_zero(accel_start.x) &&
        float_is_zero(accel_start.y) &&
        float_is_zero(accel_start.z)) {
        accel_start = st->sum;
        start_temp = T;
    }
    st->sum.x -= accel_start.x;
    st->sum.y -= accel_start.y;
    st->sum.z -= accel_start.z;
    //const float tdiff = T - TEMP_REFERENCE;
    //st->pfit.update(tdiff, st->sum);

}

void update_accel_learning(__attribute__((unused))vector3f_t *accel,
                           __attribute__((unused))float temperature)
{
    if (learn == false && _initialized) {
        start_temp = temperature;
        reset_cal(start_temp);
        learn = true;
    }
    if (learn != false) {
        add_sample(accel, temperature, &state[0]);
    }
}
