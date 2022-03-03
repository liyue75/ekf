#include <math.h>
#include "compass.h"
#include "matrix3f.h"
#include "compass_backend.h"
#include "mutex.h"
#include "xtimer.h"
#include "compass_calibrator.h"

static mutex_t compass_mutex;
extern mag_state_t _compass_state[];
extern int8_t _filter_range;
static float _mean_field_length;
static const float FILTER_KOEF = 0.1f;
static uint32_t error_count;

void compass_backend_init(void)
{
    mutex_init(&compass_mutex);
}

static void publish_raw_field(const vector3f_t *mag)
{
    _compass_state[0].last_update_ms = xtimer_now().ticks32 / 1000;
    compass_calibrate_new_sample(mag);
}

static bool field_ok(const vector3f_t *field)
{
    if (v3f_isinf(field) || v3f_isnan(field)) {
        return false;
    }
    const float range = _filter_range;
    if (range <= 0) {
        return true;
    }
    const float length = v3f_length(field);
    if (float_is_zero(length)) {
        _mean_field_length = length;
        return true;
    }
    bool ret = true;
    const float d = fabsf(_mean_field_length - length) / (_mean_field_length + length);
    float koeff = FILTER_KOEF;
    if (d * 200.0f > range) {
        ret = false;
        koeff /= (d * 10.0f);
        error_count++;
    }
    _mean_field_length = _mean_field_length * (1 - koeff) + length * koeff;
    return ret;
}

static void correct_field(vector3f_t *mag)
{
    if (v3f_is_zero(&_compass_state[0].diagonals)) {
        vector3f_t tmp = {1.0f, 1.0f, 1.0f};
        _compass_state[0].diagonals = tmp;
    }
    const vector3f_t *offsets = &_compass_state[0].offset;
    __attribute__((unused))const vector3f_t *diagonals = &_compass_state[0].diagonals;
    __attribute__((unused))const vector3f_t *offdiagonals = &_compass_state[0].offdiagonals;
    *mag = v3f_add(mag, offsets);
    if (compass_have_scale_factor()) {
        *mag = v3f_uniform_scale(mag, _compass_state[0].scale_factor);
    }
    matrix3f_t mat = {{diagonals->x, offdiagonals->x, offdiagonals->y},
                      {offdiagonals->x, diagonals->y, offdiagonals->z},
                      {offdiagonals->y, offdiagonals->z, diagonals->z}};
    *mag = m3f_multi_v(&mat, mag);
}

void compass_accumulate_sample(__attribute__((unused))vector3f_t *field,
                               __attribute__((unused))uint32_t max_samples)
{
    //rotate_field(field);
    publish_raw_field(field);
    correct_field(field);
    if (!field_ok(field)) {
        return;
    }
    {
        mutex_lock(&compass_mutex);
        _compass_state[0].accum = v3f_add(&_compass_state[0].accum, field);
        _compass_state[0].accum_count++;
        if (max_samples && _compass_state[0].accum_count >= max_samples) {
            _compass_state[0].accum_count /= 2;
            _compass_state[0].accum = v3f_uniform_scale(&_compass_state[0].accum, 0.5);
        }
        mutex_unlock(&compass_mutex);
    }
}


static void publish_filtered_field(const vector3f_t *mag)
{
    _compass_state[0].field = *mag;
    _compass_state[0].last_update_usec = xtimer_now().ticks32;
    _compass_state[0].last_update_ms = _compass_state[0].last_update_usec / 1000;
}

void compass_drain_accumlated_sample(void)
{
    mutex_lock(&compass_mutex);
    if (_compass_state[0].accum_count == 0) {
        return;
    }
    _compass_state[0].accum = v3f_uniform_scale(
        &_compass_state[0].accum, _compass_state[0].accum_count);
    publish_filtered_field(&_compass_state[0].accum);
    v3f_zero(&_compass_state[0].accum);
    _compass_state[0].accum_count = 0;
    mutex_unlock(&compass_mutex);
}
