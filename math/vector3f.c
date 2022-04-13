#include <math.h>
#include <float.h>
#include "vector3f.h"
#include "uart_device.h"
#include "board_led.h"
#include "fusion_math.h"

void rotate_vec(vector3f_t *accel, Rotation_t rotation)
{
    float tmp;
    float *x = &accel->x;
    float *y = &accel->y;
    float *z = &accel->z;
    switch (rotation) {
        case ROTATION_NONE:
            return;
        case ROTATION_YAW_45:
            tmp = HALF_SQRT_2 * (*x - *y);
            accel->y = HALF_SQRT_2 * (*x + *y);
            accel->x = tmp;
            return;
        case ROTATION_YAW_90:
            tmp = *x; *x = -*y; *y = tmp;
            return;
        case ROTATION_YAW_135:
            tmp = -HALF_SQRT_2 * (*x + *y);
            *y = HALF_SQRT_2 * (*x - *y);
            *x = tmp;
            return;
        case ROTATION_YAW_180:
            *x = -*x; *y = -*y;
            return;
        case ROTATION_YAW_225:
            tmp = HALF_SQRT_2 * (*y - *x);
            *y = -HALF_SQRT_2 * (*x + *y);
            *x = tmp;
            return;
        case ROTATION_YAW_270: {
            tmp = *x; *x = *y; *y = -tmp;
            return;
        }
        case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2 * (*x + *y);
        *y = HALF_SQRT_2 * (*y - *x);
        *x = tmp;
        return;
        }
        case ROTATION_ROLL_180: {
            *y = -*y; *z = -*z;
            return;
        }
        case ROTATION_ROLL_180_YAW_45: {
            tmp = HALF_SQRT_2 * (*x + *y);
            *y   = HALF_SQRT_2 * (*x - *y);
            *x = tmp; *z = -*z;
            return;
        }
        case ROTATION_ROLL_180_YAW_90:
        case ROTATION_PITCH_180_YAW_270: {
            tmp = *x; *x = *y; *y = tmp; *z = -*z;
            return;
        }
        case ROTATION_ROLL_180_YAW_135: {
            tmp = HALF_SQRT_2 * (*y - *x);
            *y   = HALF_SQRT_2 * (*y + *x);
            *x = tmp; *z = -*z;
            return;
        }
        case ROTATION_PITCH_180: {
            *x = -*x; *z = -*z;
            return;
        }
        case ROTATION_ROLL_180_YAW_225: {
            tmp = -HALF_SQRT_2* (*x + *y);
            *y   =  HALF_SQRT_2* (*y - *x);
            *x = tmp; *z = -*z;
            return;
        }
        case ROTATION_ROLL_180_YAW_270:
        case ROTATION_PITCH_180_YAW_90: {
            tmp = *x; *x = -*y; *y = -tmp; *z = -*z;
            return;
        }
        case ROTATION_ROLL_180_YAW_315: {
            tmp =  HALF_SQRT_2* (*x - *y);
            *y   = -HALF_SQRT_2* (*x + *y);
            *x = tmp; *z = -*z;
            return;
        }
        case ROTATION_ROLL_90: {
            tmp = *z; *z = *y; *y = -tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_45: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = HALF_SQRT_2* (*x - *y);
            *y   = HALF_SQRT_2* (*x + *y);
            *x = tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_90: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = *x; *x = -*y; *y = tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_135: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = -HALF_SQRT_2* (*x + *y);
            *y   =  HALF_SQRT_2* (*x - *y);
            *x = tmp;
            return;
        }
        case ROTATION_ROLL_270: {
            tmp = *z; *z = -*y; *y = tmp;
            return;
        }
        case ROTATION_ROLL_270_YAW_45: {
            tmp = *z; *z = -*y; *y = tmp;
            tmp = HALF_SQRT_2* (*x - *y);
            *y   = HALF_SQRT_2* (*x + *y);
            *x = tmp;
            return;
        }
        case ROTATION_ROLL_270_YAW_90: {
            tmp = *z; *z = -*y; *y = tmp;
            tmp = *x; *x = -*y; *y = tmp;
            return;
        }
        case ROTATION_ROLL_270_YAW_135: {
            tmp = *z; *z = -*y; *y = tmp;
            tmp = -HALF_SQRT_2* (*x + *y);
            *y   =  HALF_SQRT_2* (*x - *y);
            *x = tmp;
            return;
        }
        case ROTATION_PITCH_90: {
            tmp = *z; *z = -*x; *x = tmp;
            return;
        }
        case ROTATION_PITCH_270: {
            tmp = *z; *z = *x; *x = -tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_90: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = *z; *z = -*x; *x = tmp;
            return;
        }
        case ROTATION_ROLL_180_PITCH_90: {
            *y = -*y; *z = -*z;
            tmp = *z; *z = -*x; *x = tmp;
            return;
        }
        case ROTATION_ROLL_270_PITCH_90: {
            tmp = *z; *z = -*y; *y = tmp;
            tmp = *z; *z = -*x; *x = tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_180: {
            tmp = *z; *z = *y; *y = -tmp;
            *x = -*x; *z = -*z;
            return;
        }
        case ROTATION_ROLL_270_PITCH_180: {
            tmp = *z; *z = -*y; *y = tmp;
            *x = -*x; *z = -*z;
            return;
        }
        case ROTATION_ROLL_90_PITCH_270: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = *z; *z = *x; *x = -tmp;
            return;
        }
        case ROTATION_ROLL_180_PITCH_270: {
            *y = -*y; *z = -*z;
            tmp = *z; *z = *x; *x = -tmp;
            return;
        }
        case ROTATION_ROLL_270_PITCH_270: {
            tmp = *z; *z = -*y; *y = tmp;
            tmp = *z; *z = *x; *x = -tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_180_YAW_90: {
            tmp = *z; *z = *y; *y = -tmp;
            *x = -*x; *z = -*z;
            tmp = *x; *x = -*y; *y = tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_270: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = *x; *x = *y; *y = -tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_68_YAW_293: {
            float tmpx = *x;
            float tmpy = *y;
            float tmpz = *z;
            *x =  0.14303897231223747232853327204793 * tmpx +  0.36877648650320382639478111741482 * tmpy + -0.91844638134308709265241077446262 * tmpz;
            *y = -0.33213277779664740485543461545603 * tmpx + -0.85628942146641884303193137384369 * tmpy + -0.39554550256296522325882847326284 * tmpz;
            *z = -0.93232380121551217122544130688766 * tmpx +  0.36162457008209242248497616856184 * tmpy +  0.00000000000000002214311861220361 * tmpz;
            return;
        }
        case ROTATION_PITCH_315: {
            tmp = HALF_SQRT_2* (*x - *z);
            *z   = HALF_SQRT_2* (*x + *z);
            *x = tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_315: {
            tmp = *z; *z = *y; *y = -tmp;
            tmp = HALF_SQRT_2* (*x - *z);
            *z   = HALF_SQRT_2* (*x + *z);
            *x = tmp;
            return;
        }
        case ROTATION_PITCH_7: {
            const float sin_pitch = 0.1218693434051474899781908334262; // sinF(pitch);
            const float cos_pitch = 0.99254615164132198312785249072476; // cosF(pitch);
            float tmpx = *x;
            float tmpz = *z;
            *x =  cos_pitch * tmpx + sin_pitch * tmpz;
            *z = -sin_pitch * tmpx + cos_pitch * tmpz;
            return;
        }
        case ROTATION_ROLL_45: {
            tmp = HALF_SQRT_2* (*y - *z);
            *z   = HALF_SQRT_2* (*y + *z);
            *y = tmp;
            return;
        }
        case ROTATION_ROLL_315: {
            tmp = HALF_SQRT_2* (*y + *z);
            *z   = HALF_SQRT_2* (*z - *y);
            *y = tmp;
            return;
        }
        case ROTATION_CUSTOM:
            // Error: caller must perform custom rotations via matri*x multiplication
            break;
        case ROTATION_MAX:
            break;
    }
    led_on(LED_1);
    MY_LOG("vector3f.c : Unkonwn roattion\n");
}

void v3f_zero(vector3f_t *accel)
{
    accel->x = accel->y = accel->z = 0;
}

float v3f_dot_product(const vector3f_t *v1, const vector3f_t *v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

vector3f_t v3f_cross_product(const vector3f_t *v1, const vector3f_t *v2)
{
    vector3f_t tmp = {
    v1->y * v2->z - v1->z * v2->y,
    v1->z * v2->x - v1->x * v2->z,
    v1->x * v2->y - v1->y * v2->x};
    return tmp;
}

vector3f_t v3f_uniform_scale(const vector3f_t *v, const float num)
{
    vector3f_t tmp = {v->x * num, v->y * num, v->z * num};
    return tmp;
}

vector3f_t v3f_div(const vector3f_t *v, const float num)
{
    vector3f_t tmp = {v->x / num, v->y / num, v->z / num};
    return tmp;
}

vector3f_t v3f_add(const vector3f_t *v1, const vector3f_t *v2)
{
    vector3f_t tmp = {v1->x + v2->x, v1->y + v2->y, v1->z + v2->z};
    return tmp;
}

vector3f_t v3f_sub(const vector3f_t *v1, const vector3f_t *v2)
{
    vector3f_t tmp = {v1->x - v2->x, v1->y - v2->y, v1->z - v2->z};
    return tmp;
}

bool v3f_isnan(const vector3f_t *v3f)
{
    return isnan(v3f->x) || isnan(v3f->y) || isnan(v3f->z);
}

bool v3f_isinf(const vector3f_t *v3f)
{
    return isinf(v3f->x) || isinf(v3f->y) || isinf(v3f->z);
}

float v3f_length(const vector3f_t *v)
{
    return norm_3f(v->x, v->y, v->z);
}

vector3f_t v3f_normalized(const vector3f_t *v)
{
    return v3f_div(v, v3f_length(v));
}

bool v3f_is_zero(const vector3f_t *v)
{
    return (fabsf(v->x) < FLT_EPSILON) && (fabsf(v->y) < FLT_EPSILON)
        && (fabsf(v->z) < FLT_EPSILON);
}

bool v3f_not_equal(const vector3f_t *v1, const vector3f_t *v2)
{
    return (!float_is_equal(v1->x, v2->x)) || (!float_is_equal(v1->y, v2->y)) || (!float_is_equal(v1->z, v2->z));
}
