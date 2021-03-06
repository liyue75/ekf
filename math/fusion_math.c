#include <math.h>
#include <float.h>
#include "fusion_math.h"
#include "definitions.h"

#define ENABLE_DEBUG 1
#include "debug.h"

float constrain_float(float amt, float low, float high)
{
    if (isnan(amt)) {
        return (low + high) / 2;
    }
    if (amt < low) {
        return low;
    }
    if (amt > high) {
        return high;
    }
    return amt;
}

inline bool float_is_zero(const float f)
{
    return fabsf(f) < FLT_EPSILON;
}

inline bool float_is_positive(const float f)
{
    return f >= FLT_EPSILON;
}

/* float radians(float deg) */
/* { */
/*     /\* */
/* #ifdef M_PI */
/*     DEBUG(" M_PI = %d", M_PI); */
/* #endif */
/* *\/ */
/*     return deg * DEG_TO_RAD; */
/* } */



uint8_t constrain_value(const uint8_t amt, const uint8_t low, const uint8_t high)
{
    if (amt < low) {
        return low;
    }
    if (amt > high) {
        return high;
    }
    return amt;
}

float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (dt <= 0.0f || cutoff_freq <= 0.0f) {
        return 1.0;
    }
    float rc = 1.0f / (M_2PI * cutoff_freq);
    return constrain_float(dt / (dt + rc), 0.0f, 1.0f);
}

float safe_asin(const float f)
{
    if (isnan(f)) {
        return 0.0f;
    }
    if (f >= 1.0f) {
        return M_PI_2;
    }
    if (f <= -1.0f) {
        return -M_PI_2;
    }
    return asinf(f);
}

float sq(const float v)
{
    return v*v;
}

float safe_sqrt(const float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}


float sq_2f(const float first, const float second)
{
    return sq(first) + sq(second);
}

float sq_3f(const float first, const float second, const float third)
{
    return sq(first) + sq(second) + sq(third);
}

float norm_3f(const float first, const float second, const float third)
{
    return sqrtf(sq_3f(first, second, third));
}

float wrap_2PI(const float radian)
{
    float res = fmodf(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

float wrap_PI(const float radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

float wrap_360(const float angle)
{
    float res = fmod(angle, 360.0);
    if (res < 0) {
        res += 360.0;
    }
    return res;
}

float wrap_180(const float angle)
{
    float res = wrap_360(angle);
    if (res > (float)(180)) {
        res -= (float)(180);
    }
    return res;
}


float norm_2f(const float first, const float second)
{
    return sqrtf(sq_2f(first, second));
}

uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

bool float_is_equal(const float v1, const float v2)
{
    return fabsf(v1 - v2) < FLT_EPSILON;
}
