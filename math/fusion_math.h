#ifndef FUSION_MATH_H_
#define FUSION_MATH_H_
#include <stdbool.h>
#include <stdint.h>
#include <float.h>

#include "vector3f.h"
#include "definitions.h"

#ifndef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif

#ifndef MIN
#define MIN(x,y) ((x)<(y)?(x):(y))
#endif

float constrain_float(float amt, float low, float high);

bool float_is_zero(const float f);
//float radians(float deg);

uint8_t constrain_value(const uint8_t amt, const uint8_t low, const uint8_t high);

bool float_is_positive(const float f);
float calc_lowpass_alpha_dt(float dt, float cutoff_freq);
float norm_2f(const float first, const float second);
float norm_3f(const float first, const float second, const float third);
float safe_asin(const float v);

float sq(const float v);

float sq_3f(const float first, const float second, const float third);
float wrap_PI(const float radian);
float wrap_2PI(const float radian);
float wrap_360(const float angle);

static inline float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

static inline float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

#endif // FUSION_MATH_H_
