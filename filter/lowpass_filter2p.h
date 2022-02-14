#ifndef LOWPASS_FILTER2P_H_
#define LOWPASS_FILTER2P_H_

//#include "arm_math.h"
#include <stdbool.h>
#include "vector3f.h"

typedef struct biquad_params {
    float cutoff_freq;
    float sample_freq;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
} biquad_params_t;

typedef struct biquad {
    biquad_params_t params;
    bool initialised;
    vector3f_t delay_element_1;
    vector3f_t delay_element_2;
} biquad_t;

void init_lpf2p_v3f(
    biquad_t *biquad_filter);

void lpf2p_v3f_reset(biquad_t *bq);
void lpf2p_v3f_reset_value(const vector3f_t *value, biquad_t *bq);

void lpf2p_set_cutoff_frequency(float sample_freq, float cutoff_freq, biquad_t *biquad_filter);

vector3f_t lpf2p_v3f_apply(const vector3f_t *sample, biquad_t *biquad_filter);

#endif // LOWPASS_FILTER2P_H_
