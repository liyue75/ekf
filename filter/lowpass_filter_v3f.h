#ifndef LOWPASS_FILTER_V3F_H_
#define LOWPASS_FILTER_V3F_H_

#include <stdbool.h>

#include "vector3f.h"

typedef struct lpf_v3f {
    float alpha;
    float cutoff_freq;
    bool initialised;
    vector3f_t output;
} lpf_v3f_t;

void init_lpf_v3f(lpf_v3f_t * lpf);

void set_cutoff_freq_v3f(float cutoff_freq, lpf_v3f_t *lpf);

vector3f_t lpf_v3f_apply_dt(vector3f_t *sample, float dt, lpf_v3f_t *lpf);

vector3f_t lpf_v3f_apply(vector3f_t *sample, lpf_v3f_t *lpf);

void lpf_v3f_compute_alpha(float sample_freq, float cutoff_freq, lpf_v3f_t *lpf);

void lpf_v3f_reset(vector3f_t *value, lpf_v3f_t *lpf);
void set_cutoff_freq_v3f_sam(float sample_freq, float cutoff_freq, lpf_v3f_t *lpf);
#endif // LOWPASS_FILTER_V3F_H_
