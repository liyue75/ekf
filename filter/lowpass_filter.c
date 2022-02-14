#include "lowpass_filter.h"
#include "fusion_math.h"
#include "definitions.h"

void init_lpf(lpf_t *lpf)
{
    lpf->alpha = 1.0f;
    lpf->output = 0;
}

void reset(float value, lpf_t *lpf)
{
    lpf->output = value;
}

float lowpass_filter_apply(const float sample, lpf_t *lpf)
{
    lpf->output += (sample - lpf->output) * lpf->alpha;
    return lpf->output;
}

static void compute_alpha(float sample_freq, float cutoff_freq, lpf_t *lpf)
{
    if (sample_freq <= 0) {
        lpf->alpha = 1;
    } else {
        lpf->alpha = calc_lowpass_alpha_dt(1.0 / sample_freq, cutoff_freq);
    }
}

void set_cutoff_frequency(float sample_freq, float cutoff_freq, lpf_t *lpf)
{
    lpf->cutoff_freq = cutoff_freq;
    compute_alpha(sample_freq, cutoff_freq, lpf);
}

