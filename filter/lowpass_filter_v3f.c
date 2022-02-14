#include "lowpass_filter_v3f.h"
#include "fusion_math.h"
#include "definitions.h"

void init_lpf_v3f(lpf_v3f_t *lpf)
{
    lpf->alpha = 1.0f;
    lpf->initialised = false;
    lpf->output.x = lpf->output.y = lpf->output.z = 0;
    lpf->cutoff_freq = 0.0f;
}

vector3f_t lpf_v3f_apply_dt(vector3f_t *sample, float dt, lpf_v3f_t *lpf)
{
    float rc = 1.0f / (M_2PI * lpf->cutoff_freq);
    lpf->alpha = constrain_float(dt/(dt + rc), 0.0f, 1.0f);
    lpf->output.x += (sample->x - lpf->output.x) * lpf->alpha;
    lpf->output.y += (sample->y - lpf->output.y) * lpf->alpha;
    lpf->output.z += (sample->z - lpf->output.z) * lpf->alpha;
    if (!lpf->initialised) {
        lpf->initialised = true;
        lpf->output = *sample;
    }
    return lpf->output;
}

vector3f_t lpf_v3f_apply(vector3f_t *sample, lpf_v3f_t *lpf)
{
    lpf->output.x += (sample->x - lpf->output.x) * lpf->alpha;
    lpf->output.y += (sample->y - lpf->output.y) * lpf->alpha;
    lpf->output.z += (sample->z - lpf->output.z) * lpf->alpha;
    if (!lpf->initialised) {
        lpf->initialised = true;
        lpf->output = *sample;
    }
    return lpf->output;
}

void lpf_v3f_compute_alpha(float sample_freq, float cutoff_freq, lpf_v3f_t *lpf)
{
    lpf->alpha = calc_lowpass_alpha_dt(1.0 / sample_freq, cutoff_freq);
}

void lpf_v3f_reset(vector3f_t *value, lpf_v3f_t *lpf)
{
    lpf->output = *value;
    lpf->initialised = true;
}

void set_cutoff_freq_v3f(float cutoff_freq, lpf_v3f_t *lpf)
{
    lpf->cutoff_freq = cutoff_freq;
}

void set_cutoff_freq_v3f_sam(float sample_freq, float cutoff_freq, lpf_v3f_t *lpf)
{
    lpf->cutoff_freq = cutoff_freq;
    lpf_v3f_compute_alpha(sample_freq, cutoff_freq, lpf);
}
