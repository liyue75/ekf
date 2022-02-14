#ifndef LOWPASS_FILTER_H_
#define LOWPASS_FILTER_H_

typedef struct lpf {
    float alpha;
    float cutoff_freq;
    float output;
} lpf_t;

void init_lpf(lpf_t * lpf);

float lowpass_filter_apply(const float sample, lpf_t *lpf);

void set_cutoff_frequency(float sample_freq, float cutoff_freq, lpf_t *lpf);

void reset(float value, lpf_t *lpf);

#endif // LOWPASS_FILTER_H_
