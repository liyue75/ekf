#include <string.h>
#include "lowpass_filter2p.h"
#include "fusion_math.h"
#include "uart_device.h"
#include "definitions.h"

#define ENABLE_DEBUG 1
#include "debug.h"

void init_lpf2p_v3f(biquad_t *bq_filter)
{
    vector3f_t tmp = {0, 0, 0};
    bq_filter->delay_element_1 = bq_filter->delay_element_2 = tmp;
    bq_filter->initialised = false;
    memset(&bq_filter->params, 0, sizeof(bq_filter->params));
}

void lpf2p_v3f_reset(biquad_t *bq_filter)
{
    vector3f_t tmp = {0, 0, 0};
    bq_filter->delay_element_1 = bq_filter->delay_element_2 = tmp;
    bq_filter->initialised = false;
}

void lpf2p_v3f_reset_value(const vector3f_t *value,
                            biquad_t *bq_filter)
{
    float di = 1 + bq_filter->params.a1 + bq_filter->params.a2;
    vector3f_t tmp = {value->x / di, value->y /di, value->z / di};
    bq_filter->delay_element_1 = bq_filter->delay_element_2 = tmp;
    bq_filter->initialised = true;
}

void lpf2p_set_cutoff_frequency(float sample_freq, float cutoff_freq, biquad_t *biquad_filter)
{
    biquad_filter->params.cutoff_freq = cutoff_freq;
    biquad_filter->params.sample_freq = sample_freq;
    if (!float_is_positive(biquad_filter->params.cutoff_freq)) {
        return;
    }
    /* MY_LOG("params->cutoff_freq = %d\nsample_freq = %d\n", */
    /*       (int)biquad_filter->params.cutoff_freq, */
    /*       (int)biquad_filter->params.sample_freq); */
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI / fr);
    float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
    biquad_params_t *params = &biquad_filter->params;
    params->b0 = ohm * ohm / c;
    params->b1 = 2.0f * params->b0;
    params->b2 = params->b0;
    params->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    params->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
    /* MY_LOG("b0 = %d, b1 = %d, b2 = %d, a1 = %d, a2 = %d\n", */
    /*       (int)(biquad_filter->params.b0 * 1000), */
    /*       (int)(params->b1 * 1000), */
    /*       (int)(params->b2 * 1000), */
    /*       (int)(biquad_filter->params.a1 * 1000), */
    /*       (int)(params->a2 * 1000) */
    /* ); */
}

vector3f_t lpf2p_v3f_apply(const vector3f_t *sample, biquad_t *biquad_filter)
{
    if (!float_is_positive(biquad_filter->params.cutoff_freq)) {
        return *sample;
    }
    if (float_is_zero(biquad_filter->params.sample_freq)) {
        return *sample;
    }
    if (!biquad_filter->initialised) {
        lpf2p_v3f_reset_value(sample, biquad_filter);
    }
    float a1 = biquad_filter->params.a1;
    float a2 = biquad_filter->params.a2;
    float b0 = biquad_filter->params.b0;
    float b1 = biquad_filter->params.b1;
    float b2 = biquad_filter->params.b2;
    float delay_element_0_x = sample->x - biquad_filter->delay_element_1.x * a1
        - biquad_filter->delay_element_2.x * a2;
    float delay_element_0_y = sample->y - biquad_filter->delay_element_1.y * a1
        - biquad_filter->delay_element_2.y * a2;
    float delay_element_0_z = sample->z - biquad_filter->delay_element_1.z * a1
        - biquad_filter->delay_element_2.z * a2;
    vector3f_t delay_element_0 = {delay_element_0_x, delay_element_0_y, delay_element_0_z};
    float output_x = delay_element_0.x * b0 + biquad_filter->delay_element_1.x
        * b1 + biquad_filter->delay_element_2.x * b2;
    float output_y = delay_element_0.y * b0 + biquad_filter->delay_element_1.y
        * b1 + biquad_filter->delay_element_2.y * b2;
    float output_z = delay_element_0.z * b0 + biquad_filter->delay_element_1.z
        * b1 + biquad_filter->delay_element_2.z * b2;
    vector3f_t output = {output_x, output_y, output_z};
    biquad_filter->delay_element_2 = biquad_filter->delay_element_1;
    biquad_filter->delay_element_1 = delay_element_0;
    return output;
}
