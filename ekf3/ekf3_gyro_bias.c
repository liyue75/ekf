#include "ekf3_core.h"
#include "fusion_math.h"

extern state_var_t _s;
extern Matrix24 _P;
extern float _dt_imu_avg;

void ekf3_core_reset_gyro_bias(void)
{
    v3f_zero(&_s.state_struct.gyro_bias);
    ekf3_core_zero_rows(&_P, 10, 12);
    ekf3_core_zero_cols(&_P, 10, 12);
    _P[10][10] = sq(radians(0.5f * _dt_imu_avg));
    _P[11][11] = _P[10][10];
    _P[12][12] = _P[10][10];
}

float ekf3_core_initial_gyro_bias_uncertainty(void)
{
    return 2.5f;
}
