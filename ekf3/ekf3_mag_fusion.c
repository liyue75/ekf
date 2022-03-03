#include <math.h>
#include "fusion_math.h"
#include "ekf3_core.h"
#include "quaternion.h"

extern bool _mag_field_learned;
extern state_var_t _s;
extern bool _inhibit_mag_states;
extern Matrix24 _P;
extern Vector28 _Kfusion;

extern bool _inhibit_del_ang_bias_states;
extern bool _inhibit_del_vel_bias_states;
extern bool _dvel_bias_axis_inhibit[];
extern bool _inhibit_mag_states;
extern bool _inhibit_wind_states;
extern uint8_t _state_index_lim;
extern Matrix24 _KH;
extern Matrix24 _KHP;
extern fault_status_t _fault_status;

static void fuse_declination(float dec_err)
{
    const float R_DECL = sq(dec_err);
    float mag_n = _s.state_struct.earth_magfield.x;
    float mag_e = _s.state_struct.earth_magfield.y;
    if (mag_n < 1e-3f) {
        return;
    }
    float t2 = mag_e * mag_e;
    float t3 = mag_n * mag_n;
    float t4 = t2 + t3;
    if (t4 < 1e-4f) {
        return;
    }
    float t5 = _P[16][16] * t2;
    float t6 = _P[17][17] * t3;
    float t7 = t2 * t2;
    float t8 = R_DECL * t7;
    float t9 = t3 * t3;
    float t10 = R_DECL * t9;
    float t11 = R_DECL * t2 * t3 * 2.0f;
    float t14 = _P[16][17] * mag_e * mag_n;
    float t15 = _P[17][16] * mag_e * mag_n;
    float t12 = t5 + t6 + t8 + t10 + t11 - t14 - t15;
    float t13;
    if (fabsf(t12) > 1e-6f) {
        t13 = 1.0f / t12;
    } else {
        return;
    }
    float t18 = mag_e * mag_e;
    float t19 = mag_n * mag_n;
    float t20 = t18 + t19;
    float t21;
    if (fabsf(t20) > 1e-6f) {
        t21 = 1.0f / t20;
    } else {
        return;
    }
    float H_DECL[24] = {};
    H_DECL[16] = -mag_e * t21;
    H_DECL[17] = mag_n * t21;
    _Kfusion[0] = -t4 * t13 * (_P[0][16] * mag_e - _P[0][17] * mag_n);
    _Kfusion[1] = -t4 * t13 * (_P[1][16] * mag_e - _P[1][17] * mag_n);
    _Kfusion[2] = -t4 * t13 * (_P[2][16] * mag_e - _P[2][17] * mag_n);
    _Kfusion[3] = -t4 * t13 * (_P[3][16] * mag_e - _P[3][17] * mag_n);
    _Kfusion[4] = -t4 * t13 * (_P[4][16] * mag_e - _P[4][17] * mag_n);
    _Kfusion[5] = -t4 * t13 * (_P[5][16] * mag_e - _P[5][17] * mag_n);
    _Kfusion[6] = -t4 * t13 * (_P[6][16] * mag_e - _P[6][17] * mag_n);
    _Kfusion[7] = -t4 * t13 * (_P[7][16] * mag_e - _P[7][17] * mag_n);
    _Kfusion[8] = -t4 * t13 * (_P[8][16] * mag_e - _P[8][17] * mag_n);
    _Kfusion[9] = -t4 * t13 * (_P[9][16] * mag_e - _P[9][17] * mag_n);

    if (!_inhibit_del_ang_bias_states) {
        _Kfusion[10] = -t4 * t13 * (_P[10][16] * mag_e - _P[10][17] * mag_n);
        _Kfusion[11] = -t4 * t13 * (_P[11][16] * mag_e - _P[11][17] * mag_n);
        _Kfusion[12] = -t4 * t13 * (_P[12][16] * mag_e - _P[12][17] * mag_n);
    } else {
        ekf3_core_zero_range(&_Kfusion[0], 10, 12);
    }
    if (!_inhibit_del_vel_bias_states) {
        for (uint8_t index = 0; index < 3; index++) {
            const uint8_t state_index = index + 13;
            if (!_dvel_bias_axis_inhibit[index]) {
                _Kfusion[state_index] = -t4 * t13 * (_P[state_index][16] * mag_e -
                _P[state_index][17] * mag_n);
            } else {
                _Kfusion[state_index] = 0.0f;
            }
        }
    } else {
        ekf3_core_zero_range(&_Kfusion[0], 13, 15);
    }
    if (!_inhibit_mag_states) {
        _Kfusion[16] = -t4 * t13 * (_P[16][16] * mag_e - _P[16][17] * mag_n);
        _Kfusion[17] = -t4 * t13 * (_P[17][16] * mag_e - _P[17][17] * mag_n);
        _Kfusion[18] = -t4 * t13 * (_P[18][16] * mag_e - _P[18][17] * mag_n);
        _Kfusion[19] = -t4 * t13 * (_P[19][16] * mag_e - _P[19][17] * mag_n);
        _Kfusion[20] = -t4 * t13 * (_P[20][16] * mag_e - _P[20][17] * mag_n);
        _Kfusion[21] = -t4 * t13 * (_P[21][16] * mag_e - _P[21][17] * mag_n);
    } else {
        ekf3_core_zero_range(&_Kfusion[0], 16, 21);
    }
    if (!_inhibit_wind_states) {
        _Kfusion[22] = -t4 * t13 * (_P[22][16] * mag_e - _P[22][17] * mag_n);
        _Kfusion[23] = -t4 * t13 * (_P[23][16] * mag_e - _P[23][17] * mag_n);
    } else {
        ekf3_core_zero_range(&_Kfusion[0], 22, 23);
    }
    float mag_dec_ang = ekf3_core_mag_declination();
    float innovation = atan2f(mag_e, mag_n) - mag_dec_ang;
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }
    for (unsigned i = 0; i <= _state_index_lim; i++) {
        for (unsigned j = 0; j <= 15; j++) {
            _KH[i][j] = 0.0f;
        }
        _KH[i][16] = _Kfusion[i] * H_DECL[16];
        _KH[i][17] = _Kfusion[i] * H_DECL[17];
        for (unsigned j = 18; j <= 23; j++) {
            _KH[i][j] = 0.0f;
        }
    }
    for (unsigned j = 0; j <= _state_index_lim; j++) {
        for (unsigned i =0; i <= _state_index_lim ; i++) {
            _KHP[i][j] = _KH[i][16] * _P[16][j] + _KH[i][17] * _P[17][j];
        }
    }
    bool healthy_fusion = true;
    for (uint8_t i = 0; i <= _state_index_lim; i++) {
        if (_KHP[i][i] > _P[i][i]) {
            healthy_fusion = false;
        }
    }
    if (healthy_fusion) {
        for (uint8_t i = 0; i <= _state_index_lim; i++) {
            for (uint8_t j = 0; j <= _state_index_lim; j++) {
                _P[i][j] = _P[i][j] - _KHP[i][j];
            }
        }
        ekf3_core_force_symmetry();
        ekf3_core_constrain_variances();
        for (uint8_t j = 0; j <= _state_index_lim; j++) {
            _s.states_array[j] = _s.states_array[j] - _Kfusion[j] * innovation;
        }
        quat_normalize(&_s.state_struct.quat);
        _fault_status.bad_decl = false;
    } else {
        _fault_status.bad_decl = true;
    }
}

void ekf3_core_align_mag_state_declination(void)
{
    if (_mag_field_learned) {
        return;
    }
    float mag_dec_ang = ekf3_core_mag_declination();
    vector3f_t init_mag_ned = _s.state_struct.earth_magfield;
    float mag_length_ne = v3f_length(&init_mag_ned);
    _s.state_struct.earth_magfield.x = mag_length_ne * cosf(mag_dec_ang);
    _s.state_struct.earth_magfield.y = mag_length_ne * sinf(mag_dec_ang);
    if (_inhibit_mag_states) {
        float var_16 = _P[16][16];
        float var_17 = _P[17][17];
        ekf3_core_zero_rows(&_P, 16, 17);
        ekf3_core_zero_cols(&_P, 16, 17);
        _P[16][16] = var_16;
        _P[17][17] = var_17;
        fuse_declination(0.1f);
    }
}
