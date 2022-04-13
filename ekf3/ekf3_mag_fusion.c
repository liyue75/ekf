#include <math.h>
#include "fusion_math.h"
#include "ekf3_core.h"
#include "quaternion.h"
#include "ekf_source.h"
#include "ekf_buffer.h"
#include "uart_device.h"
#include "xtimer.h"

extern bool _mag_field_learned;
extern state_var_t _s;
extern Matrix24 _P;
extern Vector28 _Kfusion;
extern mag_state_t _mag_state;

extern gps_elements_t _gps_data_delayed;
extern float _vel_test_ratio;
extern bool _all_mag_sensors_failed;
extern bool _inhibit_del_ang_bias_states;
extern bool _inhibit_del_vel_bias_states;
extern bool _dvel_bias_axis_inhibit[];
extern bool _inhibit_mag_states;
extern bool _inhibit_wind_states;
extern uint8_t _state_index_lim;
extern Matrix24 _KH;
extern Matrix24 _KHP;
extern fault_status_t _fault_status;
extern bool _mag_fuse_performed;
extern bool _on_ground_not_moving;
extern source_yaw_t _yaw_source_last;
extern bool _yaw_source_reset;
extern matrix3f_t _prev_tnb;
extern yaw_elements_t _yaw_ang_data_static;
extern float _yaw_noise;
extern imu_elements_t _imu_data_delayed;
extern bool _mag_timeout;
extern bool _mag_health;
extern uint32_t _last_healthy_mag_time_ms;
extern uint32_t _imu_sample_time_ms;
extern const uint32_t _mag_fail_time_limit_ms;
extern bool _mag_data_to_fuse;
extern ekf_ring_buffer_t _stored_mag;
extern bool _mag_yaw_reset_request;
extern bool _gps_yaw_reset_request;
extern bool _final_inflight_yaw_init;
extern bool _final_inflight_mag_init;
extern bool _yaw_align_complete;
extern bool _in_flight;
extern mag_elements_t _mag_data_delayed;
extern quaternionf_t _prev_quat_mag_reset;
extern bool _tilt_align_complete;
extern bool _on_ground;
extern uint8_t _mag_yaw_anomally_count;
extern float _pos_down_at_takeoff;
extern float _innov_yaw;
extern float _yaw_innov_at_last_mag_reset;
extern float _pos_down_at_last_mag_reset;
extern quaternionf_t _quat_at_last_mag_reset;
extern float _tilt_error_variance;
extern float _yaw_reset_angle;
extern uint32_t _last_yaw_reset_ms;
extern bool _mag_state_reset_request;
extern bool _mag_state_init_complete;
extern bool _states_initialised;
extern yaw_elements_t _yaw_ang_data_delayed;
extern float _yaw_test_ratio;
extern int16_t _yaw_innov_gate;
extern bool _mag_field_learned;
extern vector3f_t _mag_test_ratio;
extern aiding_mode_t _pv_aiding_mode;
extern int16_t _mag_ef_limit;
extern bool _have_table_earth_field;
extern vector3f_t _earth_mag_field_var;
extern vector3f_t _body_mag_field_var;
extern vector3f_t _innov_mag;
extern const float _mag_var_rate_scale;
extern float _mag_noise;
extern vector3f_t _var_innov_mag;
extern int16_t _mag_innov_gate;

static void control_mag_yaw_reset(void)
{
    if (ekf3_core_assume_zero_sideslip() && (!_final_inflight_yaw_init || !_yaw_align_complete) &&
    _in_flight) {
        _gps_yaw_reset_request = true;
        return;
    } else {
        _gps_yaw_reset_request = false;
    }
    vector3f_t delta_rot_vec_temp;
    quaternionf_t delta_quat_temp;
    bool flight_reset_allowed = false;
    bool initial_reset_allowed = false;
    if (!_final_inflight_yaw_init) {
        //MY_LOG("! final inflight yaw init\n");
        delta_quat_temp = quat_div(&_s.state_struct.quat, &_prev_quat_mag_reset);
        _prev_quat_mag_reset = _s.state_struct.quat;
        quat_to_axis_angle(&delta_quat_temp, &delta_rot_vec_temp);
        //MY_LOG("delta rot vec temp %f %f %f\n", delta_rot_vec_temp.x, delta_rot_vec_temp.y, delta_rot_vec_temp.z);
        bool ang_rate_ok = v3f_length(&delta_rot_vec_temp) < 0.1745f;
        //MY_LOG("ang ok\n");
        initial_reset_allowed = ang_rate_ok && _tilt_align_complete;
        flight_reset_allowed = ang_rate_ok && !_on_ground;
        //MY_LOG("flight reset allowed = %d\n", flight_reset_allowed);
    }
    if (_on_ground) {
        _mag_yaw_anomally_count = 0;
    }
    bool final_reset_request = false;
    bool interim_reset_request = false;
    if (flight_reset_allowed && !ekf3_core_assume_zero_sideslip()) {
        final_reset_request = (_s.state_struct.position.z - _pos_down_at_takeoff) < -EKF3_MAG_FINAL_RESET_ALT;
        bool hgt_increasing = (_pos_down_at_last_mag_reset - _s.state_struct.position.z) > 0.5f;
        float yaw_innov_increase = fabsf(_innov_yaw) - fabsf(_yaw_innov_at_last_mag_reset);
        bool yaw_innov_increasing = yaw_innov_increase > 0.25f;
        delta_quat_temp = quat_div(&_quat_at_last_mag_reset, &_s.state_struct.quat);
        quat_to_axis_angle(&delta_quat_temp, &delta_rot_vec_temp);
        bool large_angle_change = v3f_length(&delta_rot_vec_temp) > yaw_innov_increase;
        interim_reset_request = !_final_inflight_yaw_init && !final_reset_request &&
            (_mag_yaw_anomally_count < MAG_ANOMALY_RESET_MAX) &&
            hgt_increasing &&
            yaw_innov_increasing &&
            !large_angle_change;
    }
    bool initial_reset_request = initial_reset_allowed && !_yaw_align_complete;
    //MY_LOG("initial reset request %d\n", initial_reset_request);
    _mag_yaw_reset_request = _mag_yaw_reset_request ||
        initial_reset_request || interim_reset_request || final_reset_request;
    if (_mag_yaw_reset_request && ekf3_core_use_compass()) {
        if (!_yaw_align_complete) {
            MY_LOG("EKF3 IMU0 MAG0 initial yaw_alignment complete\n");
        }
        ekf3_core_set_yaw_from_mag();
        if (final_reset_request) {
            MY_LOG("EKF3 IMU0 MAG0 in-flight yaw alignment complete\n");
        } else if (interim_reset_request) {
            _mag_yaw_anomally_count++;
            MY_LOG("EKF3 IMU0 MAG0 ground mag anomaly, yaw re-aligned\n");
        }
        if (interim_reset_request) {
            _final_inflight_yaw_init = false;
            _final_inflight_mag_init = false;
        }
        if (!_mag_field_learned) {
            //MY_LOG("reset mag field states\n");
            ekf3_core_reset_mag_field_states();
        }
    }
    if (_mag_state_reset_request) {
        ekf3_core_reset_mag_field_states();
    }
}

void ekf3_core_fuse_declination(float dec_err)
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
    //MY_LOG("_mag_field_learned %d, inhibit mag state %d\n", _mag_field_learned, _inhibit_mag_states);
    if (_mag_field_learned) {
        return;
    }
    float mag_dec_ang = ekf3_core_mag_declination();
    //MY_LOG("mag dec ang %f\n", degrees(mag_dec_ang));
    vector3f_t init_mag_ned = _s.state_struct.earth_magfield;
    //MY_LOG("init mag ned : %f %f %f\n", init_mag_ned.x, init_mag_ned.y, init_mag_ned.z);
    float mag_length_ne = v3f_length(&init_mag_ned);
    _s.state_struct.earth_magfield.x = mag_length_ne * cosf(mag_dec_ang);
    _s.state_struct.earth_magfield.y = mag_length_ne * sinf(mag_dec_ang);
    if (!_inhibit_mag_states) {
        float var_16 = _P[16][16];
        float var_17 = _P[17][17];
        ekf3_core_zero_rows(&_P, 16, 17);
        ekf3_core_zero_cols(&_P, 16, 17);
        _P[16][16] = var_16;
        _P[17][17] = var_17;
        ekf3_core_fuse_declination(0.1f);
    }
}

static bool fuse_euler_yaw(yaw_fusion_method_t method)
{
    const float q0 = _s.state_struct.quat.q1;
    const float q1 = _s.state_struct.quat.q2;
    const float q2 = _s.state_struct.quat.q3;
    const float q3 = _s.state_struct.quat.q4;
    //float gsf_yaw, gsf_yaw_variance;
    //if (method == YAW_FUSION_GSF) {}
    float R_YAW;
    switch (method) {
        case YAW_FUSION_GPS:
            R_YAW = sq(_yaw_ang_data_delayed.yaw_ang_err);
            break;
        //case YAW_FUSION_GSF:
        case YAW_FUSION_STATIC:
            R_YAW = sq(_yaw_ang_data_static.yaw_ang_err);
            break;
        case YAW_FUSION_MAGNETOMETER:
        case YAW_FUSION_PREDICTED:
        default:
            R_YAW = sq(_yaw_noise);
            break;
    }
    rotation_order_t order;
    switch (method) {
        case YAW_FUSION_GPS:
            order = _yaw_ang_data_delayed.order;
            break;
        case YAW_FUSION_STATIC:
            order = _yaw_ang_data_static.order;
            break;
        case YAW_FUSION_MAGNETOMETER:
        case YAW_FUSION_GSF:
        case YAW_FUSION_PREDICTED:
        default:
            order = (fabsf(_prev_tnb.a.z) < fabsf(_prev_tnb.b.z)) ? TAIT_BRYAN_321 :
            TAIT_BRYAN_312;
            break;
    }
    float yaw_ang_predicted;
    float H_YAW[4];
    matrix3f_t tbn_zero_yaw;
    if (order == TAIT_BRYAN_321) {
        bool can_use_a = false;
        const float sa0 = 2 * q3;
        const float sa1 = 2 * q2;
        const float sa2 = sa0 * q0 + sa1 * q1;
        const float sa3 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        float sa4, sa5_inv;
        if (float_is_positive(sq(sa3))) {
            sa4 = 1.0f / sq(sa3);
            sa5_inv = sq(sa2) * sa4 + 1;
            can_use_a = float_is_positive(fabsf(sa5_inv));
        }
        bool can_use_b = false;
        const float sb0 = 2 * q0;
        const float sb1 = 2 * q1;
        const float sb2 = sb0 * q3 + sb1 * q2;
        const float sb4 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        float sb3, sb5_inv;
        if (float_is_positive(sq(sb2))) {
            sb3 = 1.0f / sq(sb2);
            sb5_inv = sb3 * sq(sb4) + 1;
            can_use_b = float_is_positive(fabsf(sb5_inv));
        }
        if (can_use_a && (!can_use_b || fabsf(sa5_inv) >= fabsf(sb5_inv))) {
            const float sa5 = 1.0f / sa5_inv;
            const float sa6 = 1.0f / sa3;
            const float sa7 = sa2 * sa4;
            const float sa8 = 2 * sa7;
            const float sa9 = 2 * sa6;
            H_YAW[0] = sa5 * (sa0 * sa6 - sa8 * q0);
            H_YAW[1] = sa5 * (sa1 * sa6 - sa8 * q1);
            H_YAW[2] = sa5 * (sa1 * sa7 + sa9 * q1);
            H_YAW[3] = sa5 * (sa0 * sa7 + sa9 * q0);
        } else if (can_use_b && (!can_use_a || fabsf(sb5_inv) > fabsf(sa5_inv))) {
            const float sb5 = 1.0f / sb5_inv;
            const float sb6 = 1.0f / sb2;
            const float sb7 = sb3 * sb4;
            const float sb8 = 2 * sb7;
            const float sb9 = 2 * sb6;
            H_YAW[0] = -sb5 * (sb0 * sb6 - sb8 * q3);
            H_YAW[1] = -sb5 * (sb1 * sb6 - sb8 * q2);
            H_YAW[2] = -sb5 * (-sb1 * sb7 - sb9 * q2);
            H_YAW[3] = -sb5 * (-sb0 * sb7 - sb9 * q3);
        } else {
            return false;
        }
        vector3f_t euler321;
        quat_to_euler(&_s.state_struct.quat, &euler321.x, &euler321.y, &euler321.z);
        yaw_ang_predicted = euler321.z;
        m3f_from_euler(&tbn_zero_yaw, euler321.x, euler321.y, 0);
    } else if (order == TAIT_BRYAN_312) {
         bool can_use_a = false;
        const float sa0 = 2 * q3;
        const float sa1 = 2 * q2;
        const float sa2 = sa0 * q0 - sa1 * q1;
        const float sa3 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        float sa4, sa5_inv;
        if (float_is_positive(sq(sa3))) {
            sa4 = 1.0f / sq(sa3);
            sa5_inv = sq(sa2) * sa4 + 1;
            can_use_a = float_is_positive(fabsf(sa5_inv));
        }
        bool can_use_b = false;
        const float sb0 = 2 * q0;
        const float sb1 = 2 * q1;
        const float sb2 = -sb0 * q3 + sb1 * q2;
        const float sb4 = -sq(q0) + sq(q1) - sq(q2) + sq(q3);
        float sb3, sb5_inv;
        if (float_is_positive(sq(sb2))) {
            sb3 = 1.0f / sq(sb2);
            sb5_inv = sb3 * sq(sb4) + 1;
            can_use_b = float_is_positive(fabsf(sb5_inv));
        }
        if (can_use_a && (!can_use_b || fabsf(sa5_inv) >= fabsf(sb5_inv))) {
            const float sa5 = 1.0f / sa5_inv;
            const float sa6 = 1.0f / sa3;
            const float sa7 = sa2 * sa4;
            const float sa8 = 2 * sa7;
            const float sa9 = 2 * sa6;
            H_YAW[0] = sa5 * (sa0 * sa6 - sa8 * q0);
            H_YAW[1] = sa5 * (-sa1 * sa6 + sa8 * q1);
            H_YAW[2] = sa5 * (-sa1 * sa7 - sa9 * q1);
            H_YAW[3] = sa5 * (sa0 * sa7 + sa9 * q0);
        } else if (can_use_b && (!can_use_a || fabsf(sb5_inv) > fabsf(sa5_inv))) {
            const float sb5 = 1.0f / sb5_inv;
            const float sb6 = 1.0f / sb2;
            const float sb7 = sb3 * sb4;
            const float sb8 = 2 * sb7;
            const float sb9 = 2 * sb6;
            H_YAW[0] = -sb5 * (-sb0 * sb6 + sb8 * q3);
            H_YAW[1] = -sb5 * (sb1 * sb6 - sb8 * q2);
            H_YAW[2] = -sb5 * (-sb1 * sb7 - sb9 * q2);
            H_YAW[3] = -sb5 * (sb0 * sb7 + sb9 * q3);
        } else {
            return false;
        }
        vector3f_t euler312 = quat_to_vector312(&_s.state_struct.quat);
        yaw_ang_predicted = euler312.z;
        m3f_from_euler312(&tbn_zero_yaw, euler312.x, euler312.y, 0);
    } else {
        return false;
    }
    vector3f_t mag_meas_ned;
    float yaw_ang_measured;
    switch (method) {
        case YAW_FUSION_MAGNETOMETER:
            mag_meas_ned = m3f_multi_v(&tbn_zero_yaw, &_mag_data_delayed.mag);
            yaw_ang_measured = wrap_PI(-atan2f(mag_meas_ned.y, mag_meas_ned.x)) +
                ekf3_core_mag_declination();
            _innov_yaw = wrap_PI(yaw_ang_predicted - yaw_ang_measured);
            break;
        case YAW_FUSION_GPS:
            _innov_yaw = wrap_PI(yaw_ang_predicted - _yaw_ang_data_delayed.yaw_ang);
            break;
        case YAW_FUSION_STATIC:
            _innov_yaw = wrap_PI(yaw_ang_predicted - _yaw_ang_data_static.yaw_ang);
            break;
        case YAW_FUSION_GSF: // innovyaw = wrap_PI(yawangpredicted - gsfyaw);break;
        case YAW_FUSION_PREDICTED:
        default:
            _innov_yaw = 0.0f;
            break;
    }
    float PH[4];
    float var_innov = R_YAW;
    for (uint8_t row_index = 0; row_index <= 3; row_index++) {
        PH[row_index] = 0.0f;
        for (uint8_t col_index = 0; col_index <= 3; col_index++) {
            PH[row_index] += _P[row_index][col_index] * H_YAW[col_index];
        }
        var_innov += H_YAW[row_index] * PH[row_index];
    }
    float var_innov_inv;
    if (var_innov >= R_YAW) {
        var_innov_inv = 1.0f / var_innov;
        _fault_status.bad_yaw = false;
    } else {
        ekf3_core_covariance_init();
        _fault_status.bad_yaw = true;
        return false;
    }
    for (uint8_t row_index = 0; row_index <= _state_index_lim; row_index++) {
        _Kfusion[row_index] = 0;
        for (uint8_t col_index = 0; col_index <= 3; col_index++) {
            _Kfusion[row_index] += _P[row_index][col_index] * H_YAW[col_index];
        }
        _Kfusion[row_index]  *= var_innov_inv;
    }
    _yaw_test_ratio = sq(_innov_yaw) / (sq(MAX(0.01 * (float)_yaw_innov_gate, 1)) * var_innov);
    if (_yaw_test_ratio > 1) {
        _mag_health = false;
        if (_in_flight) {
            return false;
        }
    } else {
        _mag_health = true;
    }
    for (uint8_t row = 0; row < _state_index_lim; row++) {
        for (uint8_t column = 0; column <= 3; column++) {
            _KH[row][column] = _Kfusion[row] * H_YAW[column];
        }
    }
    for (uint8_t row = 0; row <= _state_index_lim; row++) {
        for (uint8_t column = 0; column <= _state_index_lim; column++) {
            float tmp = _KH[row][0] * _P[0][column];
            tmp += _KH[row][1] * _P[1][column];
            tmp += _KH[row][2] * _P[2][column];
            tmp += _KH[row][3] * _P[3][column];
            _KHP[row][column] = tmp;
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
        for (uint8_t i = 0; i <= _state_index_lim; i++) {
            _s.states_array[i] -= _Kfusion[i] * constrain_float(_innov_yaw, -0.5, 0.5);
        }
        quat_normalize(&_s.state_struct.quat);
        _fault_status.bad_yaw = false;
    } else {
        _fault_status.bad_yaw = true;
    }
    return true;
}

void ekf3_core_fuse_magnetometer(void)
{
    // declarations
    float *q0 = &_mag_state.q0;
    float *q1 = &_mag_state.q1;
    float *q2 = &_mag_state.q2;
    float *q3 = &_mag_state.q3;
    float *magN = &_mag_state.mag_n;
    float *magE = &_mag_state.mag_e;
    float *magD = &_mag_state.mag_d;
    float *magXbias = &_mag_state.mag_x_bias;
    float *magYbias = &_mag_state.mag_y_bias;
    float *magZbias = &_mag_state.mag_z_bias;
    matrix3f_t *DCM = &_mag_state.DCM;
    vector3f_t *MagPred = &_mag_state.mag_pred;
    float *R_MAG = &_mag_state.R_mag;
    float *SH_MAG = &_mag_state.SH_mag[0];
    Vector24 H_MAG;
    Vector5 SK_MX;
    Vector5 SK_MY;
    Vector5 SK_MZ;

    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains

    // copy required states to local variable names
    *q0       = _s.state_struct.quat.q1;
    *q1       = _s.state_struct.quat.q2;
    *q2       = _s.state_struct.quat.q3;
    *q3       = _s.state_struct.quat.q4;
    *magN     = _s.state_struct.earth_magfield.x;
    *magE     = _s.state_struct.earth_magfield.y;
    *magD     = _s.state_struct.earth_magfield.z;
    *magXbias = _s.state_struct.body_magfield.x;
    *magYbias = _s.state_struct.body_magfield.y;
    *magZbias = _s.state_struct.body_magfield.z;

    // rotate predicted earth components into body axes and calculate
    // predicted measurements
    DCM->a.x = *q0 * *q0 + *q1* *q1 - *q2* *q2 - *q3* *q3;
    DCM->a.y = 2.0f*(*q1**q2 + *q0**q3);
    DCM->a.z = 2.0f*(*q1**q3-*q0**q2);
    DCM->b.x = 2.0f*(*q1**q2 - *q0**q3);
    DCM->b.y = *q0**q0 - *q1**q1 + *q2**q2 - *q3**q3;
    DCM->b.z = 2.0f*(*q2**q3 + *q0**q1);
    DCM->c.x = 2.0f*(*q1**q3 + *q0**q2);
    DCM->c.y = 2.0f*(*q2**q3 - *q0**q1);
    DCM->c.z = *q0**q0 - *q1**q1 - *q2**q2 + *q3**q3;
    MagPred->x = DCM->a.x**magN + DCM->a.y**magE  + DCM->a.z**magD + *magXbias;
    MagPred->y = DCM->b.x**magN + DCM->b.y**magE  + DCM->b.z**magD + *magYbias;
    MagPred->z = DCM->c.x**magN + DCM->c.y**magE  + DCM->c.z**magD + *magZbias;

    // calculate the measurement innovation for each axis
    _innov_mag.x = MagPred->x - _mag_data_delayed.mag.x;
    _innov_mag.y = MagPred->y - _mag_data_delayed.mag.y;
    _innov_mag.z = MagPred->z - _mag_data_delayed.mag.z;

    // scale magnetometer observation error with total angular rate to allow for timing errors
    *R_MAG = sq(constrain_float(_mag_noise, 0.01f, 0.5f)) +
        sq(_mag_var_rate_scale* v3f_length(&_imu_data_delayed.del_ang) / _imu_data_delayed.del_ang_dt);

    // calculate common expressions used to calculate observation jacobians an innovation variance for each component
    SH_MAG[0] = 2.0f**magD**q3 + 2.0f**magE**q2 + 2.0f**magN**q1;
    SH_MAG[1] = 2.0f**magD**q0 - 2.0f**magE**q1 + 2.0f**magN**q2;
    SH_MAG[2] = 2.0f**magD**q1 + 2.0f**magE**q0 - 2.0f**magN**q3;
    SH_MAG[3] = sq(*q3);
    SH_MAG[4] = sq(*q2);
    SH_MAG[5] = sq(*q1);
    SH_MAG[6] = sq(*q0);
    SH_MAG[7] = 2.0f**magN**q0;
    SH_MAG[8] = 2.0f**magE**q3;

    // Calculate the innovation variance for each axis
    // X axis
    _var_innov_mag.x = (_P[19][19] + *R_MAG + _P[1][19]*SH_MAG[0] - _P[2][19]*SH_MAG[1] + _P[3][19]*SH_MAG[2] - _P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + (2.0f**q0**q3 + 2.0f**q1**q2)*(_P[19][17] + _P[1][17]*SH_MAG[0] - _P[2][17]*SH_MAG[1] + _P[3][17]*SH_MAG[2] - _P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][17]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][17]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - (2.0f**q0**q2 - 2.0f**q1**q3)*(_P[19][18] + _P[1][18]*SH_MAG[0] - _P[2][18]*SH_MAG[1] + _P[3][18]*SH_MAG[2] - _P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][18]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][18]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)*(_P[19][0] + _P[1][0]*SH_MAG[0] - _P[2][0]*SH_MAG[1] + _P[3][0]*SH_MAG[2] - _P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][0]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][0]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + _P[17][19]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][19]*(2.0f**q0**q2 - 2.0f**q1**q3) + SH_MAG[0]*(_P[19][1] + _P[1][1]*SH_MAG[0] - _P[2][1]*SH_MAG[1] + _P[3][1]*SH_MAG[2] - _P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][1]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][1]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - SH_MAG[1]*(_P[19][2] + _P[1][2]*SH_MAG[0] - _P[2][2]*SH_MAG[1] + _P[3][2]*SH_MAG[2] - _P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][2]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][2]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + SH_MAG[2]*(_P[19][3] + _P[1][3]*SH_MAG[0] - _P[2][3]*SH_MAG[1] + _P[3][3]*SH_MAG[2] - _P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][3]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][3]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(_P[19][16] + _P[1][16]*SH_MAG[0] - _P[2][16]*SH_MAG[1] + _P[3][16]*SH_MAG[2] - _P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + _P[17][16]*(2.0f**q0**q3 + 2.0f**q1**q2) - _P[18][16]*(2.0f**q0**q2 - 2.0f**q1**q3) + _P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + _P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2));
    if (_var_innov_mag.x >= *R_MAG) {
        _fault_status.bad_xmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        ekf3_core_covariance_init();
        _fault_status.bad_xmag = true;
        return;
    }

    // Y axis
    _var_innov_mag.y = (_P[20][20] + *R_MAG + _P[0][20]*SH_MAG[2] + _P[1][20]*SH_MAG[1] + _P[2][20]*SH_MAG[0] - _P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2.0f**q0**q3 - 2.0f**q1**q2)*(_P[20][16] + _P[0][16]*SH_MAG[2] + _P[1][16]*SH_MAG[1] + _P[2][16]*SH_MAG[0] - _P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][16]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][16]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + (2.0f**q0**q1 + 2.0f**q2**q3)*(_P[20][18] + _P[0][18]*SH_MAG[2] + _P[1][18]*SH_MAG[1] + _P[2][18]*SH_MAG[0] - _P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][18]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][18]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - (SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)*(_P[20][3] + _P[0][3]*SH_MAG[2] + _P[1][3]*SH_MAG[1] + _P[2][3]*SH_MAG[0] - _P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][3]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][3]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - _P[16][20]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][20]*(2.0f**q0**q1 + 2.0f**q2**q3) + SH_MAG[2]*(_P[20][0] + _P[0][0]*SH_MAG[2] + _P[1][0]*SH_MAG[1] + _P[2][0]*SH_MAG[0] - _P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][0]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][0]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + SH_MAG[1]*(_P[20][1] + _P[0][1]*SH_MAG[2] + _P[1][1]*SH_MAG[1] + _P[2][1]*SH_MAG[0] - _P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][1]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][1]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + SH_MAG[0]*(_P[20][2] + _P[0][2]*SH_MAG[2] + _P[1][2]*SH_MAG[1] + _P[2][2]*SH_MAG[0] - _P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][2]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][2]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(_P[20][17] + _P[0][17]*SH_MAG[2] + _P[1][17]*SH_MAG[1] + _P[2][17]*SH_MAG[0] - _P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - _P[16][17]*(2.0f**q0**q3 - 2.0f**q1**q2) + _P[18][17]*(2.0f**q0**q1 + 2.0f**q2**q3) - _P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - _P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2));
    if (_var_innov_mag.y >= *R_MAG) {
        _fault_status.bad_ymag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        ekf3_core_covariance_init();
        _fault_status.bad_ymag = true;
        return;
    }

    // Z axis
    _var_innov_mag.z = (_P[21][21] + *R_MAG + _P[0][21]*SH_MAG[1] - _P[1][21]*SH_MAG[2] + _P[3][21]*SH_MAG[0] + _P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + (2.0f**q0**q2 + 2.0f**q1**q3)*(_P[21][16] + _P[0][16]*SH_MAG[1] - _P[1][16]*SH_MAG[2] + _P[3][16]*SH_MAG[0] + _P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][16]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][16]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - (2.0f**q0**q1 - 2.0f**q2**q3)*(_P[21][17] + _P[0][17]*SH_MAG[1] - _P[1][17]*SH_MAG[2] + _P[3][17]*SH_MAG[0] + _P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][17]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][17]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + (SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)*(_P[21][2] + _P[0][2]*SH_MAG[1] - _P[1][2]*SH_MAG[2] + _P[3][2]*SH_MAG[0] + _P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][2]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][2]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + _P[16][21]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][21]*(2.0f**q0**q1 - 2.0f**q2**q3) + SH_MAG[1]*(_P[21][0] + _P[0][0]*SH_MAG[1] - _P[1][0]*SH_MAG[2] + _P[3][0]*SH_MAG[0] + _P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][0]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][0]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) - SH_MAG[2]*(_P[21][1] + _P[0][1]*SH_MAG[1] - _P[1][1]*SH_MAG[2] + _P[3][1]*SH_MAG[0] + _P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][1]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][1]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + SH_MAG[0]*(_P[21][3] + _P[0][3]*SH_MAG[1] - _P[1][3]*SH_MAG[2] + _P[3][3]*SH_MAG[0] + _P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][3]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][3]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(_P[21][18] + _P[0][18]*SH_MAG[1] - _P[1][18]*SH_MAG[2] + _P[3][18]*SH_MAG[0] + _P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + _P[16][18]*(2.0f**q0**q2 + 2.0f**q1**q3) - _P[17][18]*(2.0f**q0**q1 - 2.0f**q2**q3) + _P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2)) + _P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2));
    if (_var_innov_mag.z >= *R_MAG) {
        _fault_status.bad_zmag = false;
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        ekf3_core_covariance_init();
        _fault_status.bad_zmag = true;
        return;
    }

    // calculate the innovation test ratios
    _mag_test_ratio.x = sq(_innov_mag.x) / (sq(MAX(0.01f * (float)_mag_innov_gate, 1.0f)) * _var_innov_mag.x);
    _mag_test_ratio.y = sq(_innov_mag.y) / (sq(MAX(0.01f * (float)_mag_innov_gate, 1.0f)) * _var_innov_mag.y);
    _mag_test_ratio.z = sq(_innov_mag.z) / (sq(MAX(0.01f * (float)_mag_innov_gate, 1.0f)) * _var_innov_mag.z);

    // check the last values from all components and set magnetometer health accordingly
    _mag_health = (_mag_test_ratio.x < 1.0f && _mag_test_ratio.y < 1.0f && _mag_test_ratio.z < 1.0f);

    // if the magnetometer is unhealthy, do not proceed further
    if (!_mag_health) {
        return;
    }

    for (uint8_t obsIndex = 0; obsIndex <= 2; obsIndex++) {

        if (obsIndex == 0) {

            for (uint8_t i = 0; i<=_state_index_lim; i++) H_MAG[i] = 0.0f;
            H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2;
            H_MAG[1] = SH_MAG[0];
            H_MAG[2] = -SH_MAG[1];
            H_MAG[3] = SH_MAG[2];
            H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
            H_MAG[17] = 2.0f**q0**q3 + 2.0f**q1**q2;
            H_MAG[18] = 2.0f**q1**q3 - 2.0f**q0**q2;
            H_MAG[19] = 1.0f;
            H_MAG[20] = 0.0f;
            H_MAG[21] = 0.0f;

            // calculate Kalman gain
            SK_MX[0] = 1.0f / _var_innov_mag.x;
            SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
            SK_MX[2] = SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2;
            SK_MX[3] = 2.0f**q0**q2 - 2.0f**q1**q3;
            SK_MX[4] = 2.0f**q0**q3 + 2.0f**q1**q2;

            _Kfusion[0] = SK_MX[0]*(_P[0][19] + _P[0][1]*SH_MAG[0] - _P[0][2]*SH_MAG[1] + _P[0][3]*SH_MAG[2] + _P[0][0]*SK_MX[2] - _P[0][16]*SK_MX[1] + _P[0][17]*SK_MX[4] - _P[0][18]*SK_MX[3]);
            _Kfusion[1] = SK_MX[0]*(_P[1][19] + _P[1][1]*SH_MAG[0] - _P[1][2]*SH_MAG[1] + _P[1][3]*SH_MAG[2] + _P[1][0]*SK_MX[2] - _P[1][16]*SK_MX[1] + _P[1][17]*SK_MX[4] - _P[1][18]*SK_MX[3]);
            _Kfusion[2] = SK_MX[0]*(_P[2][19] + _P[2][1]*SH_MAG[0] - _P[2][2]*SH_MAG[1] + _P[2][3]*SH_MAG[2] + _P[2][0]*SK_MX[2] - _P[2][16]*SK_MX[1] + _P[2][17]*SK_MX[4] - _P[2][18]*SK_MX[3]);
            _Kfusion[3] = SK_MX[0]*(_P[3][19] + _P[3][1]*SH_MAG[0] - _P[3][2]*SH_MAG[1] + _P[3][3]*SH_MAG[2] + _P[3][0]*SK_MX[2] - _P[3][16]*SK_MX[1] + _P[3][17]*SK_MX[4] - _P[3][18]*SK_MX[3]);
            _Kfusion[4] = SK_MX[0]*(_P[4][19] + _P[4][1]*SH_MAG[0] - _P[4][2]*SH_MAG[1] + _P[4][3]*SH_MAG[2] + _P[4][0]*SK_MX[2] - _P[4][16]*SK_MX[1] + _P[4][17]*SK_MX[4] - _P[4][18]*SK_MX[3]);
            _Kfusion[5] = SK_MX[0]*(_P[5][19] + _P[5][1]*SH_MAG[0] - _P[5][2]*SH_MAG[1] + _P[5][3]*SH_MAG[2] + _P[5][0]*SK_MX[2] - _P[5][16]*SK_MX[1] + _P[5][17]*SK_MX[4] - _P[5][18]*SK_MX[3]);
            _Kfusion[6] = SK_MX[0]*(_P[6][19] + _P[6][1]*SH_MAG[0] - _P[6][2]*SH_MAG[1] + _P[6][3]*SH_MAG[2] + _P[6][0]*SK_MX[2] - _P[6][16]*SK_MX[1] + _P[6][17]*SK_MX[4] - _P[6][18]*SK_MX[3]);
            _Kfusion[7] = SK_MX[0]*(_P[7][19] + _P[7][1]*SH_MAG[0] - _P[7][2]*SH_MAG[1] + _P[7][3]*SH_MAG[2] + _P[7][0]*SK_MX[2] - _P[7][16]*SK_MX[1] + _P[7][17]*SK_MX[4] - _P[7][18]*SK_MX[3]);
            _Kfusion[8] = SK_MX[0]*(_P[8][19] + _P[8][1]*SH_MAG[0] - _P[8][2]*SH_MAG[1] + _P[8][3]*SH_MAG[2] + _P[8][0]*SK_MX[2] - _P[8][16]*SK_MX[1] + _P[8][17]*SK_MX[4] - _P[8][18]*SK_MX[3]);
            _Kfusion[9] = SK_MX[0]*(_P[9][19] + _P[9][1]*SH_MAG[0] - _P[9][2]*SH_MAG[1] + _P[9][3]*SH_MAG[2] + _P[9][0]*SK_MX[2] - _P[9][16]*SK_MX[1] + _P[9][17]*SK_MX[4] - _P[9][18]*SK_MX[3]);

            if (!_inhibit_del_ang_bias_states) {
                _Kfusion[10] = SK_MX[0]*(_P[10][19] + _P[10][1]*SH_MAG[0] - _P[10][2]*SH_MAG[1] + _P[10][3]*SH_MAG[2] + _P[10][0]*SK_MX[2] - _P[10][16]*SK_MX[1] + _P[10][17]*SK_MX[4] - _P[10][18]*SK_MX[3]);
                _Kfusion[11] = SK_MX[0]*(_P[11][19] + _P[11][1]*SH_MAG[0] - _P[11][2]*SH_MAG[1] + _P[11][3]*SH_MAG[2] + _P[11][0]*SK_MX[2] - _P[11][16]*SK_MX[1] + _P[11][17]*SK_MX[4] - _P[11][18]*SK_MX[3]);
                _Kfusion[12] = SK_MX[0]*(_P[12][19] + _P[12][1]*SH_MAG[0] - _P[12][2]*SH_MAG[1] + _P[12][3]*SH_MAG[2] + _P[12][0]*SK_MX[2] - _P[12][16]*SK_MX[1] + _P[12][17]*SK_MX[4] - _P[12][18]*SK_MX[3]);
            } else {
                // zero indexes 10 to 12
                ekf3_core_zero_range(&_Kfusion[0], 10, 12);
            }

            if (!_inhibit_del_vel_bias_states) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!_dvel_bias_axis_inhibit[index]) {
                        _Kfusion[stateIndex] = SK_MX[0]*(_P[stateIndex][19] + _P[stateIndex][1]*SH_MAG[0] - _P[stateIndex][2]*SH_MAG[1] + _P[stateIndex][3]*SH_MAG[2] + _P[stateIndex][0]*SK_MX[2] - _P[stateIndex][16]*SK_MX[1] + _P[stateIndex][17]*SK_MX[4] - _P[stateIndex][18]*SK_MX[3]);
                    } else {
                        _Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                ekf3_core_zero_range(&_Kfusion[0], 13, 15);
            }
            // zero Kalman gains to inhibit magnetic field state estimation
            if (!_inhibit_mag_states) {
                _Kfusion[16] = SK_MX[0]*(_P[16][19] + _P[16][1]*SH_MAG[0] - _P[16][2]*SH_MAG[1] + _P[16][3]*SH_MAG[2] + _P[16][0]*SK_MX[2] - _P[16][16]*SK_MX[1] + _P[16][17]*SK_MX[4] - _P[16][18]*SK_MX[3]);
                _Kfusion[17] = SK_MX[0]*(_P[17][19] + _P[17][1]*SH_MAG[0] - _P[17][2]*SH_MAG[1] + _P[17][3]*SH_MAG[2] + _P[17][0]*SK_MX[2] - _P[17][16]*SK_MX[1] + _P[17][17]*SK_MX[4] - _P[17][18]*SK_MX[3]);
                _Kfusion[18] = SK_MX[0]*(_P[18][19] + _P[18][1]*SH_MAG[0] - _P[18][2]*SH_MAG[1] + _P[18][3]*SH_MAG[2] + _P[18][0]*SK_MX[2] - _P[18][16]*SK_MX[1] + _P[18][17]*SK_MX[4] - _P[18][18]*SK_MX[3]);
                _Kfusion[19] = SK_MX[0]*(_P[19][19] + _P[19][1]*SH_MAG[0] - _P[19][2]*SH_MAG[1] + _P[19][3]*SH_MAG[2] + _P[19][0]*SK_MX[2] - _P[19][16]*SK_MX[1] + _P[19][17]*SK_MX[4] - _P[19][18]*SK_MX[3]);
                _Kfusion[20] = SK_MX[0]*(_P[20][19] + _P[20][1]*SH_MAG[0] - _P[20][2]*SH_MAG[1] + _P[20][3]*SH_MAG[2] + _P[20][0]*SK_MX[2] - _P[20][16]*SK_MX[1] + _P[20][17]*SK_MX[4] - _P[20][18]*SK_MX[3]);
                _Kfusion[21] = SK_MX[0]*(_P[21][19] + _P[21][1]*SH_MAG[0] - _P[21][2]*SH_MAG[1] + _P[21][3]*SH_MAG[2] + _P[21][0]*SK_MX[2] - _P[21][16]*SK_MX[1] + _P[21][17]*SK_MX[4] - _P[21][18]*SK_MX[3]);
            } else {
                // zero indexes 16 to 21
                ekf3_core_zero_range(&_Kfusion[0], 16, 21);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!_inhibit_wind_states) {
                _Kfusion[22] = SK_MX[0]*(_P[22][19] + _P[22][1]*SH_MAG[0] - _P[22][2]*SH_MAG[1] + _P[22][3]*SH_MAG[2] + _P[22][0]*SK_MX[2] - _P[22][16]*SK_MX[1] + _P[22][17]*SK_MX[4] - _P[22][18]*SK_MX[3]);
                _Kfusion[23] = SK_MX[0]*(_P[23][19] + _P[23][1]*SH_MAG[0] - _P[23][2]*SH_MAG[1] + _P[23][3]*SH_MAG[2] + _P[23][0]*SK_MX[2] - _P[23][16]*SK_MX[1] + _P[23][17]*SK_MX[4] - _P[23][18]*SK_MX[3]);
            } else {
                // zero indexes 22 to 23 = 2
                ekf3_core_zero_range(&_Kfusion[0], 22, 23);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            _mag_fuse_performed = true;
        } else if (obsIndex == 1) { // Fuse Y axis

            // calculate observation jacobians
            for (uint8_t i = 0; i<=_state_index_lim; i++) H_MAG[i] = 0.0f;
            H_MAG[0] = SH_MAG[2];
            H_MAG[1] = SH_MAG[1];
            H_MAG[2] = SH_MAG[0];
            H_MAG[3] = 2.0f**magD**q2 - SH_MAG[8] - SH_MAG[7];
            H_MAG[16] = 2.0f**q1**q2 - 2.0f**q0**q3;
            H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
            H_MAG[18] = 2.0f**q0**q1 + 2.0f**q2**q3;
            H_MAG[19] = 0.0f;
            H_MAG[20] = 1.0f;
            H_MAG[21] = 0.0f;

            // calculate Kalman gain
            SK_MY[0] = 1.0f / _var_innov_mag.y;
            SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
            SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2;
            SK_MY[3] = 2.0f**q0**q3 - 2.0f**q1**q2;
            SK_MY[4] = 2.0f**q0**q1 + 2.0f**q2**q3;

            _Kfusion[0] = SK_MY[0]*(_P[0][20] + _P[0][0]*SH_MAG[2] + _P[0][1]*SH_MAG[1] + _P[0][2]*SH_MAG[0] - _P[0][3]*SK_MY[2] - _P[0][17]*SK_MY[1] - _P[0][16]*SK_MY[3] + _P[0][18]*SK_MY[4]);
            _Kfusion[1] = SK_MY[0]*(_P[1][20] + _P[1][0]*SH_MAG[2] + _P[1][1]*SH_MAG[1] + _P[1][2]*SH_MAG[0] - _P[1][3]*SK_MY[2] - _P[1][17]*SK_MY[1] - _P[1][16]*SK_MY[3] + _P[1][18]*SK_MY[4]);
            _Kfusion[2] = SK_MY[0]*(_P[2][20] + _P[2][0]*SH_MAG[2] + _P[2][1]*SH_MAG[1] + _P[2][2]*SH_MAG[0] - _P[2][3]*SK_MY[2] - _P[2][17]*SK_MY[1] - _P[2][16]*SK_MY[3] + _P[2][18]*SK_MY[4]);
            _Kfusion[3] = SK_MY[0]*(_P[3][20] + _P[3][0]*SH_MAG[2] + _P[3][1]*SH_MAG[1] + _P[3][2]*SH_MAG[0] - _P[3][3]*SK_MY[2] - _P[3][17]*SK_MY[1] - _P[3][16]*SK_MY[3] + _P[3][18]*SK_MY[4]);
            _Kfusion[4] = SK_MY[0]*(_P[4][20] + _P[4][0]*SH_MAG[2] + _P[4][1]*SH_MAG[1] + _P[4][2]*SH_MAG[0] - _P[4][3]*SK_MY[2] - _P[4][17]*SK_MY[1] - _P[4][16]*SK_MY[3] + _P[4][18]*SK_MY[4]);
            _Kfusion[5] = SK_MY[0]*(_P[5][20] + _P[5][0]*SH_MAG[2] + _P[5][1]*SH_MAG[1] + _P[5][2]*SH_MAG[0] - _P[5][3]*SK_MY[2] - _P[5][17]*SK_MY[1] - _P[5][16]*SK_MY[3] + _P[5][18]*SK_MY[4]);
            _Kfusion[6] = SK_MY[0]*(_P[6][20] + _P[6][0]*SH_MAG[2] + _P[6][1]*SH_MAG[1] + _P[6][2]*SH_MAG[0] - _P[6][3]*SK_MY[2] - _P[6][17]*SK_MY[1] - _P[6][16]*SK_MY[3] + _P[6][18]*SK_MY[4]);
            _Kfusion[7] = SK_MY[0]*(_P[7][20] + _P[7][0]*SH_MAG[2] + _P[7][1]*SH_MAG[1] + _P[7][2]*SH_MAG[0] - _P[7][3]*SK_MY[2] - _P[7][17]*SK_MY[1] - _P[7][16]*SK_MY[3] + _P[7][18]*SK_MY[4]);
            _Kfusion[8] = SK_MY[0]*(_P[8][20] + _P[8][0]*SH_MAG[2] + _P[8][1]*SH_MAG[1] + _P[8][2]*SH_MAG[0] - _P[8][3]*SK_MY[2] - _P[8][17]*SK_MY[1] - _P[8][16]*SK_MY[3] + _P[8][18]*SK_MY[4]);
            _Kfusion[9] = SK_MY[0]*(_P[9][20] + _P[9][0]*SH_MAG[2] + _P[9][1]*SH_MAG[1] + _P[9][2]*SH_MAG[0] - _P[9][3]*SK_MY[2] - _P[9][17]*SK_MY[1] - _P[9][16]*SK_MY[3] + _P[9][18]*SK_MY[4]);

            if (!_inhibit_del_ang_bias_states) {
                _Kfusion[10] = SK_MY[0]*(_P[10][20] + _P[10][0]*SH_MAG[2] + _P[10][1]*SH_MAG[1] + _P[10][2]*SH_MAG[0] - _P[10][3]*SK_MY[2] - _P[10][17]*SK_MY[1] - _P[10][16]*SK_MY[3] + _P[10][18]*SK_MY[4]);
                _Kfusion[11] = SK_MY[0]*(_P[11][20] + _P[11][0]*SH_MAG[2] + _P[11][1]*SH_MAG[1] + _P[11][2]*SH_MAG[0] - _P[11][3]*SK_MY[2] - _P[11][17]*SK_MY[1] - _P[11][16]*SK_MY[3] + _P[11][18]*SK_MY[4]);
                _Kfusion[12] = SK_MY[0]*(_P[12][20] + _P[12][0]*SH_MAG[2] + _P[12][1]*SH_MAG[1] + _P[12][2]*SH_MAG[0] - _P[12][3]*SK_MY[2] - _P[12][17]*SK_MY[1] - _P[12][16]*SK_MY[3] + _P[12][18]*SK_MY[4]);
            } else {
                // zero indexes 10 to 12
                ekf3_core_zero_range(&_Kfusion[0], 10, 12);
            }

            if (!_inhibit_del_vel_bias_states) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!_dvel_bias_axis_inhibit[index]) {
                        _Kfusion[stateIndex] = SK_MY[0]*(_P[stateIndex][20] + _P[stateIndex][0]*SH_MAG[2] + _P[stateIndex][1]*SH_MAG[1] + _P[stateIndex][2]*SH_MAG[0] - _P[stateIndex][3]*SK_MY[2] - _P[stateIndex][17]*SK_MY[1] - _P[stateIndex][16]*SK_MY[3] + _P[stateIndex][18]*SK_MY[4]);
                    } else {
                        _Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                ekf3_core_zero_range(&_Kfusion[0], 13, 15);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (!_inhibit_mag_states) {
                _Kfusion[16] = SK_MY[0]*(_P[16][20] + _P[16][0]*SH_MAG[2] + _P[16][1]*SH_MAG[1] + _P[16][2]*SH_MAG[0] - _P[16][3]*SK_MY[2] - _P[16][17]*SK_MY[1] - _P[16][16]*SK_MY[3] + _P[16][18]*SK_MY[4]);
                _Kfusion[17] = SK_MY[0]*(_P[17][20] + _P[17][0]*SH_MAG[2] + _P[17][1]*SH_MAG[1] + _P[17][2]*SH_MAG[0] - _P[17][3]*SK_MY[2] - _P[17][17]*SK_MY[1] - _P[17][16]*SK_MY[3] + _P[17][18]*SK_MY[4]);
                _Kfusion[18] = SK_MY[0]*(_P[18][20] + _P[18][0]*SH_MAG[2] + _P[18][1]*SH_MAG[1] + _P[18][2]*SH_MAG[0] - _P[18][3]*SK_MY[2] - _P[18][17]*SK_MY[1] - _P[18][16]*SK_MY[3] + _P[18][18]*SK_MY[4]);
                _Kfusion[19] = SK_MY[0]*(_P[19][20] + _P[19][0]*SH_MAG[2] + _P[19][1]*SH_MAG[1] + _P[19][2]*SH_MAG[0] - _P[19][3]*SK_MY[2] - _P[19][17]*SK_MY[1] - _P[19][16]*SK_MY[3] + _P[19][18]*SK_MY[4]);
                _Kfusion[20] = SK_MY[0]*(_P[20][20] + _P[20][0]*SH_MAG[2] + _P[20][1]*SH_MAG[1] + _P[20][2]*SH_MAG[0] - _P[20][3]*SK_MY[2] - _P[20][17]*SK_MY[1] - _P[20][16]*SK_MY[3] + _P[20][18]*SK_MY[4]);
                _Kfusion[21] = SK_MY[0]*(_P[21][20] + _P[21][0]*SH_MAG[2] + _P[21][1]*SH_MAG[1] + _P[21][2]*SH_MAG[0] - _P[21][3]*SK_MY[2] - _P[21][17]*SK_MY[1] - _P[21][16]*SK_MY[3] + _P[21][18]*SK_MY[4]);
            } else {
                // zero indexes 16 to 21
                ekf3_core_zero_range(&_Kfusion[0], 16, 21);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!_inhibit_wind_states) {
                _Kfusion[22] = SK_MY[0]*(_P[22][20] + _P[22][0]*SH_MAG[2] + _P[22][1]*SH_MAG[1] + _P[22][2]*SH_MAG[0] - _P[22][3]*SK_MY[2] - _P[22][17]*SK_MY[1] - _P[22][16]*SK_MY[3] + _P[22][18]*SK_MY[4]);
                _Kfusion[23] = SK_MY[0]*(_P[23][20] + _P[23][0]*SH_MAG[2] + _P[23][1]*SH_MAG[1] + _P[23][2]*SH_MAG[0] - _P[23][3]*SK_MY[2] - _P[23][17]*SK_MY[1] - _P[23][16]*SK_MY[3] + _P[23][18]*SK_MY[4]);
            } else {
                // zero indexes 22 to 23
                ekf3_core_zero_range(&_Kfusion[0], 22, 23);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            _mag_fuse_performed = true;
        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // calculate observation jacobians
            for (uint8_t i = 0; i<=_state_index_lim; i++) H_MAG[i] = 0.0f;
            H_MAG[0] = SH_MAG[1];
            H_MAG[1] = -SH_MAG[2];
            H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2;
            H_MAG[3] = SH_MAG[0];
            H_MAG[16] = 2.0f**q0**q2 + 2.0f**q1**q3;
            H_MAG[17] = 2.0f**q2**q3 - 2.0f**q0**q1;
            H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            H_MAG[19] = 0.0f;
            H_MAG[20] = 0.0f;
            H_MAG[21] = 1.0f;

            // calculate Kalman gain
            SK_MZ[0] = 1.0f / _var_innov_mag.z;
            SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            SK_MZ[2] = SH_MAG[7] + SH_MAG[8] - 2.0f**magD**q2;
            SK_MZ[3] = 2.0f**q0**q1 - 2.0f**q2**q3;
            SK_MZ[4] = 2.0f**q0**q2 + 2.0f**q1**q3;

            _Kfusion[0] = SK_MZ[0]*(_P[0][21] + _P[0][0]*SH_MAG[1] - _P[0][1]*SH_MAG[2] + _P[0][3]*SH_MAG[0] + _P[0][2]*SK_MZ[2] + _P[0][18]*SK_MZ[1] + _P[0][16]*SK_MZ[4] - _P[0][17]*SK_MZ[3]);
            _Kfusion[1] = SK_MZ[0]*(_P[1][21] + _P[1][0]*SH_MAG[1] - _P[1][1]*SH_MAG[2] + _P[1][3]*SH_MAG[0] + _P[1][2]*SK_MZ[2] + _P[1][18]*SK_MZ[1] + _P[1][16]*SK_MZ[4] - _P[1][17]*SK_MZ[3]);
            _Kfusion[2] = SK_MZ[0]*(_P[2][21] + _P[2][0]*SH_MAG[1] - _P[2][1]*SH_MAG[2] + _P[2][3]*SH_MAG[0] + _P[2][2]*SK_MZ[2] + _P[2][18]*SK_MZ[1] + _P[2][16]*SK_MZ[4] - _P[2][17]*SK_MZ[3]);
            _Kfusion[3] = SK_MZ[0]*(_P[3][21] + _P[3][0]*SH_MAG[1] - _P[3][1]*SH_MAG[2] + _P[3][3]*SH_MAG[0] + _P[3][2]*SK_MZ[2] + _P[3][18]*SK_MZ[1] + _P[3][16]*SK_MZ[4] - _P[3][17]*SK_MZ[3]);
            _Kfusion[4] = SK_MZ[0]*(_P[4][21] + _P[4][0]*SH_MAG[1] - _P[4][1]*SH_MAG[2] + _P[4][3]*SH_MAG[0] + _P[4][2]*SK_MZ[2] + _P[4][18]*SK_MZ[1] + _P[4][16]*SK_MZ[4] - _P[4][17]*SK_MZ[3]);
            _Kfusion[5] = SK_MZ[0]*(_P[5][21] + _P[5][0]*SH_MAG[1] - _P[5][1]*SH_MAG[2] + _P[5][3]*SH_MAG[0] + _P[5][2]*SK_MZ[2] + _P[5][18]*SK_MZ[1] + _P[5][16]*SK_MZ[4] - _P[5][17]*SK_MZ[3]);
            _Kfusion[6] = SK_MZ[0]*(_P[6][21] + _P[6][0]*SH_MAG[1] - _P[6][1]*SH_MAG[2] + _P[6][3]*SH_MAG[0] + _P[6][2]*SK_MZ[2] + _P[6][18]*SK_MZ[1] + _P[6][16]*SK_MZ[4] - _P[6][17]*SK_MZ[3]);
            _Kfusion[7] = SK_MZ[0]*(_P[7][21] + _P[7][0]*SH_MAG[1] - _P[7][1]*SH_MAG[2] + _P[7][3]*SH_MAG[0] + _P[7][2]*SK_MZ[2] + _P[7][18]*SK_MZ[1] + _P[7][16]*SK_MZ[4] - _P[7][17]*SK_MZ[3]);
            _Kfusion[8] = SK_MZ[0]*(_P[8][21] + _P[8][0]*SH_MAG[1] - _P[8][1]*SH_MAG[2] + _P[8][3]*SH_MAG[0] + _P[8][2]*SK_MZ[2] + _P[8][18]*SK_MZ[1] + _P[8][16]*SK_MZ[4] - _P[8][17]*SK_MZ[3]);
            _Kfusion[9] = SK_MZ[0]*(_P[9][21] + _P[9][0]*SH_MAG[1] - _P[9][1]*SH_MAG[2] + _P[9][3]*SH_MAG[0] + _P[9][2]*SK_MZ[2] + _P[9][18]*SK_MZ[1] + _P[9][16]*SK_MZ[4] - _P[9][17]*SK_MZ[3]);

            if (!_inhibit_del_ang_bias_states) {
                _Kfusion[10] = SK_MZ[0]*(_P[10][21] + _P[10][0]*SH_MAG[1] - _P[10][1]*SH_MAG[2] + _P[10][3]*SH_MAG[0] + _P[10][2]*SK_MZ[2] + _P[10][18]*SK_MZ[1] + _P[10][16]*SK_MZ[4] - _P[10][17]*SK_MZ[3]);
                _Kfusion[11] = SK_MZ[0]*(_P[11][21] + _P[11][0]*SH_MAG[1] - _P[11][1]*SH_MAG[2] + _P[11][3]*SH_MAG[0] + _P[11][2]*SK_MZ[2] + _P[11][18]*SK_MZ[1] + _P[11][16]*SK_MZ[4] - _P[11][17]*SK_MZ[3]);
                _Kfusion[12] = SK_MZ[0]*(_P[12][21] + _P[12][0]*SH_MAG[1] - _P[12][1]*SH_MAG[2] + _P[12][3]*SH_MAG[0] + _P[12][2]*SK_MZ[2] + _P[12][18]*SK_MZ[1] + _P[12][16]*SK_MZ[4] - _P[12][17]*SK_MZ[3]);
            } else {
                // zero indexes 10 to 12
                ekf3_core_zero_range(&_Kfusion[0], 10, 12);
            }

            if (!_inhibit_del_vel_bias_states) {
                for (uint8_t index = 0; index < 3; index++) {
                    const uint8_t stateIndex = index + 13;
                    if (!_dvel_bias_axis_inhibit[index]) {
                        _Kfusion[stateIndex] = SK_MZ[0]*(_P[stateIndex][21] + _P[stateIndex][0]*SH_MAG[1] - _P[stateIndex][1]*SH_MAG[2] + _P[stateIndex][3]*SH_MAG[0] + _P[stateIndex][2]*SK_MZ[2] + _P[stateIndex][18]*SK_MZ[1] + _P[stateIndex][16]*SK_MZ[4] - _P[stateIndex][17]*SK_MZ[3]);
                    } else {
                        _Kfusion[stateIndex] = 0.0f;
                    }
                }
            } else {
                // zero indexes 13 to 15
                ekf3_core_zero_range(&_Kfusion[0], 13, 15);
            }

            // zero Kalman gains to inhibit magnetic field state estimation
            if (!_inhibit_mag_states) {
                _Kfusion[16] = SK_MZ[0]*(_P[16][21] + _P[16][0]*SH_MAG[1] - _P[16][1]*SH_MAG[2] + _P[16][3]*SH_MAG[0] + _P[16][2]*SK_MZ[2] + _P[16][18]*SK_MZ[1] + _P[16][16]*SK_MZ[4] - _P[16][17]*SK_MZ[3]);
                _Kfusion[17] = SK_MZ[0]*(_P[17][21] + _P[17][0]*SH_MAG[1] - _P[17][1]*SH_MAG[2] + _P[17][3]*SH_MAG[0] + _P[17][2]*SK_MZ[2] + _P[17][18]*SK_MZ[1] + _P[17][16]*SK_MZ[4] - _P[17][17]*SK_MZ[3]);
                _Kfusion[18] = SK_MZ[0]*(_P[18][21] + _P[18][0]*SH_MAG[1] - _P[18][1]*SH_MAG[2] + _P[18][3]*SH_MAG[0] + _P[18][2]*SK_MZ[2] + _P[18][18]*SK_MZ[1] + _P[18][16]*SK_MZ[4] - _P[18][17]*SK_MZ[3]);
                _Kfusion[19] = SK_MZ[0]*(_P[19][21] + _P[19][0]*SH_MAG[1] - _P[19][1]*SH_MAG[2] + _P[19][3]*SH_MAG[0] + _P[19][2]*SK_MZ[2] + _P[19][18]*SK_MZ[1] + _P[19][16]*SK_MZ[4] - _P[19][17]*SK_MZ[3]);
                _Kfusion[20] = SK_MZ[0]*(_P[20][21] + _P[20][0]*SH_MAG[1] - _P[20][1]*SH_MAG[2] + _P[20][3]*SH_MAG[0] + _P[20][2]*SK_MZ[2] + _P[20][18]*SK_MZ[1] + _P[20][16]*SK_MZ[4] - _P[20][17]*SK_MZ[3]);
                _Kfusion[21] = SK_MZ[0]*(_P[21][21] + _P[21][0]*SH_MAG[1] - _P[21][1]*SH_MAG[2] + _P[21][3]*SH_MAG[0] + _P[21][2]*SK_MZ[2] + _P[21][18]*SK_MZ[1] + _P[21][16]*SK_MZ[4] - _P[21][17]*SK_MZ[3]);
            } else {
                // zero indexes 16 to 21
                ekf3_core_zero_range(&_Kfusion[0], 16, 21);
            }

            // zero Kalman gains to inhibit wind state estimation
            if (!_inhibit_wind_states) {
                _Kfusion[22] = SK_MZ[0]*(_P[22][21] + _P[22][0]*SH_MAG[1] - _P[22][1]*SH_MAG[2] + _P[22][3]*SH_MAG[0] + _P[22][2]*SK_MZ[2] + _P[22][18]*SK_MZ[1] + _P[22][16]*SK_MZ[4] - _P[22][17]*SK_MZ[3]);
                _Kfusion[23] = SK_MZ[0]*(_P[23][21] + _P[23][0]*SH_MAG[1] - _P[23][1]*SH_MAG[2] + _P[23][3]*SH_MAG[0] + _P[23][2]*SK_MZ[2] + _P[23][18]*SK_MZ[1] + _P[23][16]*SK_MZ[4] - _P[23][17]*SK_MZ[3]);
            } else {
                // zero indexes 22 to 23
                ekf3_core_zero_range(&_Kfusion[0], 22, 23);
            }

            // set flags to indicate to other processes that fusion has been performed and is required on the next frame
            // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
            _mag_fuse_performed = true;
        }
        // correct the covariance _P = (I - K*H)*_P
        // take advantage of the empty columns in _KH to reduce the
        // number of operations
        for (unsigned i = 0; i<=_state_index_lim; i++) {
            for (unsigned j = 0; j<=3; j++) {
                _KH[i][j] = _Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 4; j<=15; j++) {
                _KH[i][j] = 0.0f;
            }
            for (unsigned j = 16; j<=21; j++) {
                _KH[i][j] = _Kfusion[i] * H_MAG[j];
            }
            for (unsigned j = 22; j<=23; j++) {
                _KH[i][j] = 0.0f;
            }
        }
        for (unsigned j = 0; j<=_state_index_lim; j++) {
            for (unsigned i = 0; i<=_state_index_lim; i++) {
                float res = 0;
                res += _KH[i][0] * _P[0][j];
                res += _KH[i][1] * _P[1][j];
                res += _KH[i][2] * _P[2][j];
                res += _KH[i][3] * _P[3][j];
                res += _KH[i][16] * _P[16][j];
                res += _KH[i][17] * _P[17][j];
                res += _KH[i][18] * _P[18][j];
                res += _KH[i][19] * _P[19][j];
                res += _KH[i][20] * _P[20][j];
                res += _KH[i][21] * _P[21][j];
                _KHP[i][j] = res;
            }
        }
        // Check that we are not going to drive any variances negative and skip the update if so
        bool healthyFusion = true;
        for (uint8_t i= 0; i<=_state_index_lim; i++) {
            if (_KHP[i][i] > _P[i][i]) {
                healthyFusion = false;
            }
        }
        if (healthyFusion) {
            // update the covariance matrix
            for (uint8_t i= 0; i<=_state_index_lim; i++) {
                for (uint8_t j= 0; j<=_state_index_lim; j++) {
                    _P[i][j] = _P[i][j] - _KHP[i][j];
                }
            }

            // force the covariance matrix to be symmetrical and limit the variances to prevent ill-conditioning.
            ekf3_core_force_symmetry();
            ekf3_core_constrain_variances();

            // correct the state vector
            for (uint8_t j= 0; j<=_state_index_lim; j++) {
                _s.states_array[j] = _s.states_array[j] - _Kfusion[j] * ((float *)(&_innov_mag.x))[obsIndex];
            }

            // add table constraint here for faster convergence
            if (_have_table_earth_field && _mag_ef_limit > 0) {
                ekf3_core_mag_table_constrain();
            }

            quat_normalize(&_s.state_struct.quat);

        } else {
            // record bad axis
            if (obsIndex == 0) {
                _fault_status.bad_xmag = true;
            } else if (obsIndex == 1) {
                _fault_status.bad_ymag = true;
            } else if (obsIndex == 2) {
                _fault_status.bad_zmag = true;
            }
            ekf3_core_covariance_init();
            return;
        }
    }
}

void ekf3_core_select_mag_fusion(void)
{
    _mag_fuse_performed = false;
    const source_yaw_t yaw_source = ekf_source_get_yaw_source();
    if (yaw_source != _yaw_source_last) {
        MY_LOG("yaw source != yaw source last\n");
        _yaw_source_last = yaw_source;
        _yaw_source_reset = true;
    }
    if (!_on_ground_not_moving) {
        //MY_LOG("!on ground not moving\n");
        if (fabsf(_prev_tnb.a.z) < fabsf(_prev_tnb.b.z)) {
            _yaw_ang_data_static.order = TAIT_BRYAN_321;
            _yaw_ang_data_static.yaw_ang = atan2f(_prev_tnb.a.y, _prev_tnb.a.x);
        } else {
            _yaw_ang_data_static.order = TAIT_BRYAN_312;
            _yaw_ang_data_static.yaw_ang = atan2f(-_prev_tnb.b.x, _prev_tnb.b.y);
        }
        _yaw_ang_data_static.yaw_ang_err = MAX(_yaw_noise, 0.05f);
        _yaw_ang_data_static.time_ms = _imu_data_delayed.time_ms;
    }
    if (_mag_health) {
        _mag_timeout = false;
        _last_healthy_mag_time_ms = _imu_sample_time_ms;
    } else if ((_imu_sample_time_ms - _last_healthy_mag_time_ms) > _mag_fail_time_limit_ms &&
               ekf3_core_use_compass()) {
        //MY_LOG("select mag: mag unhealth\n");
        _mag_timeout = true;
    }
    ekf3_core_read_mag_data();
    _mag_data_to_fuse = ekf_ring_buffer_recall(&_stored_mag, &_mag_data_delayed, _imu_data_delayed.time_ms);
    if (_mag_data_to_fuse) {
        //MY_LOG("now %ld, mag data %ld imudata delay %ld\n", xtimer_now().ticks32 / 1000, _mag_data_delayed.time_ms, _imu_data_delayed.time_ms);
        if (_yaw_source_reset && (yaw_source == YAW_COMPASS || yaw_source == YAW_GPS_COMPASS_FALLBACK)) {
            _mag_yaw_reset_request = true;
            _yaw_source_reset = false;
        }
        control_mag_yaw_reset();
    }
    bool data_ready = (_mag_data_to_fuse && _states_initialised && ekf3_core_use_compass() &&
    _yaw_align_complete);
    if (data_ready) {
        if (_inhibit_mag_states || _mag_state_reset_request || !_mag_state_init_complete) {
            fuse_euler_yaw(YAW_FUSION_MAGNETOMETER);
            v3f_zero(&_mag_test_ratio);
        } else {
            if (_pv_aiding_mode != AID_ABSOLUTE ||
            (_mag_ef_limit > 0 && _have_table_earth_field)) {
                ekf3_core_fuse_declination(0.34);
            }
            ekf3_core_fuse_magnetometer();
            _yaw_test_ratio = 0;
        }
    }
    /* MY_LOG("mag field learned %d, final_inflight mag init %d\n", _mag_field_learned, */
    /*        _final_inflight_mag_init); */
    if (!_mag_field_learned && _final_inflight_mag_init) {
        _mag_field_learned = (_P[16][16] < sq(0.01)) && (_P[17][17] < sq(0.01)) &&
            (_P[18][18] < sq(0.01));
    }
    if (_mag_field_learned && !_inhibit_mag_states) {
        _earth_mag_field_var.x = _P[16][16];
        _earth_mag_field_var.y = _P[17][17];
        _earth_mag_field_var.z = _P[18][18];
        _body_mag_field_var.x = _P[19][19];
        _body_mag_field_var.y = _P[20][20];
        _body_mag_field_var.z = _P[21][21];
    }
}

void ekf3_core_reset_quat_state_yaw_only(float yaw, float yaw_variance, rotation_order_t order)
{
    quaternionf_t quat_before_reset = _s.state_struct.quat;
    quaternionf_t tmp = quat_inverse(&quat_before_reset);
    quat_to_rotation_matrix(&tmp, &_prev_tnb);
    vector3f_t euler_angles;
    if (order == TAIT_BRYAN_321) {
        quat_to_euler(&_s.state_struct.quat, &euler_angles.x, &euler_angles.y, &euler_angles.z);
        quat_from_euler(&_s.state_struct.quat, euler_angles.x, euler_angles.y, yaw);
    } else if (order == TAIT_BRYAN_312) {
        euler_angles = quat_to_vector312(&_s.state_struct.quat);
        quat_from_euler(&_s.state_struct.quat, euler_angles.x, euler_angles.y, yaw);
    } else {
        return;
    }
    tmp = quat_inverse(&_s.state_struct.quat);
    quat_to_rotation_matrix(&tmp, &_prev_tnb);
    float delta_yaw = wrap_PI(yaw - euler_angles.z);
    quaternionf_t quat_delta = quat_div(&_s.state_struct.quat, &quat_before_reset);
    ekf3_core_store_quat_rotate(&quat_delta);
    vector3f_t angle_err_var_vec = {0.5 * _tilt_error_variance, 0.5 * _tilt_error_variance,
yaw_variance};
    ekf3_core_covariance_prediction(&angle_err_var_vec);
    _yaw_reset_angle += delta_yaw;
    _last_yaw_reset_ms = _imu_sample_time_ms;
    ekf3_core_record_yaw_reset();
    _gps_yaw_reset_request = false;
    _mag_yaw_reset_request = false;
}

void ekf3_core_record_mag_reset(void)
{
    _mag_state_reset_request = false;
    _mag_state_init_complete = true;
    if (_in_flight) {
        _final_inflight_mag_init = true;
    }
    _pos_down_at_last_mag_reset = _s.state_struct.position.z;
    _quat_at_last_mag_reset = _s.state_struct.quat;
    _yaw_innov_at_last_mag_reset = _innov_yaw;
}

void ekf3_core_realign_yaw_gps(void)
{
    vector3f_t euler_angles;
    quat_to_euler(&_s.state_struct.quat, &euler_angles.x, &euler_angles.y, &euler_angles.z);
    if ((sq(_gps_data_delayed.vel.x) + sq(_gps_data_delayed.vel.y)) > 25.0f) {
        float vel_yaw = atan2f(_s.state_struct.velocity.y, _s.state_struct.velocity.x);
        float gps_yaw = atan2f(_gps_data_delayed.vel.y, _gps_data_delayed.vel.x);
        float yaw_err = MAX(fabsf(wrap_PI(gps_yaw - vel_yaw)), fabsf(wrap_PI(gps_yaw - euler_angles.x)));
        bool bad_mag_yaw = ((yaw_err > 0.7854) && (_vel_test_ratio > 1.0f) && (_pv_aiding_mode == AID_ABSOLUTE)) ||
            !_yaw_align_complete;
        if (bad_mag_yaw) {
            rotation_order_t order;
            ekf3_core_best_rotation_order(&order);
            ekf3_core_reset_quat_state_yaw_only(gps_yaw, sq(radians(45)), order);
            ekf3_core_reset_velocity(RESET_DATA_SOURCE_GPS);
            ekf3_core_reset_position(RESET_DATA_SOURCE_GPS);
            MY_LOG("EKF3 IMU0 yaw aligned to GPS velocity\n");
            if (ekf3_core_use_compass()) {
                _mag_state_reset_request = true;
                _all_mag_sensors_failed = false;
            }
        }
    }
}
