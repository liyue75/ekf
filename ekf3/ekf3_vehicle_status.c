#include <math.h>
#include "ekf3.h"
#include "ekf3_core.h"
#include "fusion_math.h"
#include "vector3f.h"
#include "location.h"
//#include "gps.h"
#include "gps_dal.h"

extern bool _in_flight;
extern bool _gps_good_to_align;
extern int16_t _gps_check_scaler;
extern vector3f_t _mag_test_ratio;
extern bool _consistent_mag_data;
extern uint32_t _mag_yaw_reset_timer_ms;
extern uint32_t _imu_sample_time_ms;
extern bool _motors_armed;
extern bool _mag_yaw_reset_request;
//extern gps_status_t _gps_state;
extern imu_elements_t _imu_data_delayed;
extern uint32_t _last_pre_align_gps_check_time_ms;
extern float _gps_drift_ne;
extern float _gps_ver_vel_filt;
extern float _gps_horiz_vel_filt;
extern location_t _gps_loc_prev;
extern bool _on_ground;
extern int8_t _gps_check;
extern gps_check_status_t _gps_check_status;
extern gps_elements_t _gps_data_new;
extern float _gps_vert_vel_filt;
extern float _gps_horiz_vel_filt;
extern gps_elements_t _gps_data_delayed;
extern float _gps_spd_accuracy;
extern uint32_t _last_gps_vel_fail_ms;
extern uint32_t _last_gps_vel_pass_ms;
extern bool _gps_spd_acc_pass;
extern bool _ekf_innovations_pass;
extern float _s_acc_filter_state1;
extern float _s_acc_filter_state2;
extern uint32_t _last_gps_check_time_ms;
extern uint32_t _last_innov_pass_time_ms;
extern uint32_t _last_innov_fail_time_ms;
extern bool _gps_accuracy_good;
extern vector3f_t _gps_vel_innov;
extern vector3f_t _gps_vel_var_innov;
extern uint32_t _gps_vel_innov_time_ms;
extern float _vel_test_ratio;
extern float _pos_test_ratio;
extern float _hgt_test_ratio;

void ekf3_core_calc_gps_good_to_align(void)
{
    if (_in_flight && !ekf3_core_use_compass()) {
        _gps_good_to_align = true;
        return;
    }
    float check_scaler = 0.01f * _gps_check_scaler;
    if (_gps_good_to_align) {
        check_scaler *= 1.3f;
    }
    if ((_mag_test_ratio.x <= 1.0f && _mag_test_ratio.y <= 1.0f && _mag_test_ratio.z <= 1.0f) ||
    (!_consistent_mag_data)) {
        _mag_yaw_reset_timer_ms = _imu_sample_time_ms;
    }
    if ((_imu_sample_time_ms - _mag_yaw_reset_timer_ms > 5000) && !_motors_armed) {
        _mag_yaw_reset_request = true;
        _mag_yaw_reset_timer_ms = _imu_sample_time_ms;
    }
    const location_t gps_loc = *dal_gps_location();
    const float pos_filt_time_const = 10.0;
    float delta_time = constrain_float((float)(_imu_data_delayed.time_ms -
                                               _last_pre_align_gps_check_time_ms) * 0.001f,
                                       0.01f, pos_filt_time_const);
    _last_pre_align_gps_check_time_ms = _imu_data_delayed.time_ms;
    _gps_drift_ne += location_get_distance(&_gps_loc_prev, &gps_loc);
    _gps_loc_prev = gps_loc;
    _gps_drift_ne *= (1.0f - delta_time / pos_filt_time_const);
    _gps_drift_ne = MIN(_gps_drift_ne, 10.0f);
    bool gps_drift_fail = (_gps_drift_ne > 3.0f * check_scaler) && _on_ground &&
        (_gps_check & MASK_GPS_POS_DRIFT);
    if (gps_drift_fail) {
        _gps_check_status.bad_horiz_drift = true;
    } else {
        _gps_check_status.bad_horiz_drift = false;
    }
    bool gps_vert_vel_fail;
    if (_gps_data_new.have_vz && _on_ground) {
        _gps_vert_vel_filt = 0.1f * _gps_data_new.vel.z + 0.9f * _gps_vert_vel_filt;
        _gps_vert_vel_filt = constrain_float(_gps_vert_vel_filt, -10.0f, 10.0f);
        gps_vert_vel_fail = (fabsf(_gps_vert_vel_filt) > 0.3f * check_scaler) && (_gps_check &
            MASK_GPS_VERT_SPD);
    } else {
        gps_vert_vel_fail = false;
    }
    if (gps_vert_vel_fail) {
        _gps_check_status.bad_vert_vel = true;
    } else {
        _gps_check_status.bad_vert_vel = false;
    }
    bool gps_horiz_vel_fail = false;
    if (_on_ground) {
        _gps_horiz_vel_filt = 0.1f * norm_2f(_gps_data_delayed.vel.x,
                                             _gps_data_delayed.vel.y) + 0.9f * _gps_horiz_vel_filt;
        _gps_horiz_vel_filt = constrain_float(_gps_horiz_vel_filt, -10.0f, 10.0f);
        gps_horiz_vel_fail = (fabsf(_gps_horiz_vel_filt) > 0.3f * check_scaler) && (
            _gps_check & MASK_GPS_HORIZ_SPD);
    }
    if (gps_horiz_vel_fail) {
        _gps_check_status.bad_horiz_vel = true;
    } else {
        _gps_check_status.bad_horiz_vel = false;
    }
    float h_acc = 0.0f;
    bool h_acc_fail = false;
    if (dal_gps_horizontal_accuracy(&h_acc)) {
        h_acc_fail = (h_acc > 5.0f * check_scaler) && (_gps_check & MASK_GPS_POS_ERR);
    }
    if (h_acc_fail) {
        _gps_check_status.bad_h_acc = true;
    } else {
        _gps_check_status.bad_h_acc = false;
    }
    float v_acc = 0.0f;
    bool v_acc_fail = false;
    if (dal_gps_vertical_accuracy(&v_acc)) {
        v_acc_fail = (v_acc > 7.5f * check_scaler) && (_gps_check & MASK_GPS_POS_ERR);
    }
    if (v_acc_fail) {
        _gps_check_status.bad_v_acc = true;
    } else {
        _gps_check_status.bad_v_acc = false;
    }
    bool gps_spd_acc_fail = (_gps_spd_accuracy > 1.0f * check_scaler) && (_gps_check &
                                                                          MASK_GPS_SPD_ERR);
    if (gps_spd_acc_fail) {
        _gps_check_status.bad_s_acc = true;
    } else {
        _gps_check_status.bad_s_acc = false;
    }
    bool hdop_fail = (dal_gps_get_hdop() > 250) && (_gps_check & MASK_GPS_HDOP);
    if (hdop_fail) {
        _gps_check_status.bad_hdop = true;
    } else {
        _gps_check_status.bad_hdop = false;
    }
    bool num_sats_fail = (dal_gps_num_sats() < 6) && (_gps_check & MASK_GPS_NSATS);
    if (num_sats_fail) {
        _gps_check_status.bad_sats = true;
    } else {
        _gps_check_status.bad_sats = false;
    }
    bool yaw_fail;
    if ((_mag_test_ratio.x > 1.0f || _mag_test_ratio.y > 1.0f || _mag_test_ratio.z > 1.0f) &&
    (_gps_check & MASK_GPS_YAW_ERR)) {
        yaw_fail = true;
    } else {
        yaw_fail = false;
    }
    if (yaw_fail) {
        _gps_check_status.bad_yaw = true;
    } else {
        _gps_check_status.bad_yaw = false;
    }
    if (_last_gps_vel_fail_ms == 0) {
        _last_gps_vel_fail_ms = _imu_sample_time_ms;
    }
    if (gps_spd_acc_fail || num_sats_fail || hdop_fail || h_acc_fail || v_acc_fail ||
    yaw_fail || gps_drift_fail || gps_vert_vel_fail || gps_horiz_vel_fail) {
        _last_gps_vel_fail_ms = _imu_sample_time_ms;
    } else {
        _last_gps_vel_pass_ms = _imu_sample_time_ms;
    }
    if (!_gps_good_to_align && _imu_sample_time_ms - _last_gps_vel_fail_ms > 10000) {
        _gps_good_to_align = true;
    } else if (_gps_good_to_align && _imu_sample_time_ms - _last_gps_vel_pass_ms > 5000) {
        _gps_good_to_align = false;
    }
}

void ekf3_core_calc_gps_good_for_flight(void)
{
    const float alpha1 = 0.2f;
    const float tau = 10.0f;
    if (_last_gps_check_time_ms == 0) {
        _last_gps_check_time_ms = _imu_sample_time_ms;
    }
    float dt_lpf = (_imu_sample_time_ms - _last_gps_check_time_ms) * 1e-3f;
    _last_gps_check_time_ms = _imu_sample_time_ms;
    float alpha2 = constrain_float(dt_lpf / tau, 0.0f, 1.0f);
    float gps_spd_acc_raw;
    if (!dal_gps_speed_accuracy(&gps_spd_acc_raw)) {
        gps_spd_acc_raw = 0.0f;
    }
    _s_acc_filter_state1 = constrain_float((alpha1 * gps_spd_acc_raw +
                                            (1.0f - alpha1) * _s_acc_filter_state1),
                                           0.0f, 10.0f);
    _s_acc_filter_state2 = MAX(_s_acc_filter_state1, ((1.0f - alpha2) * _s_acc_filter_state2));
    if (_s_acc_filter_state2 > 1.5f) {
        _gps_spd_acc_pass = false;
    } else if (_s_acc_filter_state2 < 1.0f) {
        _gps_spd_acc_pass = true;
    }
    if (_last_innov_fail_time_ms == 0) {
        _last_innov_fail_time_ms = _imu_sample_time_ms;
        _last_innov_pass_time_ms = _imu_sample_time_ms;
    }
    if (_vel_test_ratio < 1.0f && _pos_test_ratio < 1.0f) {
        _last_innov_pass_time_ms = _imu_sample_time_ms;
    } else if (_vel_test_ratio > 0.7f || _pos_test_ratio > 0.7f) {
        _last_innov_fail_time_ms = _imu_sample_time_ms;
    }
    if ((_imu_sample_time_ms - _last_innov_pass_time_ms) > 1000) {
        _ekf_innovations_pass = false;
    } else if ((_imu_sample_time_ms - _last_innov_fail_time_ms) > 10000){
        _ekf_innovations_pass = true;
    }
    _gps_accuracy_good = _gps_spd_acc_pass && _ekf_innovations_pass;
}
