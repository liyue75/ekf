#include "ekf3_core.h"
#include "location.h"
//#include "compass.h"
#include "compass_dal.h"
#include "fusion_math.h"
#include "uart_device.h"
#include "board_led.h"

extern bool _tilt_align_complete;
extern bool _yaw_align_complete;
extern bool _gnd_offset_valid;
extern bool _gps_accuracy_good;
extern bool _vel_timeout;
extern bool _hgt_timeout;
extern bool _pos_timeout;
extern bool _tas_timeout;
extern bool _flow_data_valid;
extern bool _use_gps_vert_vel;
extern Matrix24 _P;
extern state_var_t _s;
extern bool _mag_field_learned;
extern bool _valid_origin;
extern location_t _ekf_origin;
extern location_t *_public_origin;
extern double _ekf_gps_ref_hgt;
extern vector3f_t _earth_rate_ned;
extern bool _common_origin_valid;
extern bool _all_mag_sensors_failed;
extern int8_t _mag_mask;
extern int8_t _mag_cal;
extern bool _inhibit_wind_states;
extern bool _inhibit_mag_states;
extern bool _inhibit_del_vel_bias_states;
extern bool _inhibit_del_ang_bias_states;
extern uint8_t _state_index_lim;
extern bool _manoeuvring;
extern float _acc_nav_mag_horiz;
extern mag_cal_t _effective_mag_cal;
extern bool _final_inflight_yaw_init;
extern bool _final_inflight_mag_init;
extern bool _in_flight;
extern bool _on_ground;
extern vector3f_t _earth_mag_field_var;
extern vector3f_t _body_mag_field_var;
extern float _mag_noise;
extern bool _mag_state_init_complete;
extern bool _final_inflight_mag_init;
extern bool _mag_yaw_reset_request;
extern bool _tilt_align_complete;
extern float _acc_bias_lim;
extern float _dt_ekf_avg;
extern float _tilt_error_variance;
extern bool _yaw_align_complete;
extern aiding_mode_t _pv_aiding_mode;
extern aiding_mode_t _pv_aiding_mode_prev;
extern matrix3f_t _prev_tnb;
extern bool _del_ang_bias_learned;
extern bool _valid_origin;
extern bool _gps_data_to_fuse;
extern source_yaw_t _yaw_source_last;
extern bool _gps_good_to_align;
extern bool _motors_armed;
extern uint32_t _imu_sample_time_ms;
extern uint32_t _prev_body_vel_fuse_time_ms;
extern const uint16_t _tilt_drift_time_max_ms;
extern const uint16_t _pos_retry_time_use_vel_ms;
extern const uint16_t _pos_retry_time_no_vel_ms;
extern uint32_t _last_vel_pass_time_ms;
extern uint32_t _last_pos_pass_time_ms;
extern bool _pos_timeout;
extern bool _vel_timeout;
extern bool _gps_in_use;
extern float _vel_test_ratio;
extern float _pos_test_ratio;
extern vector2f_t _last_known_position_ne;
extern float _mea_hgt_at_take_off;
extern bool _body_vel_fusion_active;
extern gps_elements_t _gps_data_delayed;
extern nav_filter_status_t _filter_status;

bool _ekf3_start_using_gps = false;

bool ekf3_core_use_compass(void)
{
    source_yaw_t yaw_source = ekf3_get_yaw_source();
    if (yaw_source != YAW_COMPASS && yaw_source != YAW_GPS_COMPASS_FALLBACK) {
        return false;
    }
    return dal_compass_use_for_yaw() && !_all_mag_sensors_failed;
}

bool ekf3_core_set_origin(const location_t *loc)
{
    //MY_LOG("ekf3 core set origin\n");
    if (_valid_origin) {
        return false;
    }
    _ekf_origin = *loc;
    _ekf_gps_ref_hgt = (double)0.01 * (double)_ekf_origin.alt;
    ekf3_core_calc_earth_rate_ned(&_earth_rate_ned, _ekf_origin.lat);
    _valid_origin = true;
    MY_LOG("EKF3 IMU0 origin set, _ekf_origin: %ld %ld %ld\n", _ekf_origin.lat, _ekf_origin.lng,
           _ekf_origin.alt);
    if (!_common_origin_valid) {
        _common_origin_valid = true;
        _public_origin = &_ekf_origin;
    }
    return true;
}

mag_cal_t ekf3_core_effective_mag_cal(void)
{
    if (_mag_mask & 0) {
        return MAG_CAL_NEVER;
    }
    const int8_t mag_cal_param_val = _mag_cal;
    if (mag_cal_param_val == 5) {
        return MAG_CAL_NEVER;
    }
    if (mag_cal_param_val == 6) {
        return MAG_CAL_WHEN_FLYING;
    }
    return mag_cal_param_val;
}

static void update_state_index_lim(void)
{
    if (_inhibit_wind_states) {
        if (_inhibit_mag_states) {
            if (_inhibit_del_vel_bias_states) {
                if (_inhibit_del_ang_bias_states) {
                    _state_index_lim = 9;
                } else {
                    _state_index_lim = 12;
                }
            } else {
                _state_index_lim = 15;
            }
        } else {
            _state_index_lim = 21;
        }
    } else {
        _state_index_lim = 23;
    }
}

static void set_wind_mag_state_learning_mode(void)
{
    const bool can_estimated_wind = false;
    if (!_inhibit_wind_states && !can_estimated_wind) {
        //MY_LOG("inhibit_wind states");
        _inhibit_wind_states = true;
        update_state_index_lim();
    }
    /* else if (_inhibit_wind_states && can_estimated_wind && */
    /*          (sq(_s.state_struct.velocity.x) + sq(_s.state_struct.velocity.y) > sq(5.0f) || */
    /*          _drag_fusion_enabled)) { */

    /* } */
    _manoeuvring = _acc_nav_mag_horiz > 0.5f;
    //MY_LOG("acc_anv_mag_horiz %f\n", _acc_nav_mag_horiz);
    bool mag_cal_requested = ((_effective_mag_cal == MAG_CAL_WHEN_FLYING) && _in_flight) ||
        ((_effective_mag_cal == MAG_CAL_WHEN_MANOEUVRING) && _manoeuvring) ||
        ((_effective_mag_cal == MAG_CAL_AFTER_FIRST_CLIMB) && _final_inflight_yaw_init && _final_inflight_mag_init) ||
        (_effective_mag_cal == MAG_CAL_ALWAYS);
    bool mag_denied =  !ekf3_core_use_compass() || (_effective_mag_cal == MAG_CAL_NEVER) ||
        (_on_ground && _effective_mag_cal != MAG_CAL_ALWAYS);
    bool set_mag_inhibit = !mag_cal_requested || mag_denied;
    if (!_inhibit_mag_states && set_mag_inhibit) {
        _inhibit_mag_states = true;
        update_state_index_lim();
    } else if (_inhibit_mag_states && !set_mag_inhibit) {
        _inhibit_mag_states = false;
        update_state_index_lim();
        if (_mag_field_learned) {
            _P[16][16] = _earth_mag_field_var.x;
            _P[17][17] = _earth_mag_field_var.y;
            _P[18][18] = _earth_mag_field_var.z;
            _P[19][19] = _body_mag_field_var.x;
            _P[20][20] = _body_mag_field_var.y;
            _P[21][21] = _body_mag_field_var.z;
        } else {
            for (uint8_t i = 16; i <= 21; i++) {
                _P[i][i] = sq(_mag_noise);
            }
            ekf3_core_align_mag_state_declination();
        }
        if (!_mag_state_init_complete || (!_final_inflight_mag_init && _in_flight)) {
            _mag_yaw_reset_request = true;
        }
    }
    if (_tilt_align_complete && _inhibit_del_vel_bias_states) {
        _inhibit_del_vel_bias_states = false;
        update_state_index_lim();
        _P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * _acc_bias_lim * _dt_ekf_avg);
        _P[14][14] = _P[13][13];
        _P[15][15] = _P[13][13];
    }
    if (_tilt_align_complete && _inhibit_del_ang_bias_states) {
        _inhibit_del_ang_bias_states = false;
        update_state_index_lim();
        _P[10][10] = sq(radians(ekf3_core_initial_gyro_bias_uncertainty() * _dt_ekf_avg));
        _P[11][11] = _P[10][10];
        _P[12][12] = _P[10][10];
    }
    if (_on_ground) {
        _final_inflight_yaw_init = false;
        _final_inflight_mag_init = false;
        _mag_field_learned = false;
    }
    update_state_index_lim();
}

static void check_attitude_alignment_status(void)
{
    if (!_tilt_align_complete) {
        if (_tilt_error_variance < sq(radians(5.0))) {
            _tilt_align_complete = true;
            MY_LOG("EKF3 IMU tilt alignment complete\n");
        }
    }
    if (!_yaw_align_complete && _tilt_align_complete && ekf3_core_use_compass()) {
        _mag_yaw_reset_request = true;
    }
}

static void check_gyro_cal_status(void)
{
    const float del_ang_bias_var_max = sq(radians(0.15 * _dt_ekf_avg));
    const source_yaw_t yaw_source= ekf_source_get_yaw_source();
    if (!ekf3_core_use_compass() && (yaw_source != YAW_GPS) && (yaw_source != YAW_GPS_COMPASS_FALLBACK) &&
    (yaw_source != YAW_EXTNAV)) {
        vector3f_t del_ang_bias_var_vec = {_P[10][10], _P[11][11], _P[12][12]};
        vector3f_t tmp = m3f_multi_v(&_prev_tnb, &del_ang_bias_var_vec);
        _del_ang_bias_learned = (fabsf(tmp.x) < del_ang_bias_var_max) &&
            (fabsf(tmp.y) < del_ang_bias_var_max);
    } else {
        _del_ang_bias_learned = (_P[10][10] <= del_ang_bias_var_max) &&
            (_P[11][11] <= del_ang_bias_var_max) &&
            (_P[11][11] <= del_ang_bias_var_max);
    }
}

static bool ready_to_use_gps(void)
{
    if (ekf_source_get_posxy_source() != XY_GPS) {
        return false;
    }
    /* MY_LOG("ready_to use gps %d %d %d %d %d %d %d\n", */
    /*        _valid_origin, _tilt_align_complete, _yaw_align_complete, */
    /*        _del_ang_bias_learned, ekf3_core_assume_zero_sideslip(), */
    /*        _gps_good_to_align, _gps_data_to_fuse); */
    return  _valid_origin && _tilt_align_complete && _yaw_align_complete &&
        (_del_ang_bias_learned || ekf3_core_assume_zero_sideslip())
        && _gps_good_to_align && _gps_data_to_fuse;
}

static void set_aiding_mode(void)
{
    reset_data_source_t pos_reset_source = RESET_DATA_SOURCE_DEFAULT;
    reset_data_source_t vel_reset_source = RESET_DATA_SOURCE_DEFAULT;
    _pv_aiding_mode_prev = _pv_aiding_mode;
    check_gyro_cal_status();
    //MY_LOG("_P[10][10] %f del ang bias learned %d\n", _P[10][10], _del_ang_bias_learned);
    //MY_LOG("yaw source last %d\n", _yaw_source_last);
    if (_yaw_source_last == YAW_NONE && !_motors_armed && _on_ground && _pv_aiding_mode != AID_NONE) {
        MY_LOG("set aiding mode: yaw none\n");
    }
    switch(_pv_aiding_mode) {
        case AID_NONE: {
            if (ready_to_use_gps()) {
                _pv_aiding_mode = AID_ABSOLUTE;
            }
            break;
        }
        case AID_RELATIVE: {
            if (ready_to_use_gps()) {
                _pv_aiding_mode = AID_ABSOLUTE;
            }
            break;
        }
        case AID_ABSOLUTE: {
            uint16_t min_test_time_ms = MIN(_tilt_drift_time_max_ms,
                                            MIN(_pos_retry_time_use_vel_ms,
                                                _pos_retry_time_no_vel_ms));
            bool pos_used = (_imu_sample_time_ms - _last_pos_pass_time_ms <= min_test_time_ms);
            bool gps_vel_used = (_imu_sample_time_ms - _last_vel_pass_time_ms <= min_test_time_ms);
            bool att_aiding = pos_used || gps_vel_used;
            bool vel_aiding = gps_vel_used;
            bool pos_aiding = pos_used;
            bool att_aid_loss_critical = false;
            if (!att_aiding) {
                att_aid_loss_critical = (_imu_sample_time_ms - _last_pos_pass_time_ms > _tilt_drift_time_max_ms) &&
                    (_imu_sample_time_ms - _last_vel_pass_time_ms > _tilt_drift_time_max_ms); //15s
            }
            bool pos_aid_loss_critical = false;
            if (!pos_aiding) {
                uint16_t max_loss_time_ms;
                if (!vel_aiding) {
                    max_loss_time_ms = _pos_retry_time_no_vel_ms; //7s
                } else {
                    max_loss_time_ms = _pos_retry_time_use_vel_ms; //10s
                }
                pos_aid_loss_critical = (_imu_sample_time_ms - _last_pos_pass_time_ms > max_loss_time_ms);
            }
            if (att_aid_loss_critical) {
                MY_LOG("att aid loss critical\n");
                _pv_aiding_mode = AID_NONE;
                _pos_timeout = true;
                _vel_timeout = true;
                _gps_in_use = false;
            } else if (pos_aid_loss_critical) {
                MY_LOG("pos aid loss critical\n");
                _pos_timeout = true;
                _vel_timeout = !gps_vel_used;
                _gps_in_use = false;
            }
            break;
        }
    }
    //MY_LOG("pv aiding mode %d\n", _pv_aiding_mode);
    if (_pv_aiding_mode != _pv_aiding_mode_prev) {
        switch (_pv_aiding_mode) {
            case AID_NONE:
                MY_LOG("EKF3 IMU stopped aiding");
                _pos_timeout = true;
                _vel_timeout = true;
                _vel_test_ratio = 0;
                _pos_test_ratio = 0;
                _last_known_position_ne.x = _s.state_struct.position.x;
                _last_known_position_ne.y = _s.state_struct.position.y;
                //baro_data_delayed.hgt
                _mea_hgt_at_take_off = _gps_data_delayed.hgt;
                _s.state_struct.position.z = -_mea_hgt_at_take_off;
                _body_vel_fusion_active = false;
                break;
            case AID_RELATIVE:
                MY_LOG("EKF3 IMU started relative aiding\n");
                _pos_timeout = true;
                _vel_timeout = true;
                break;
            case AID_ABSOLUTE:
                if (ready_to_use_gps()) {
                    pos_reset_source = RESET_DATA_SOURCE_GPS;
                    vel_reset_source = RESET_DATA_SOURCE_GPS;
                    MY_LOG("EKF3 IMU is using GPS\n");
                }
                _pos_timeout = false;
                _vel_timeout = false;
                _last_pos_pass_time_ms = _imu_sample_time_ms;
                _last_vel_pass_time_ms = _imu_sample_time_ms;
                _ekf3_start_using_gps = true;
                led_on(LED_1);
                break;
        }
        ekf3_core_reset_velocity(vel_reset_source);
        ekf3_core_reset_position(pos_reset_source);
    }
}

void ekf3_core_control_filter_modes(void)
{
    ekf3_core_detect_flight();
    set_wind_mag_state_learning_mode();
    check_attitude_alignment_status();
    set_aiding_mode();
}

void ekf3_core_record_yaw_reset(void)
{
    _yaw_align_complete = true;
    if (_in_flight) {
        _final_inflight_yaw_init = true;
    }
}

void ekf3_core_update_filter_status(void)
{
    _filter_status.value = 0;
    bool doing_body_vel_nav = (_pv_aiding_mode != AID_NONE) && (_imu_sample_time_ms -
                                                                _prev_body_vel_fuse_time_ms < 5000);
    bool doing_flow_nav = (_pv_aiding_mode != AID_NONE) && (_flow_data_valid);
    bool doing_wind_rel_nav = (!_tas_timeout) && ekf3_core_assume_zero_sideslip();
    bool doing_normal_gps_nav = !_pos_timeout && (_pv_aiding_mode == AID_ABSOLUTE);
    bool some_vert_ref_data = (!_vel_timeout && (_use_gps_vert_vel)) || !_hgt_timeout;
    bool some_horiz_ref_data = !(_vel_timeout && _pos_timeout && _tas_timeout) ||
        doing_body_vel_nav || doing_flow_nav;
    bool filter_healthy = ekf3_core_healthy() && _tilt_align_complete && (_yaw_align_complete ||
    (!ekf3_core_use_compass() && (_pv_aiding_mode != AID_ABSOLUTE)));
    bool hgt_not_accurate = ekf_source_get_posz_source() == Z_GPS && !_valid_origin;

    _filter_status.flags.attitude = filter_healthy;
    _filter_status.flags.horiz_vel = some_horiz_ref_data && filter_healthy;
    _filter_status.flags.vert_vel = some_vert_ref_data && filter_healthy;
    _filter_status.flags.horiz_pos_rel = ((doing_flow_nav && _gnd_offset_valid) || doing_wind_rel_nav ||
                                          doing_normal_gps_nav || doing_body_vel_nav) && filter_healthy;
    _filter_status.flags.horiz_pos_abs = doing_normal_gps_nav && filter_healthy;
    _filter_status.flags.vert_pos = !_hgt_timeout && filter_healthy && !hgt_not_accurate;
    _filter_status.flags.terrain_alt = _gnd_offset_valid && filter_healthy;
    _filter_status.flags.const_pos_mode = (_pv_aiding_mode == AID_NONE) && filter_healthy;
    _filter_status.flags.pred_horiz_pos_rel = _filter_status.flags.horiz_pos_rel;
    _filter_status.flags.pred_horiz_pos_abs = _filter_status.flags.horiz_pos_abs;
    _filter_status.flags.takeoff_detected = false;
    _filter_status.flags.takeoff = false;
    _filter_status.flags.touchdown = false;
    _filter_status.flags.using_gps = ((_imu_sample_time_ms - _last_pos_pass_time_ms) < 4000) &&
        (_pv_aiding_mode == AID_ABSOLUTE);
    _filter_status.flags.gps_glitching = !_gps_accuracy_good && (_pv_aiding_mode == AID_ABSOLUTE) &&
        ekf_source_get_posxy_source() == XY_GPS;
    _filter_status.flags.gps_quality_good = _gps_good_to_align;
    _filter_status.flags.initialized = ekf3_core_healthy();
}
