#include <math.h>
#include <string.h>
#include "ekf3_core.h"
#include "location.h"
//#include "inertial_sensor.h"
#include "dal.h"
#include "ins_dal.h"
#include "fusion_math.h"
//#include "gps.h"
#include "gps_dal.h"
#include "ekf_buffer.h"
#include "ahrs.h"
#include "board_led.h"
#include "ekf_source.h"
#include "nav_common.h"
#include "uart_device.h"
#include "dal.h"
#include "inertial_sensor.h"

#include "xtimer.h"
#define ENABLE_DEBUG 1
#include "debug.h"

extern uint64_t _imu_sample_time_us;
extern float _hrt_filt_freq;
extern int8_t _tau_vel_pos_output;
extern bool _common_origin_valid;
extern int16_t _mag_ef_limit;
static uint32_t first_init_time_ms;
static uint32_t last_init_fail_report_ms;
float _dt_ekf_avg;
float _dt;

bool _mag_fuse_performed;
mag_cal_t _effective_mag_cal;

uint8_t _imu_buffer_length;
uint8_t _obs_buffer_length;

extern bool fly_forward;
extern vehicle_class_t vehicle_class;
extern uint16_t mag_delay_ms;
extern uint8_t _sensor_interval_min_ms;
extern float _gyro_bias_process_noise;
extern float _accel_bias_process_noise;
extern float _mag_noise;
extern float _mag_earth_process_noise;
extern float _mag_body_process_noise;
extern float _wind_vel_process_noise;
extern float _wnd_var_hgt_rate_scale;
extern float _gyr_noise;
extern float _acc_noise;
extern float _gps_horiz_vel_noise;
extern float _gps_horiz_pos_noise;
extern float _yaw_noise;
extern float _acc_bias_lim;

bool _states_initialised;
bool _mag_timeout;
bool _mag_health;
bool _vel_timeout;
bool _hgt_timeout;
bool _pos_timeout;
bool _tas_timeout;
bool _bad_imu_data;
uint32_t _bad_imu_data_ms;
uint32_t _good_imu_data_ms;
uint32_t _vert_vel_var_clip_counter;

float _gps_noise_scaler;
Matrix24 _P;
Matrix24 _KH;
Matrix24 _KHP;
Matrix24 _nextP;
Vector28 _Kfusion;
ekf_imu_buffer_t _stored_imu;
ekf_ring_buffer_t _stored_gps;
ekf_ring_buffer_t _stored_mag;
ekf_imu_buffer_t _stored_output;
matrix3f_t _prev_tnb;
float _acc_nav_mag;
float _acc_nav_mag_horiz;
float _hgt_rate;
bool _in_flight;
bool _prev_in_flight;
bool _manoeuvring;
Vector6 _innov_vel_pos;
Vector6 _var_innov_vel_pos;
Vector6 _vel_pos_obs;
bool _fuse_vel_data;
bool _fuse_pos_data;
bool _fuse_hgt_data;
vector3f_t _innov_mag;
vector3f_t _var_innov_mag;
uint32_t _last_mag_update_us;
uint32_t _last_mag_read_ms;

vector3f_t _accel_pos_offset;
float _stored_rng_meas[2][3];
uint32_t _stored_rng_meas_time_ms[2][3];
uint8_t _rng_meas_index[2];
bool _terrain_hgt_stable;

uint32_t _last_body_vel_pass_time_ms;
Vector3 _body_vel_test_ratio;
Vector3 _var_innov_body_vel;
Vector3 _innov_body_vel;
uint32_t _prev_body_vel_fuse_time_ms;
uint32_t _body_odm_meas_time_ms;
bool _body_vel_fusion_delayed;
bool _body_vel_fusion_active;

vert_vel_t _vert_comp_filt_state;
float _baro_hgt_offset;
float _rng_on_gnd;
vector3f_t _earth_rate_ned;
float _dt_imu_avg;
uint32_t _imu_sample_time_ms;
bool _on_ground;
bool _on_ground_not_moving;
bool _prev_on_ground;
vector3f_t _gyro_prev;
vector3f_t _accel_prev;
uint32_t _last_move_check_log_time_ms;
float _gyro_diff;
float _accel_diff;
bool _all_mag_sensors_failed;

vector3f_t _del_ang_corrected;
vector3f_t _del_vel_corrected;
bool _mag_field_learned;
uint32_t _was_learning_compass_ms;
vector3f_t _earth_mag_field_var;
vector3f_t _body_mag_field_var;
bool _del_ang_bias_learned;

state_var_t _s;

imu_elements_t _imu_data_delayed;
imu_elements_t _imu_data_new;
imu_elements_t _imu_data_down_sampled_new;
quaternionf_t _imu_quat_down_sample_new = {1, 0, 0, 0};
uint32_t _frames_since_predict;
bool _start_predict_enabled;
uint8_t _mag_select_index;
ekf_timing_t _timing;
uint32_t _last_filter_ok_ms;
fault_status_t _fault_status;
mag_state_t _mag_state;

inactive_bias_t _inactive_bias;

extern location_t _location;
extern source_set_t _source_set;
uint32_t _last_launch_accel_time_ms;
float _vel_test_ratio;
float _pos_test_ratio;
float _hgt_test_ratio;
bool _inhibit_wind_states;
bool _wind_state_aligned;
bool _inhibit_mag_states;
bool _last_inhibit_mag_states;
bool _need_mag_body_var_reset;
bool _need_earth_body_var_reset;
bool _inhibit_del_ang_bias_states;
bool _gps_in_use;

uint32_t _last_mag_update_us;
uint32_t _last_mag_read_ms;
vector3f_t _last_mag_offsets;
bool _last_mag_offsets_valid;
vector3f_t _vel_dot_ned;
vector3f_t _vel_dot_ned_filt;

mag_elements_t _mag_data_delayed;
gps_elements_t _gps_data_new;
gps_elements_t _gps_data_delayed;
uint8_t _last_gps_idx;
output_elements_t _output_data_new;
output_elements_t _output_data_delayed;
vector3f_t _del_ang_correction;
vector3f_t _vel_err_integral;
vector3f_t _pos_err_integral;
uint32_t _last_time_gps_received_ms;
gps_check_status_t _gps_check_status;
uint8_t _local_filter_time_step_ms;
float _gps_spd_accuracy;
float _gps_pos_accuracy;
float _gps_hgt_accuracy;
bool _gps_data_to_fuse;
bool _mag_data_to_fuse;
aiding_mode_t _pv_aiding_mode;
aiding_mode_t _pv_aiding_mode_prev;
bool _gnd_offset_valid;
//vector3f_t _del_ang_body_of;
//float _del_time_of;
bool _flow_fusion_active;
float _pos_down_at_takeoff;
bool _use_gps_vert_vel;
float _yaw_reset_angle;
uint32_t _last_yaw_reset_ms;
bool _tilt_align_complete;
bool _yaw_align_complete;
bool _mag_state_init_complete;
float _bad_imu_vel_err_integral;
float _innov_yaw;
bool _gps_good_to_align;
uint32_t _mag_yaw_reset_timer_ms;
uint32_t _last_pre_align_gps_check_time_ms;
float _gps_drift_ne;
float _gps_vert_vel_filt;
float _gps_horiz_vel_filt;
location_t _gps_loc_prev;
uint32_t _last_gps_vel_fail_ms;
uint32_t _last_gps_vel_pass_ms;
bool _gps_spd_acc_pass;
bool _ekf_innovations_pass;
float _s_acc_filter_state1;
float _s_acc_filter_state2;
uint32_t _last_gps_check_time_ms;
uint32_t _last_innov_pass_time_ms;
uint32_t _last_innov_fail_time_ms;
bool _gps_accuracy_good;
vector3f_t _gps_vel_innov;
vector3f_t _gps_vel_var_innov;
uint32_t _gps_vel_innov_time_ms;
bool _gps_is_in_use;

bool _flow_data_valid;
float _P_opt;
float _terrain_state;
float _prev_pos_n;
float _prev_pos_e;
float _hgt_mea;
bool _inhibit_gnd_state;
vector2f_t _flow_gyro_bias;
bool _range_data_to_fuse;
vector3f_t _mag_test_ratio;
bool _consistent_mag_data;
uint32_t _mag_yaw_reset_timer_ms;
bool _motors_armed;
bool _prev_motors_armed;
bool _pos_vel_fusion_delayed;
bool _opt_flow_fusion_delayed;
bool _air_spd_fusion_delayed;
bool _side_slip_fusion_delayed;
bool _air_data_fusion_wind_only;
vector2f_t _pos_reset_ne;
vector2f_t _vel_reset_ne;
uint32_t _last_vel_reset_ms;
float _pos_reset_d;

location_t _ekf_origin;
location_t *_public_origin;
bool _valid_origin;
double _ekf_gps_ref_hgt;
float _ekf_origin_hgt_var;
vector3f_t _vel_offset_ned;
vector3f_t _pos_offset_ned;
nav_filter_status_t _filter_status;
vector3f_t _output_track_error;
float _tilt_error_variance;

uint8_t _mag_select_index;
bool _have_table_earth_field;
vector3f_t _table_earth_field_ga;
float _table_declination;
uint32_t _last_one_hz_ms;

bool _inhibit_del_vel_bias_states;
bool _dvel_bias_axis_inhibit[3];
vector3f_t _dvel_bias_axis_var_prev;
uint8_t _state_index_lim;

extern float _gps_vert_vel_noise;
extern float _baro_alt_noise;
extern int8_t _origin_hgt_mode;

uint32_t _prev_tas_step_ms;
uint32_t _prev_beta_drag_step_ms;
uint32_t _last_baro_received_ms;
uint16_t _hgt_retry_time_ms;
uint32_t _last_vel_pass_time_ms;
uint32_t _last_pos_pass_time_ms;
uint32_t _last_hgt_pass_time_ms;
uint32_t _last_tas_pass_time_ms;
uint32_t _last_synth_yaw_time_ms;
uint32_t _time_at_last_aux_ekf_ms;
uint32_t _last_healthy_mag_time_ms;
uint32_t _flow_valid_mea_time_ms;
uint32_t _rng_valid_mea_time_ms;
uint32_t _flow_mea_time_ms;
uint32_t _prev_flow_fuse_time_ms;
uint32_t _gnd_hgt_valid_time_ms;
uint32_t _ekf_start_time_ms;
uint32_t _last_gps_vel_fail_ms;
vector2f_t _last_known_position_ne;
bool _wind_states_aligned;
uint32_t _last_gps_aid_bad_time_ms;
uint32_t _time_tas_received_ms;
uint32_t _last_pre_align_gps_check_time_ms;
uint32_t _last_pos_reset_ms;
uint32_t _last_vel_reset_ms;
float _yaw_test_ratio;
uint32_t _last_pos_reset_d_ms;
float _pos_down_obs_noise;
quaternionf_t _prev_quat_mag_reset = {1, 0, 0, 0};
float _hgt_innov_filt_state;
bool _run_updates;
uint32_t _last_rng_meas_time_ms;

source_z_t _active_hgt_source;
source_z_t _prev_hgt_source;
float _mea_hgt_at_take_off;
bool _final_inflight_yaw_init;
uint8_t _mag_yaw_anomally_count;
bool _final_inflight_mag_init;
bool _mag_state_reset_request;
bool _mag_yaw_reset_request;
bool _gps_yaw_reset_request;
float _pos_down_at_last_mag_reset;
float _yaw_innov_at_last_mag_reset;
quaternionf_t _quat_at_last_mag_reset = {1, 0, 0, 0};
source_xy_t _posxy_source_last;
bool _posxy_source_reset;
source_yaw_t _yaw_source_last;
bool _yaw_source_reset;

yaw_elements_t _yaw_ang_data_static;
yaw_elements_t _yaw_ang_data_delayed;

bool _ekf3_core_have_inited = false;

void init_ekf3_core(void)
{
    first_init_time_ms = 0;
    last_init_fail_report_ms = 0;
    _ekf3_core_have_inited = true;
}

bool ekf3_setup_core(void)
{
    if (dal_ins_get_loop_rate_hz() > 0) {
        _dt_ekf_avg = 1.0f / dal_ins_get_loop_rate_hz();
        _dt_ekf_avg = MAX(_dt_ekf_avg, EKF_TARGET_DT);
    }
    uint16_t max_time_delay_ms = MAX(mag_delay_ms, EKF_TARGET_DT_MS);
    float gps_delay_sec = 0;
    dal_gps_get_lag(&gps_delay_sec);
    max_time_delay_ms = MAX(max_time_delay_ms, MIN(gps_delay_sec * 1000, 250));
    //_imu_buffer_length = (max_time_delay_ms / EKF_TARGET_DT_MS) + 1;
    _imu_buffer_length = (max_time_delay_ms / (_dt_ekf_avg * 1000)) + 1;
    uint16_t ekf_delay_ms = max_time_delay_ms + (int)(ceilf(max_time_delay_ms * 0.5f));
    //_obs_buffer_length = ekf_delay_ms / _sensor_interval_min_ms + 1;
    _obs_buffer_length = ekf_delay_ms / 100 + 1;
    _obs_buffer_length = MIN(_obs_buffer_length, _imu_buffer_length);
    init_ekf_ring_buffer(&_stored_gps, sizeof(gps_elements_t), _obs_buffer_length);
    if (!init_ekf_ring_buffer(&_stored_mag, sizeof(mag_elements_t), _obs_buffer_length)) {
        MY_LOG("init mag ring buffer fail\n");
        return false;
    }
    init_ekf_imu_buffer(&_stored_imu, sizeof(imu_elements_t), _imu_buffer_length);
    init_ekf_imu_buffer(&_stored_output, sizeof(output_elements_t), _imu_buffer_length);
    MY_LOG("EKF3 IMU0 buffs IMU=%u OBS=%u dt=%.4f\n", _imu_buffer_length, _obs_buffer_length,
           _dt_ekf_avg);
    return true;
}

bool ekf3_core_assume_zero_sideslip(void)
{
    return dal_get_fly_forward() && (dal_get_vehicle_class() != GROUND);
}

static void initialise_variables_mag(void)
{
    _last_healthy_mag_time_ms = _imu_sample_time_ms;
    _last_mag_update_us = 0;
    _mag_yaw_reset_timer_ms = _imu_sample_time_ms;
    _mag_timeout = false;
    _all_mag_sensors_failed = false;
    _final_inflight_mag_init = false;
    _mag_state.q0 = 1;
    m3f_identity(&_mag_state.DCM);
    _inhibit_mag_states = true;
    _mag_select_index = 0;
    _last_mag_offsets_valid = false;
    _mag_state_reset_request = false;
    _mag_state_init_complete = false;
    _mag_yaw_reset_request = false;
    _pos_down_at_last_mag_reset = _s.state_struct.position.z;
    _yaw_innov_at_last_mag_reset = 0;
    quat_initialise(&_s.state_struct.quat);
    _quat_at_last_mag_reset = _s.state_struct.quat;
    _mag_field_learned = false;
    ekf_ring_buffer_reset(&_stored_mag);
    _need_mag_body_var_reset = false;
    _need_earth_body_var_reset = false;
}

static void initialise_variables(void)
{
    _local_filter_time_step_ms = (uint8_t)(1000 * dal_ins_get_loop_delta_t());
    //MY_LOG("local_filter_time_step_ms: %d\n", _local_filter_time_step_ms); //20ms
    _local_filter_time_step_ms = MAX(_local_filter_time_step_ms, (uint8_t)EKF_TARGET_DT_MS);
    _imu_sample_time_ms = (uint32_t)_imu_sample_time_us / 1000;
    _prev_tas_step_ms = _imu_sample_time_ms;
    _prev_beta_drag_step_ms = _imu_sample_time_ms;
    _last_baro_received_ms =_imu_sample_time_ms;
    _last_vel_pass_time_ms = 0;
    _last_pos_pass_time_ms = 0;
    _last_hgt_pass_time_ms = 0;
    _last_tas_pass_time_ms = 0;
    _last_synth_yaw_time_ms = 0;
    _last_time_gps_received_ms = 0;
    _time_at_last_aux_ekf_ms = _imu_sample_time_ms;
    _flow_valid_mea_time_ms = _imu_sample_time_ms;
    _rng_valid_mea_time_ms = _imu_sample_time_ms;
    _flow_mea_time_ms = 0;
    _prev_flow_fuse_time_ms = 0;
    _gnd_hgt_valid_time_ms = 0;
    _ekf_start_time_ms = _imu_sample_time_ms;
    _last_gps_vel_fail_ms = 0;
    _last_gps_aid_bad_time_ms = 0;
    _time_tas_received_ms = 0;
    _last_pre_align_gps_check_time_ms = _imu_sample_time_ms;
    _last_pos_reset_ms = 0;
    _last_vel_reset_ms = 0;
    _last_pos_reset_d_ms = 0;
    _last_rng_meas_time_ms = 0;

    memset(&_dvel_bias_axis_inhibit, 0, sizeof(_dvel_bias_axis_inhibit));
    v3f_zero(&_dvel_bias_axis_var_prev);
    _gps_noise_scaler = 1.0f;
    _hgt_timeout = true;
    _tas_timeout = true;
    _bad_imu_data = false;
    _bad_imu_data_ms = 0;
    _good_imu_data_ms = 0;
    _vert_vel_var_clip_counter = 0;
    _final_inflight_yaw_init = false;
    _dt_imu_avg = get_loop_delta_t();
    _dt_ekf_avg = EKF_TARGET_DT;
    _dt = 0;
    v3f_zero(&_vel_dot_ned_filt);
    v2f_zero(&_last_known_position_ne);
    m3f_zero(&_prev_tnb);
    memset(&_P[0][0], 0, sizeof(_P));
    memset(&_KH[0][0], 0, sizeof(_KH));
    memset(&_KHP[0][0], 0, sizeof(_KHP));
    memset(&_nextP[0][0], 0, sizeof(_nextP));
    _flow_data_valid = false;
    _range_data_to_fuse = false;
    _P_opt = 0;
    _terrain_state = 0;
    _prev_pos_n = _s.state_struct.position.x;
    _prev_pos_e = _s.state_struct.position.y;
    _inhibit_gnd_state = false;
    _flow_gyro_bias.x = 0;
    _flow_gyro_bias.y = 0;
    _pv_aiding_mode = AID_NONE;
    _pv_aiding_mode_prev = AID_NONE;
    _pos_timeout = true;
    _vel_timeout = true;
    memset(&_fault_status, 0, sizeof(_fault_status));
    _hgt_rate = 0;
    _on_ground = true;
    _prev_on_ground = true;
    _in_flight = false;
    _prev_in_flight = false;
    _manoeuvring = false;
    _inhibit_wind_states = true;
    _wind_states_aligned = false;
    _inhibit_del_vel_bias_states = true;
    _inhibit_del_ang_bias_states = true;
    _gnd_offset_valid = false;
    _valid_origin = false;
    _gps_spd_accuracy = 0;
    _gps_pos_accuracy = 0;
    _gps_hgt_accuracy = 0;
    _baro_hgt_offset = 0;
    _rng_on_gnd = 0.05f;
    _yaw_reset_angle = 0;
    _last_yaw_reset_ms = 0;
    _tilt_error_variance = sq(M_2PI);
    _tilt_align_complete = false;
    _yaw_align_complete = false;
    _have_table_earth_field = false;
    _state_index_lim = 23;
    _last_gps_idx = 0;
    v3f_zero(&_del_ang_correction);
    v3f_zero(&_vel_err_integral);
    v3f_zero(&_pos_err_integral);
    _gps_good_to_align = false;
    _gps_in_use = false;
    _motors_armed = false;
    _prev_motors_armed = false;
    memset(&_gps_check_status, 0, sizeof(_gps_check_status));
    _gps_spd_acc_pass = false;
    _ekf_innovations_pass = false;
    _s_acc_filter_state1 = 0;
    _s_acc_filter_state2 = 0;
    _last_gps_check_time_ms = 0;
    _last_innov_pass_time_ms = 0;
    _last_innov_fail_time_ms = 0;
    _gps_accuracy_good = false;
    location_t tmp = {0, 0, 0, 0, 0, 0, 0 ,0};
    _gps_loc_prev = tmp;
    _gps_drift_ne = 0;
    _gps_vert_vel_filt = 0;
    _gps_horiz_vel_filt = 0;
    memset(&_s.states_array, 0, sizeof(_s.states_array));
    memset(&_vert_comp_filt_state, 0, sizeof(_vert_comp_filt_state));
    _pos_vel_fusion_delayed = false;
    _opt_flow_fusion_delayed = false;
    _flow_fusion_active = false;
    _air_spd_fusion_delayed = false;
    _side_slip_fusion_delayed = false;
    _air_data_fusion_wind_only = false;
    v2f_zero(&_pos_reset_ne);
    v2f_zero(&_vel_reset_ne);
    _pos_reset_d = 0;
    _hgt_innov_filt_state = 0;
    v3f_zero(&_imu_data_down_sampled_new.del_ang);
    v3f_zero(&_imu_data_down_sampled_new.del_vel);
    _imu_data_down_sampled_new.del_ang_dt = 0;
    _imu_data_down_sampled_new.del_vel_dt = 0;
    _imu_data_down_sampled_new.gyro_index = 0;
    _imu_data_down_sampled_new.accel_index = 0;
    _run_updates = false;
    _frames_since_predict = 0;
    _gps_yaw_reset_request = false;
    _del_ang_bias_learned = false;
    memset(&_filter_status, 0, sizeof(_filter_status));
    _active_hgt_source = Z_GPS;
    _prev_hgt_source = _active_hgt_source;
    memset(&_rng_meas_index, 0, sizeof(_rng_meas_index));
    memset(&_stored_rng_meas_time_ms, 0, sizeof(_stored_rng_meas_time_ms));
    memset(&_stored_rng_meas, 0, sizeof(_stored_rng_meas));
    _terrain_hgt_stable = true;
    _ekf_origin_hgt_var = 0;
    _ekf_gps_ref_hgt = 0;
    v3f_zero(&_vel_offset_ned);
    v3f_zero(&_pos_offset_ned);
    memset(&_vel_pos_obs, 0, sizeof(_vel_pos_obs));

    _last_body_vel_pass_time_ms = 0;
    memset(_body_vel_test_ratio, 0, sizeof(_body_vel_test_ratio));
    memset(_var_innov_body_vel, 0, sizeof(_var_innov_body_vel));
    memset(_innov_body_vel, 0, sizeof(_innov_body_vel));
    _prev_body_vel_fuse_time_ms = 0;
    _body_odm_meas_time_ms = 0;
    _body_vel_fusion_delayed = false;
    _body_vel_fusion_active = false;
    ekf_imu_buffer_reset(&_stored_imu);
    ekf_imu_buffer_reset(&_stored_output);
    ekf_ring_buffer_reset(&_stored_gps);

    initialise_variables_mag();

    _effective_mag_cal = ekf3_core_effective_mag_cal();
}

static void calc_tilt_error_variance(void)
{

    const float q0 = quat_idx(&_s.state_struct.quat, 0);
    const float q1 = quat_idx(&_s.state_struct.quat, 1);
    const float q2 = quat_idx(&_s.state_struct.quat, 2);
    const float q3 = quat_idx(&_s.state_struct.quat, 3);
    // equations generated by quaternion_error_propagation(): in AP_NavEKF3/derivation/main.py
    // only diagonals have been used
    // dq0 ... dq3  terms have been zeroed
    const float PS1 = q0*q1 + q2*q3;
    const float PS2 = q1*PS1;
    const float PS4 = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    const float PS5 = q0*PS4;
    const float PS6 = 2*PS2 + PS5;
    const float PS8 = PS1*q2;
    const float PS10 = PS4*q3;
    const float PS11 = PS10 + 2*PS8;
    const float PS12 = PS1*q3;
    const float PS13 = PS4*q2;
    const float PS14 = -2*PS12 + PS13;
    const float PS15 = PS1*q0;
    const float PS16 = q1*PS4;
    const float PS17 = 2*PS15 - PS16;
    const float PS18 = q0*q2 - q1*q3;
    const float PS19 = PS18*q2;
    const float PS20 = 2*PS19 + PS5;
    const float PS22 = q1*PS18;
    const float PS23 = -PS10 + 2*PS22;
    const float PS25 = PS18*q3;
    const float PS26 = PS16 + 2*PS25;
    const float PS28 = PS18*q0;
    const float PS29 = -PS13 + 2*PS28;
    const float PS32 = PS12 + PS28;
    const float PS33 = PS19 + PS2;
    const float PS34 = PS15 - PS25;
    const float PS35 = PS22 - PS8;

    _tilt_error_variance  = 4*sq(PS11)*_P[2][2] + 4*sq(PS14)*_P[3][3] + 4*sq(PS17)*_P[0][0] + 4*sq(PS6)*_P[1][1];
    _tilt_error_variance += 4*sq(PS20)*_P[2][2] + 4*sq(PS23)*_P[1][1] + 4*sq(PS26)*_P[3][3] + 4*sq(PS29)*_P[0][0];
    _tilt_error_variance += 16*sq(PS32)*_P[1][1] + 16*sq(PS33)*_P[3][3] + 16*sq(PS34)*_P[2][2] + 16*sq(PS35)*_P[0][0];

    _tilt_error_variance = constrain_float(_tilt_error_variance, 0.0f, sq(radians(30.0f)));
    //MY_LOG("tilt error var = %f\n", _tilt_error_variance);
}

void ekf3_core_covariance_prediction(vector3f_t *rot_var_vec_ptr)
{
    float dax_var;
    float day_var;
    float daz_var;
    float dvx_var;
    float dvy_var;
    float dvz_var;
    float dvx;
    float dvy;
    float dvz;
    float dax;
    float day;
    float daz;
    float q0;
    float q1;
    float q2;
    float q3;
    float dax_b;
    float day_b;
    float daz_b;
    float dvx_b;
    float dvy_b;
    float dvz_b;
    _dt = constrain_float(0.5f * (_imu_data_delayed.del_ang_dt + _imu_data_delayed.del_vel_dt),
                          0.5f * _dt_ekf_avg, 2.0f * _dt_ekf_avg);
    float alpha = 0.1f * _dt;
    _hgt_rate = _hgt_rate * (1.0f - alpha) - _s.state_struct.velocity.z * alpha;
    Vector14 process_noise_variance = {0};
    if (!_inhibit_del_ang_bias_states) {
        float d_ang_bias_var = sq(sq(_dt) * constrain_float(_gyro_bias_process_noise, 0.0, 1.0));
        for (uint8_t i = 0; i <= 2; i++) {
            process_noise_variance[i] = d_ang_bias_var;
        }
    }
    if (!_inhibit_del_vel_bias_states) {
        float d_vel_bias_var = sq(sq(_dt) * constrain_float(_accel_bias_process_noise, 0, 1.0));
        for (uint8_t i = 3; i <= 5; i++) {
            process_noise_variance[i] = d_vel_bias_var;
        }
    }
    if (!_inhibit_mag_states && _last_inhibit_mag_states) {
        _need_mag_body_var_reset = true;
        _need_earth_body_var_reset = true;
    }
    if (_need_mag_body_var_reset) {
        _need_mag_body_var_reset = false;
        ekf3_core_zero_cols(&_P, 19, 21);
        ekf3_core_zero_rows(&_P, 19, 21);
        _P[19][19] = sq(_mag_noise);
        _P[20][20] = _P[19][19];
        _P[21][21] = _P[19][19];
    }
    if (_need_earth_body_var_reset) {
        _need_earth_body_var_reset = false;
        ekf3_core_zero_cols(&_P, 16, 18);
        ekf3_core_zero_rows(&_P, 16, 18);
        _P[16][16] = sq(_mag_noise);
        _P[17][17] = _P[16][16];
        _P[18][18] = _P[16][16];
        ekf3_core_fuse_declination(radians(20.0f));
    }
    if (!_inhibit_mag_states) {
        float mag_earth_var = sq(_dt * constrain_float(_mag_earth_process_noise, 0, 1));
        float mag_body_var = sq(_dt * constrain_float(_mag_body_process_noise, 0, 1));
        for (uint8_t i = 6; i <= 8; i++) {
            process_noise_variance[i] = mag_earth_var;
        }
        for (uint8_t i = 9; i <= 11; i++) {
            process_noise_variance[i] = mag_body_var;
        }
    }
    _last_inhibit_mag_states = _inhibit_mag_states;
    if (!_inhibit_wind_states) {
        float wind_vel_var = sq(_dt * constrain_float(_wind_vel_process_noise, 0, 1) *
        (1.0f + constrain_float(_wnd_var_hgt_rate_scale, 0, 1) * fabsf(_hgt_rate)));
        for (uint8_t i = 12; i <= 13; i++) {
            process_noise_variance[i] = wind_vel_var;
        }
    }
    dvx = _imu_data_delayed.del_vel.x;
    dvy = _imu_data_delayed.del_vel.y;
    dvz = _imu_data_delayed.del_vel.z;
    dax = _imu_data_delayed.del_ang.x;
    day = _imu_data_delayed.del_ang.y;
    daz = _imu_data_delayed.del_ang.z;
    q0 = quat_idx(&_s.state_struct.quat, 0);
    q1 = quat_idx(&_s.state_struct.quat, 1);
    q2 = quat_idx(&_s.state_struct.quat, 2);
    q3 = quat_idx(&_s.state_struct.quat, 3);
    dax_b = _s.state_struct.gyro_bias.x;
    day_b = _s.state_struct.gyro_bias.y;
    daz_b = _s.state_struct.gyro_bias.z;
    dvx_b = _s.state_struct.accel_bias.x;
    dvy_b = _s.state_struct.accel_bias.y;
    dvz_b = _s.state_struct.accel_bias.z;
    bool quat_cov_reset_only = false;
    if (rot_var_vec_ptr != NULL) {
        const vector3f_t rot_var_vec = *rot_var_vec_ptr;
        matrix3f_t R_ef = {{rot_var_vec.x, 0, 0},
                           {0, rot_var_vec.y, 0},
                           {0, 0, rot_var_vec.z}};
        matrix3f_t Tnb;
        quaternionf_t tmp = quat_inverse(&_s.state_struct.quat);
        quat_to_rotation_matrix(&tmp, &Tnb);
        matrix3f_t Tnb_tr = m3f_transposed(&Tnb);
        matrix3f_t mul_tmp = m3f_multi_m(&Tnb, &R_ef);
        matrix3f_t R_bf = m3f_multi_m(&mul_tmp, &Tnb_tr);
        dax_var = R_bf.a.x;
        day_var = R_bf.b.y;
        daz_var = R_bf.c.z;
        quat_cov_reset_only = true;
        ekf3_core_zero_rows(&_P, 0, 3);
        ekf3_core_zero_cols(&_P, 0, 3);
    } else {
        float gyr_noise = constrain_float(_gyr_noise, 0, 1.0);
        dax_var = day_var = daz_var = sq(_dt * gyr_noise);
    }
    float acc_noise = _bad_imu_data ? BAD_IMU_DATA_ACC_P_NSE : constrain_float(_acc_noise, 0, BAD_IMU_DATA_ACC_P_NSE);
    dvx_var = dvy_var = dvz_var = sq(_dt * acc_noise);
    if (!_inhibit_del_vel_bias_states) {
        for (uint8_t state_index = 13; state_index <= 15; state_index++) {
            const uint8_t index = state_index - 13;
            //const bool is_bias_observable = (fabsf(((float *)&_prev_tnb)[index * 3 + 2]) > 0.8f) || !_on_ground;
            const bool is_bias_observable = false;
            if (!is_bias_observable && !_dvel_bias_axis_inhibit[index]) {
                ((float *)&_dvel_bias_axis_var_prev)[index] = _P[state_index][state_index];
                _dvel_bias_axis_inhibit[index] = true;
            } else if (is_bias_observable && _dvel_bias_axis_inhibit[index]) {
                _P[state_index][state_index] = _dvel_bias_axis_inhibit[index];
                _dvel_bias_axis_inhibit[index] = false;
            }
        }
    }
    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the lower diagonal and copy to take advantage of symmetry

    // intermediate calculations
    const float PS0 = sq(q1);
    const float PS1 = 0.25F*dax_var;
    const float PS2 = sq(q2);
    const float PS3 = 0.25F*day_var;
    const float PS4 = sq(q3);
    const float PS5 = 0.25F*daz_var;
    const float PS6 = 0.5F*q1;
    const float PS7 = 0.5F*q2;
    const float PS8 = PS7*_P[10][11];
    const float PS9 = 0.5F*q3;
    const float PS10 = PS9*_P[10][12];
    const float PS11 = 0.5F*dax - 0.5F*dax_b;
    const float PS12 = 0.5F*day - 0.5F*day_b;
    const float PS13 = 0.5F*daz - 0.5F*daz_b;
    const float PS14 = PS10 - PS11*_P[1][10] - PS12*_P[2][10] - PS13*_P[3][10] + PS6*_P[10][10] + PS8 + _P[0][10];
    const float PS15 = PS6*_P[10][11];
    const float PS16 = PS9*_P[11][12];
    const float PS17 = -PS11*_P[1][11] - PS12*_P[2][11] - PS13*_P[3][11] + PS15 + PS16 + PS7*_P[11][11] + _P[0][11];
    const float PS18 = PS6*_P[10][12];
    const float PS19 = PS7*_P[11][12];
    const float PS20 = -PS11*_P[1][12] - PS12*_P[2][12] - PS13*_P[3][12] + PS18 + PS19 + PS9*_P[12][12] + _P[0][12];
    const float PS21 = PS12*_P[1][2];
    const float PS22 = -PS13*_P[1][3];
    const float PS23 = -PS11*_P[1][1] - PS21 + PS22 + PS6*_P[1][10] + PS7*_P[1][11] + PS9*_P[1][12] + _P[0][1];
    const float PS24 = -PS11*_P[1][2];
    const float PS25 = PS13*_P[2][3];
    const float PS26 = -PS12*_P[2][2] + PS24 - PS25 + PS6*_P[2][10] + PS7*_P[2][11] + PS9*_P[2][12] + _P[0][2];
    const float PS27 = PS11*_P[1][3];
    const float PS28 = -PS12*_P[2][3];
    const float PS29 = -PS13*_P[3][3] - PS27 + PS28 + PS6*_P[3][10] + PS7*_P[3][11] + PS9*_P[3][12] + _P[0][3];
    const float PS30 = PS11*_P[0][1];
    const float PS31 = PS12*_P[0][2];
    const float PS32 = PS13*_P[0][3];
    const float PS33 = -PS30 - PS31 - PS32 + PS6*_P[0][10] + PS7*_P[0][11] + PS9*_P[0][12] + _P[0][0];
    const float PS34 = 0.5F*q0;
    const float PS35 = q2*q3;
    const float PS36 = q0*q1;
    const float PS37 = q1*q3;
    const float PS38 = q0*q2;
    const float PS39 = q1*q2;
    const float PS40 = q0*q3;
    const float PS41 = 2*PS2;
    const float PS42 = 2*PS4 - 1;
    const float PS43 = PS41 + PS42;
    const float PS44 = -PS11*_P[1][13] - PS12*_P[2][13] - PS13*_P[3][13] + PS6*_P[10][13] + PS7*_P[11][13] + PS9*_P[12][13] + _P[0][13];
    const float PS45 = PS37 + PS38;
    const float PS46 = -PS11*_P[1][15] - PS12*_P[2][15] - PS13*_P[3][15] + PS6*_P[10][15] + PS7*_P[11][15] + PS9*_P[12][15] + _P[0][15];
    const float PS47 = 2*PS46;
    const float PS48 = dvy - dvy_b;
    const float PS49 = PS48*q0;
    const float PS50 = dvz - dvz_b;
    const float PS51 = PS50*q1;
    const float PS52 = dvx - dvx_b;
    const float PS53 = PS52*q3;
    const float PS54 = PS49 - PS51 + 2*PS53;
    const float PS55 = 2*PS29;
    const float PS56 = -PS39 + PS40;
    const float PS57 = -PS11*_P[1][14] - PS12*_P[2][14] - PS13*_P[3][14] + PS6*_P[10][14] + PS7*_P[11][14] + PS9*_P[12][14] + _P[0][14];
    const float PS58 = 2*PS57;
    const float PS59 = PS48*q2;
    const float PS60 = PS50*q3;
    const float PS61 = PS59 + PS60;
    const float PS62 = 2*PS23;
    const float PS63 = PS50*q2;
    const float PS64 = PS48*q3;
    const float PS65 = -PS64;
    const float PS66 = PS63 + PS65;
    const float PS67 = 2*PS33;
    const float PS68 = PS50*q0;
    const float PS69 = PS48*q1;
    const float PS70 = PS52*q2;
    const float PS71 = PS68 + PS69 - 2*PS70;
    const float PS72 = 2*PS26;
    const float PS73 = -PS11*_P[1][4] - PS12*_P[2][4] - PS13*_P[3][4] + PS6*_P[4][10] + PS7*_P[4][11] + PS9*_P[4][12] + _P[0][4];
    const float PS74 = 2*PS0;
    const float PS75 = PS42 + PS74;
    const float PS76 = PS39 + PS40;
    const float PS77 = 2*PS44;
    const float PS78 = PS51 - PS53;
    const float PS79 = -PS70;
    const float PS80 = PS68 + 2*PS69 + PS79;
    const float PS81 = -PS35 + PS36;
    const float PS82 = PS52*q1;
    const float PS83 = PS60 + PS82;
    const float PS84 = PS52*q0;
    const float PS85 = PS63 - 2*PS64 + PS84;
    const float PS86 = -PS11*_P[1][5] - PS12*_P[2][5] - PS13*_P[3][5] + PS6*_P[5][10] + PS7*_P[5][11] + PS9*_P[5][12] + _P[0][5];
    const float PS87 = PS41 + PS74 - 1;
    const float PS88 = PS35 + PS36;
    const float PS89 = 2*PS63 + PS65 + PS84;
    const float PS90 = -PS37 + PS38;
    const float PS91 = PS59 + PS82;
    const float PS92 = PS69 + PS79;
    const float PS93 = PS49 - 2*PS51 + PS53;
    const float PS94 = -PS11*_P[1][6] - PS12*_P[2][6] - PS13*_P[3][6] + PS6*_P[6][10] + PS7*_P[6][11] + PS9*_P[6][12] + _P[0][6];
    const float PS95 = sq(q0);
    const float PS96 = -PS34*_P[10][11];
    const float PS97 = PS11*_P[0][11] - PS12*_P[3][11] + PS13*_P[2][11] - PS19 + PS9*_P[11][11] + PS96 + _P[1][11];
    const float PS98 = PS13*_P[0][2];
    const float PS99 = PS12*_P[0][3];
    const float PS100 = PS11*_P[0][0] - PS34*_P[0][10] - PS7*_P[0][12] + PS9*_P[0][11] + PS98 - PS99 + _P[0][1];
    const float PS101 = PS11*_P[0][2];
    const float PS102 = PS101 + PS13*_P[2][2] + PS28 - PS34*_P[2][10] - PS7*_P[2][12] + PS9*_P[2][11] + _P[1][2];
    const float PS103 = PS9*_P[10][11];
    const float PS104 = PS7*_P[10][12];
    const float PS105 = PS103 - PS104 + PS11*_P[0][10] - PS12*_P[3][10] + PS13*_P[2][10] - PS34*_P[10][10] + _P[1][10];
    const float PS106 = -PS34*_P[10][12];
    const float PS107 = PS106 + PS11*_P[0][12] - PS12*_P[3][12] + PS13*_P[2][12] + PS16 - PS7*_P[12][12] + _P[1][12];
    const float PS108 = PS11*_P[0][3];
    const float PS109 = PS108 - PS12*_P[3][3] + PS25 - PS34*_P[3][10] - PS7*_P[3][12] + PS9*_P[3][11] + _P[1][3];
    const float PS110 = PS13*_P[1][2];
    const float PS111 = PS12*_P[1][3];
    const float PS112 = PS110 - PS111 + PS30 - PS34*_P[1][10] - PS7*_P[1][12] + PS9*_P[1][11] + _P[1][1];
    const float PS113 = PS11*_P[0][13] - PS12*_P[3][13] + PS13*_P[2][13] - PS34*_P[10][13] - PS7*_P[12][13] + PS9*_P[11][13] + _P[1][13];
    const float PS114 = PS11*_P[0][15] - PS12*_P[3][15] + PS13*_P[2][15] - PS34*_P[10][15] - PS7*_P[12][15] + PS9*_P[11][15] + _P[1][15];
    const float PS115 = 2*PS114;
    const float PS116 = 2*PS109;
    const float PS117 = PS11*_P[0][14] - PS12*_P[3][14] + PS13*_P[2][14] - PS34*_P[10][14] - PS7*_P[12][14] + PS9*_P[11][14] + _P[1][14];
    const float PS118 = 2*PS117;
    const float PS119 = 2*PS112;
    const float PS120 = 2*PS100;
    const float PS121 = 2*PS102;
    const float PS122 = PS11*_P[0][4] - PS12*_P[3][4] + PS13*_P[2][4] - PS34*_P[4][10] - PS7*_P[4][12] + PS9*_P[4][11] + _P[1][4];
    const float PS123 = 2*PS113;
    const float PS124 = PS11*_P[0][5] - PS12*_P[3][5] + PS13*_P[2][5] - PS34*_P[5][10] - PS7*_P[5][12] + PS9*_P[5][11] + _P[1][5];
    const float PS125 = PS11*_P[0][6] - PS12*_P[3][6] + PS13*_P[2][6] - PS34*_P[6][10] - PS7*_P[6][12] + PS9*_P[6][11] + _P[1][6];
    const float PS126 = -PS34*_P[11][12];
    const float PS127 = -PS10 + PS11*_P[3][12] + PS12*_P[0][12] + PS126 - PS13*_P[1][12] + PS6*_P[12][12] + _P[2][12];
    const float PS128 = PS11*_P[3][3] + PS22 - PS34*_P[3][11] + PS6*_P[3][12] - PS9*_P[3][10] + PS99 + _P[2][3];
    const float PS129 = PS13*_P[0][1];
    const float PS130 = PS108 + PS12*_P[0][0] - PS129 - PS34*_P[0][11] + PS6*_P[0][12] - PS9*_P[0][10] + _P[0][2];
    const float PS131 = PS6*_P[11][12];
    const float PS132 = -PS103 + PS11*_P[3][11] + PS12*_P[0][11] - PS13*_P[1][11] + PS131 - PS34*_P[11][11] + _P[2][11];
    const float PS133 = PS11*_P[3][10] + PS12*_P[0][10] - PS13*_P[1][10] + PS18 - PS9*_P[10][10] + PS96 + _P[2][10];
    const float PS134 = PS12*_P[0][1];
    const float PS135 = -PS13*_P[1][1] + PS134 + PS27 - PS34*_P[1][11] + PS6*_P[1][12] - PS9*_P[1][10] + _P[1][2];
    const float PS136 = PS11*_P[2][3];
    const float PS137 = -PS110 + PS136 + PS31 - PS34*_P[2][11] + PS6*_P[2][12] - PS9*_P[2][10] + _P[2][2];
    const float PS138 = PS11*_P[3][13] + PS12*_P[0][13] - PS13*_P[1][13] - PS34*_P[11][13] + PS6*_P[12][13] - PS9*_P[10][13] + _P[2][13];
    const float PS139 = PS11*_P[3][15] + PS12*_P[0][15] - PS13*_P[1][15] - PS34*_P[11][15] + PS6*_P[12][15] - PS9*_P[10][15] + _P[2][15];
    const float PS140 = 2*PS139;
    const float PS141 = 2*PS128;
    const float PS142 = PS11*_P[3][14] + PS12*_P[0][14] - PS13*_P[1][14] - PS34*_P[11][14] + PS6*_P[12][14] - PS9*_P[10][14] + _P[2][14];
    const float PS143 = 2*PS142;
    const float PS144 = 2*PS135;
    const float PS145 = 2*PS130;
    const float PS146 = 2*PS137;
    const float PS147 = PS11*_P[3][4] + PS12*_P[0][4] - PS13*_P[1][4] - PS34*_P[4][11] + PS6*_P[4][12] - PS9*_P[4][10] + _P[2][4];
    const float PS148 = 2*PS138;
    const float PS149 = PS11*_P[3][5] + PS12*_P[0][5] - PS13*_P[1][5] - PS34*_P[5][11] + PS6*_P[5][12] - PS9*_P[5][10] + _P[2][5];
    const float PS150 = PS11*_P[3][6] + PS12*_P[0][6] - PS13*_P[1][6] - PS34*_P[6][11] + PS6*_P[6][12] - PS9*_P[6][10] + _P[2][6];
    const float PS151 = PS106 - PS11*_P[2][10] + PS12*_P[1][10] + PS13*_P[0][10] - PS15 + PS7*_P[10][10] + _P[3][10];
    const float PS152 = PS12*_P[1][1] + PS129 + PS24 - PS34*_P[1][12] - PS6*_P[1][11] + PS7*_P[1][10] + _P[1][3];
    const float PS153 = -PS101 + PS13*_P[0][0] + PS134 - PS34*_P[0][12] - PS6*_P[0][11] + PS7*_P[0][10] + _P[0][3];
    const float PS154 = PS104 - PS11*_P[2][12] + PS12*_P[1][12] + PS13*_P[0][12] - PS131 - PS34*_P[12][12] + _P[3][12];
    const float PS155 = -PS11*_P[2][11] + PS12*_P[1][11] + PS126 + PS13*_P[0][11] - PS6*_P[11][11] + PS8 + _P[3][11];
    const float PS156 = -PS11*_P[2][2] + PS21 - PS34*_P[2][12] - PS6*_P[2][11] + PS7*_P[2][10] + PS98 + _P[2][3];
    const float PS157 = PS111 - PS136 + PS32 - PS34*_P[3][12] - PS6*_P[3][11] + PS7*_P[3][10] + _P[3][3];
    const float PS158 = -PS11*_P[2][13] + PS12*_P[1][13] + PS13*_P[0][13] - PS34*_P[12][13] - PS6*_P[11][13] + PS7*_P[10][13] + _P[3][13];
    const float PS159 = -PS11*_P[2][15] + PS12*_P[1][15] + PS13*_P[0][15] - PS34*_P[12][15] - PS6*_P[11][15] + PS7*_P[10][15] + _P[3][15];
    const float PS160 = 2*PS159;
    const float PS161 = 2*PS157;
    const float PS162 = -PS11*_P[2][14] + PS12*_P[1][14] + PS13*_P[0][14] - PS34*_P[12][14] - PS6*_P[11][14] + PS7*_P[10][14] + _P[3][14];
    const float PS163 = 2*PS162;
    const float PS164 = 2*PS152;
    const float PS165 = 2*PS153;
    const float PS166 = 2*PS156;
    const float PS167 = -PS11*_P[2][4] + PS12*_P[1][4] + PS13*_P[0][4] - PS34*_P[4][12] - PS6*_P[4][11] + PS7*_P[4][10] + _P[3][4];
    const float PS168 = 2*PS158;
    const float PS169 = -PS11*_P[2][5] + PS12*_P[1][5] + PS13*_P[0][5] - PS34*_P[5][12] - PS6*_P[5][11] + PS7*_P[5][10] + _P[3][5];
    const float PS170 = -PS11*_P[2][6] + PS12*_P[1][6] + PS13*_P[0][6] - PS34*_P[6][12] - PS6*_P[6][11] + PS7*_P[6][10] + _P[3][6];
    const float PS171 = 2*PS45;
    const float PS172 = 2*PS56;
    const float PS173 = 2*PS61;
    const float PS174 = 2*PS66;
    const float PS175 = 2*PS71;
    const float PS176 = 2*PS54;
    const float PS177 = -PS171*_P[13][15] + PS172*_P[13][14] + PS173*_P[1][13] + PS174*_P[0][13] + PS175*_P[2][13] - PS176*_P[3][13] + PS43*_P[13][13] + _P[4][13];
    const float PS178 = -PS171*_P[15][15] + PS172*_P[14][15] + PS173*_P[1][15] + PS174*_P[0][15] + PS175*_P[2][15] - PS176*_P[3][15] + PS43*_P[13][15] + _P[4][15];
    const float PS179 = -PS171*_P[3][15] + PS172*_P[3][14] + PS173*_P[1][3] + PS174*_P[0][3] + PS175*_P[2][3] - PS176*_P[3][3] + PS43*_P[3][13] + _P[3][4];
    const float PS180 = -PS171*_P[14][15] + PS172*_P[14][14] + PS173*_P[1][14] + PS174*_P[0][14] + PS175*_P[2][14] - PS176*_P[3][14] + PS43*_P[13][14] + _P[4][14];
    const float PS181 = -PS171*_P[1][15] + PS172*_P[1][14] + PS173*_P[1][1] + PS174*_P[0][1] + PS175*_P[1][2] - PS176*_P[1][3] + PS43*_P[1][13] + _P[1][4];
    const float PS182 = -PS171*_P[0][15] + PS172*_P[0][14] + PS173*_P[0][1] + PS174*_P[0][0] + PS175*_P[0][2] - PS176*_P[0][3] + PS43*_P[0][13] + _P[0][4];
    const float PS183 = -PS171*_P[2][15] + PS172*_P[2][14] + PS173*_P[1][2] + PS174*_P[0][2] + PS175*_P[2][2] - PS176*_P[2][3] + PS43*_P[2][13] + _P[2][4];
    const float PS184 = 4*dvy_var;
    const float PS185 = 4*dvz_var;
    const float PS186 = -PS171*_P[4][15] + PS172*_P[4][14] + PS173*_P[1][4] + PS174*_P[0][4] + PS175*_P[2][4] - PS176*_P[3][4] + PS43*_P[4][13] + _P[4][4];
    const float PS187 = 2*PS177;
    const float PS188 = 2*PS182;
    const float PS189 = 2*PS181;
    const float PS190 = 2*PS81;
    const float PS191 = 2*PS183;
    const float PS192 = 2*PS179;
    const float PS193 = 2*PS76;
    const float PS194 = PS43*dvx_var;
    const float PS195 = PS75*dvy_var;
    const float PS196 = -PS171*_P[5][15] + PS172*_P[5][14] + PS173*_P[1][5] + PS174*_P[0][5] + PS175*_P[2][5] - PS176*_P[3][5] + PS43*_P[5][13] + _P[4][5];
    const float PS197 = 2*PS88;
    const float PS198 = PS87*dvz_var;
    const float PS199 = 2*PS90;
    const float PS200 = -PS171*_P[6][15] + PS172*_P[6][14] + PS173*_P[1][6] + PS174*_P[0][6] + PS175*_P[2][6] - PS176*_P[3][6] + PS43*_P[6][13] + _P[4][6];
    const float PS201 = 2*PS83;
    const float PS202 = 2*PS78;
    const float PS203 = 2*PS85;
    const float PS204 = 2*PS80;
    const float PS205 = PS190*_P[14][15] - PS193*_P[13][14] + PS201*_P[2][14] - PS202*_P[0][14] + PS203*_P[3][14] - PS204*_P[1][14] + PS75*_P[14][14] + _P[5][14];
    const float PS206 = PS190*_P[13][15] - PS193*_P[13][13] + PS201*_P[2][13] - PS202*_P[0][13] + PS203*_P[3][13] - PS204*_P[1][13] + PS75*_P[13][14] + _P[5][13];
    const float PS207 = PS190*_P[0][15] - PS193*_P[0][13] + PS201*_P[0][2] - PS202*_P[0][0] + PS203*_P[0][3] - PS204*_P[0][1] + PS75*_P[0][14] + _P[0][5];
    const float PS208 = PS190*_P[1][15] - PS193*_P[1][13] + PS201*_P[1][2] - PS202*_P[0][1] + PS203*_P[1][3] - PS204*_P[1][1] + PS75*_P[1][14] + _P[1][5];
    const float PS209 = PS190*_P[15][15] - PS193*_P[13][15] + PS201*_P[2][15] - PS202*_P[0][15] + PS203*_P[3][15] - PS204*_P[1][15] + PS75*_P[14][15] + _P[5][15];
    const float PS210 = PS190*_P[2][15] - PS193*_P[2][13] + PS201*_P[2][2] - PS202*_P[0][2] + PS203*_P[2][3] - PS204*_P[1][2] + PS75*_P[2][14] + _P[2][5];
    const float PS211 = PS190*_P[3][15] - PS193*_P[3][13] + PS201*_P[2][3] - PS202*_P[0][3] + PS203*_P[3][3] - PS204*_P[1][3] + PS75*_P[3][14] + _P[3][5];
    const float PS212 = 4*dvx_var;
    const float PS213 = PS190*_P[5][15] - PS193*_P[5][13] + PS201*_P[2][5] - PS202*_P[0][5] + PS203*_P[3][5] - PS204*_P[1][5] + PS75*_P[5][14] + _P[5][5];
    const float PS214 = 2*PS89;
    const float PS215 = 2*PS91;
    const float PS216 = 2*PS92;
    const float PS217 = 2*PS93;
    const float PS218 = PS190*_P[6][15] - PS193*_P[6][13] + PS201*_P[2][6] - PS202*_P[0][6] + PS203*_P[3][6] - PS204*_P[1][6] + PS75*_P[6][14] + _P[5][6];
    const float PS219 = -PS197*_P[14][15] + PS199*_P[13][15] - PS214*_P[2][15] + PS215*_P[3][15] + PS216*_P[0][15] + PS217*_P[1][15] + PS87*_P[15][15] + _P[6][15];
    const float PS220 = -PS197*_P[14][14] + PS199*_P[13][14] - PS214*_P[2][14] + PS215*_P[3][14] + PS216*_P[0][14] + PS217*_P[1][14] + PS87*_P[14][15] + _P[6][14];
    const float PS221 = -PS197*_P[13][14] + PS199*_P[13][13] - PS214*_P[2][13] + PS215*_P[3][13] + PS216*_P[0][13] + PS217*_P[1][13] + PS87*_P[13][15] + _P[6][13];
    const float PS222 = -PS197*_P[6][14] + PS199*_P[6][13] - PS214*_P[2][6] + PS215*_P[3][6] + PS216*_P[0][6] + PS217*_P[1][6] + PS87*_P[6][15] + _P[6][6];

    _nextP[0][0] = PS0*PS1 - PS11*PS23 - PS12*PS26 - PS13*PS29 + PS14*PS6 + PS17*PS7 + PS2*PS3 + PS20*PS9 + PS33 + PS4*PS5;
    _nextP[0][1] = -PS1*PS36 + PS11*PS33 - PS12*PS29 + PS13*PS26 - PS14*PS34 + PS17*PS9 - PS20*PS7 + PS23 + PS3*PS35 - PS35*PS5;
    _nextP[1][1] = PS1*PS95 + PS100*PS11 + PS102*PS13 - PS105*PS34 - PS107*PS7 - PS109*PS12 + PS112 + PS2*PS5 + PS3*PS4 + PS9*PS97;
    _nextP[0][2] = -PS1*PS37 + PS11*PS29 + PS12*PS33 - PS13*PS23 - PS14*PS9 - PS17*PS34 + PS20*PS6 + PS26 - PS3*PS38 + PS37*PS5;
    _nextP[1][2] = PS1*PS40 + PS100*PS12 + PS102 - PS105*PS9 + PS107*PS6 + PS109*PS11 - PS112*PS13 - PS3*PS40 - PS34*PS97 - PS39*PS5;
    _nextP[2][2] = PS0*PS5 + PS1*PS4 + PS11*PS128 + PS12*PS130 + PS127*PS6 - PS13*PS135 - PS132*PS34 - PS133*PS9 + PS137 + PS3*PS95;
    _nextP[0][3] = PS1*PS39 - PS11*PS26 + PS12*PS23 + PS13*PS33 + PS14*PS7 - PS17*PS6 - PS20*PS34 + PS29 - PS3*PS39 - PS40*PS5;
    _nextP[1][3] = -PS1*PS38 + PS100*PS13 - PS102*PS11 + PS105*PS7 - PS107*PS34 + PS109 + PS112*PS12 - PS3*PS37 + PS38*PS5 - PS6*PS97;
    _nextP[2][3] = -PS1*PS35 - PS11*PS137 + PS12*PS135 - PS127*PS34 + PS128 + PS13*PS130 - PS132*PS6 + PS133*PS7 + PS3*PS36 - PS36*PS5;
    _nextP[3][3] = PS0*PS3 + PS1*PS2 - PS11*PS156 + PS12*PS152 + PS13*PS153 + PS151*PS7 - PS154*PS34 - PS155*PS6 + PS157 + PS5*PS95;

    if (quat_cov_reset_only) {
        // covariance matrix is symmetrical, so copy diagonals and copy lower half in _nextP
        // to lower and upper half in P
        for (uint8_t row = 0; row <= 3; row++) {
            // copy diagonals
            _P[row][row] = constrain_float(_nextP[row][row], 0.0f, 1.0f);
            // copy off diagonals
            for (uint8_t column = 0 ; column < row; column++) {
                _P[row][column] = _P[column][row] = _nextP[column][row];
            }
        }
        calc_tilt_error_variance();
        return;
    }

    _nextP[0][4] = PS43*PS44 - PS45*PS47 - PS54*PS55 + PS56*PS58 + PS61*PS62 + PS66*PS67 + PS71*PS72 + PS73;
    _nextP[1][4] = PS113*PS43 - PS115*PS45 - PS116*PS54 + PS118*PS56 + PS119*PS61 + PS120*PS66 + PS121*PS71 + PS122;
    _nextP[2][4] = PS138*PS43 - PS140*PS45 - PS141*PS54 + PS143*PS56 + PS144*PS61 + PS145*PS66 + PS146*PS71 + PS147;
    _nextP[3][4] = PS158*PS43 - PS160*PS45 - PS161*PS54 + PS163*PS56 + PS164*PS61 + PS165*PS66 + PS166*PS71 + PS167;
    _nextP[4][4] = -PS171*PS178 + PS172*PS180 + PS173*PS181 + PS174*PS182 + PS175*PS183 - PS176*PS179 + PS177*PS43 + PS184*sq(PS56) + PS185*sq(PS45) + PS186 + sq(PS43)*dvx_var;
    _nextP[0][5] = PS47*PS81 + PS55*PS85 + PS57*PS75 - PS62*PS80 - PS67*PS78 + PS72*PS83 - PS76*PS77 + PS86;
    _nextP[1][5] = PS115*PS81 + PS116*PS85 + PS117*PS75 - PS119*PS80 - PS120*PS78 + PS121*PS83 - PS123*PS76 + PS124;
    _nextP[2][5] = PS140*PS81 + PS141*PS85 + PS142*PS75 - PS144*PS80 - PS145*PS78 + PS146*PS83 - PS148*PS76 + PS149;
    _nextP[3][5] = PS160*PS81 + PS161*PS85 + PS162*PS75 - PS164*PS80 - PS165*PS78 + PS166*PS83 - PS168*PS76 + PS169;
    _nextP[4][5] = PS172*PS195 + PS178*PS190 + PS180*PS75 - PS185*PS45*PS81 - PS187*PS76 - PS188*PS78 - PS189*PS80 + PS191*PS83 + PS192*PS85 - PS193*PS194 + PS196;
    _nextP[5][5] = PS185*sq(PS81) + PS190*PS209 - PS193*PS206 + PS201*PS210 - PS202*PS207 + PS203*PS211 - PS204*PS208 + PS205*PS75 + PS212*sq(PS76) + PS213 + sq(PS75)*dvy_var;
    _nextP[0][6] = PS46*PS87 + PS55*PS91 - PS58*PS88 + PS62*PS93 + PS67*PS92 - PS72*PS89 + PS77*PS90 + PS94;
    _nextP[1][6] = PS114*PS87 + PS116*PS91 - PS118*PS88 + PS119*PS93 + PS120*PS92 - PS121*PS89 + PS123*PS90 + PS125;
    _nextP[2][6] = PS139*PS87 + PS141*PS91 - PS143*PS88 + PS144*PS93 + PS145*PS92 - PS146*PS89 + PS148*PS90 + PS150;
    _nextP[3][6] = PS159*PS87 + PS161*PS91 - PS163*PS88 + PS164*PS93 + PS165*PS92 - PS166*PS89 + PS168*PS90 + PS170;
    _nextP[4][6] = -PS171*PS198 + PS178*PS87 - PS180*PS197 - PS184*PS56*PS88 + PS187*PS90 + PS188*PS92 + PS189*PS93 - PS191*PS89 + PS192*PS91 + PS194*PS199 + PS200;
    _nextP[5][6] = PS190*PS198 - PS195*PS197 - PS197*PS205 + PS199*PS206 + PS207*PS216 + PS208*PS217 + PS209*PS87 - PS210*PS214 + PS211*PS215 - PS212*PS76*PS90 + PS218;
    _nextP[6][6] = PS184*sq(PS88) - PS197*PS220 + PS199*PS221 + PS212*sq(PS90) - PS214*(-PS197*_P[2][14] + PS199*_P[2][13] - PS214*_P[2][2] + PS215*_P[2][3] + PS216*_P[0][2] + PS217*_P[1][2] + PS87*_P[2][15] + _P[2][6]) + PS215*(-PS197*_P[3][14] + PS199*_P[3][13] - PS214*_P[2][3] + PS215*_P[3][3] + PS216*_P[0][3] + PS217*_P[1][3] + PS87*_P[3][15] + _P[3][6]) + PS216*(-PS197*_P[0][14] + PS199*_P[0][13] - PS214*_P[0][2] + PS215*_P[0][3] + PS216*_P[0][0] + PS217*_P[0][1] + PS87*_P[0][15] + _P[0][6]) + PS217*(-PS197*_P[1][14] + PS199*_P[1][13] - PS214*_P[1][2] + PS215*_P[1][3] + PS216*_P[0][1] + PS217*_P[1][1] + PS87*_P[1][15] + _P[1][6]) + PS219*PS87 + PS222 + sq(PS87)*dvz_var;
    _nextP[0][7] = -PS11*_P[1][7] - PS12*_P[2][7] - PS13*_P[3][7] + PS6*_P[7][10] + PS7*_P[7][11] + PS73*_dt + PS9*_P[7][12] + _P[0][7];
    _nextP[1][7] = PS11*_P[0][7] - PS12*_P[3][7] + PS122*_dt + PS13*_P[2][7] - PS34*_P[7][10] - PS7*_P[7][12] + PS9*_P[7][11] + _P[1][7];
    _nextP[2][7] = PS11*_P[3][7] + PS12*_P[0][7] - PS13*_P[1][7] + PS147*_dt - PS34*_P[7][11] + PS6*_P[7][12] - PS9*_P[7][10] + _P[2][7];
    _nextP[3][7] = -PS11*_P[2][7] + PS12*_P[1][7] + PS13*_P[0][7] + PS167*_dt - PS34*_P[7][12] - PS6*_P[7][11] + PS7*_P[7][10] + _P[3][7];
    _nextP[4][7] = -PS171*_P[7][15] + PS172*_P[7][14] + PS173*_P[1][7] + PS174*_P[0][7] + PS175*_P[2][7] - PS176*_P[3][7] + PS186*_dt + PS43*_P[7][13] + _P[4][7];
    _nextP[5][7] = PS190*_P[7][15] - PS193*_P[7][13] + PS201*_P[2][7] - PS202*_P[0][7] + PS203*_P[3][7] - PS204*_P[1][7] + PS75*_P[7][14] + _P[5][7] + _dt*(PS190*_P[4][15] - PS193*_P[4][13] + PS201*_P[2][4] - PS202*_P[0][4] + PS203*_P[3][4] - PS204*_P[1][4] + PS75*_P[4][14] + _P[4][5]);
    _nextP[6][7] = -PS197*_P[7][14] + PS199*_P[7][13] - PS214*_P[2][7] + PS215*_P[3][7] + PS216*_P[0][7] + PS217*_P[1][7] + PS87*_P[7][15] + _P[6][7] +_dt*(-PS197*_P[4][14] + PS199*_P[4][13] - PS214*_P[2][4] + PS215*_P[3][4] + PS216*_P[0][4] + PS217*_P[1][4] + PS87*_P[4][15] + _P[4][6]);
    _nextP[7][7] = _P[4][7]*_dt + _P[7][7] +_dt*(_P[4][4]*_dt + _P[4][7]);
    _nextP[0][8] = -PS11*_P[1][8] - PS12*_P[2][8] - PS13*_P[3][8] + PS6*_P[8][10] + PS7*_P[8][11] + PS86*_dt + PS9*_P[8][12] + _P[0][8];
    _nextP[1][8] = PS11*_P[0][8] - PS12*_P[3][8] + PS124*_dt + PS13*_P[2][8] - PS34*_P[8][10] - PS7*_P[8][12] + PS9*_P[8][11] + _P[1][8];
    _nextP[2][8] = PS11*_P[3][8] + PS12*_P[0][8] - PS13*_P[1][8] + PS149*_dt - PS34*_P[8][11] + PS6*_P[8][12] - PS9*_P[8][10] + _P[2][8];
    _nextP[3][8] = -PS11*_P[2][8] + PS12*_P[1][8] + PS13*_P[0][8] + PS169*_dt - PS34*_P[8][12] - PS6*_P[8][11] + PS7*_P[8][10] + _P[3][8];
    _nextP[4][8] = -PS171*_P[8][15] + PS172*_P[8][14] + PS173*_P[1][8] + PS174*_P[0][8] + PS175*_P[2][8] - PS176*_P[3][8] + PS196*_dt + PS43*_P[8][13] + _P[4][8];
    _nextP[5][8] = PS190*_P[8][15] - PS193*_P[8][13] + PS201*_P[2][8] - PS202*_P[0][8] + PS203*_P[3][8] - PS204*_P[1][8] + PS213*_dt + PS75*_P[8][14] + _P[5][8];
    _nextP[6][8] = -PS197*_P[8][14] + PS199*_P[8][13] - PS214*_P[2][8] + PS215*_P[3][8] + PS216*_P[0][8] + PS217*_P[1][8] + PS87*_P[8][15] + _P[6][8] +_dt*(-PS197*_P[5][14] + PS199*_P[5][13] - PS214*_P[2][5] + PS215*_P[3][5] + PS216*_P[0][5] + PS217*_P[1][5] + PS87*_P[5][15] + _P[5][6]);
    _nextP[7][8] = _P[4][8]*_dt + _P[7][8] +_dt*(_P[4][5]*_dt + _P[5][7]);
    _nextP[8][8] = _P[5][8]*_dt + _P[8][8] +_dt*(_P[5][5]*_dt + _P[5][8]);
    _nextP[0][9] = -PS11*_P[1][9] - PS12*_P[2][9] - PS13*_P[3][9] + PS6*_P[9][10] + PS7*_P[9][11] + PS9*_P[9][12] + PS94*_dt + _P[0][9];
    _nextP[1][9] = PS11*_P[0][9] - PS12*_P[3][9] + PS125*_dt + PS13*_P[2][9] - PS34*_P[9][10] - PS7*_P[9][12] + PS9*_P[9][11] + _P[1][9];
    _nextP[2][9] = PS11*_P[3][9] + PS12*_P[0][9] - PS13*_P[1][9] + PS150*_dt - PS34*_P[9][11] + PS6*_P[9][12] - PS9*_P[9][10] + _P[2][9];
    _nextP[3][9] = -PS11*_P[2][9] + PS12*_P[1][9] + PS13*_P[0][9] + PS170*_dt - PS34*_P[9][12] - PS6*_P[9][11] + PS7*_P[9][10] + _P[3][9];
    _nextP[4][9] = -PS171*_P[9][15] + PS172*_P[9][14] + PS173*_P[1][9] + PS174*_P[0][9] + PS175*_P[2][9] - PS176*_P[3][9] + PS200*_dt + PS43*_P[9][13] + _P[4][9];
    _nextP[5][9] = PS190*_P[9][15] - PS193*_P[9][13] + PS201*_P[2][9] - PS202*_P[0][9] + PS203*_P[3][9] - PS204*_P[1][9] + PS218*_dt + PS75*_P[9][14] + _P[5][9];
    _nextP[6][9] = -PS197*_P[9][14] + PS199*_P[9][13] - PS214*_P[2][9] + PS215*_P[3][9] + PS216*_P[0][9] + PS217*_P[1][9] + PS222*_dt + PS87*_P[9][15] + _P[6][9];
    _nextP[7][9] = _P[4][9]*_dt + _P[7][9] +_dt*(_P[4][6]*_dt + _P[6][7]);
    _nextP[8][9] = _P[5][9]*_dt + _P[8][9] +_dt*(_P[5][6]*_dt + _P[6][8]);
    _nextP[9][9] = _P[6][9]*_dt + _P[9][9] +_dt*(_P[6][6]*_dt + _P[6][9]);

    if (_state_index_lim > 9) {
        _nextP[0][10] = PS14;
        _nextP[1][10] = PS105;
        _nextP[2][10] = PS133;
        _nextP[3][10] = PS151;
        _nextP[4][10] = -PS171*_P[10][15] + PS172*_P[10][14] + PS173*_P[1][10] + PS174*_P[0][10] + PS175*_P[2][10] - PS176*_P[3][10] + PS43*_P[10][13] + _P[4][10];
        _nextP[5][10] = PS190*_P[10][15] - PS193*_P[10][13] + PS201*_P[2][10] - PS202*_P[0][10] + PS203*_P[3][10] - PS204*_P[1][10] + PS75*_P[10][14] + _P[5][10];
        _nextP[6][10] = -PS197*_P[10][14] + PS199*_P[10][13] - PS214*_P[2][10] + PS215*_P[3][10] + PS216*_P[0][10] + PS217*_P[1][10] + PS87*_P[10][15] + _P[6][10];
        _nextP[7][10] = _P[4][10]*_dt + _P[7][10];
        _nextP[8][10] = _P[5][10]*_dt + _P[8][10];
        _nextP[9][10] = _P[6][10]*_dt + _P[9][10];
        _nextP[10][10] = _P[10][10];
        _nextP[0][11] = PS17;
        _nextP[1][11] = PS97;
        _nextP[2][11] = PS132;
        _nextP[3][11] = PS155;
        _nextP[4][11] = -PS171*_P[11][15] + PS172*_P[11][14] + PS173*_P[1][11] + PS174*_P[0][11] + PS175*_P[2][11] - PS176*_P[3][11] + PS43*_P[11][13] + _P[4][11];
        _nextP[5][11] = PS190*_P[11][15] - PS193*_P[11][13] + PS201*_P[2][11] - PS202*_P[0][11] + PS203*_P[3][11] - PS204*_P[1][11] + PS75*_P[11][14] + _P[5][11];
        _nextP[6][11] = -PS197*_P[11][14] + PS199*_P[11][13] - PS214*_P[2][11] + PS215*_P[3][11] + PS216*_P[0][11] + PS217*_P[1][11] + PS87*_P[11][15] + _P[6][11];
        _nextP[7][11] = _P[4][11]*_dt + _P[7][11];
        _nextP[8][11] = _P[5][11]*_dt + _P[8][11];
        _nextP[9][11] = _P[6][11]*_dt + _P[9][11];
        _nextP[10][11] = _P[10][11];
        _nextP[11][11] = _P[11][11];
        _nextP[0][12] = PS20;
        _nextP[1][12] = PS107;
        _nextP[2][12] = PS127;
        _nextP[3][12] = PS154;
        _nextP[4][12] = -PS171*_P[12][15] + PS172*_P[12][14] + PS173*_P[1][12] + PS174*_P[0][12] + PS175*_P[2][12] - PS176*_P[3][12] + PS43*_P[12][13] + _P[4][12];
        _nextP[5][12] = PS190*_P[12][15] - PS193*_P[12][13] + PS201*_P[2][12] - PS202*_P[0][12] + PS203*_P[3][12] - PS204*_P[1][12] + PS75*_P[12][14] + _P[5][12];
        _nextP[6][12] = -PS197*_P[12][14] + PS199*_P[12][13] - PS214*_P[2][12] + PS215*_P[3][12] + PS216*_P[0][12] + PS217*_P[1][12] + PS87*_P[12][15] + _P[6][12];
        _nextP[7][12] = _P[4][12]*_dt + _P[7][12];
        _nextP[8][12] = _P[5][12]*_dt + _P[8][12];
        _nextP[9][12] = _P[6][12]*_dt + _P[9][12];
        _nextP[10][12] = _P[10][12];
        _nextP[11][12] = _P[11][12];
        _nextP[12][12] = _P[12][12];

        if (_state_index_lim > 12) {
            _nextP[0][13] = PS44;
            _nextP[1][13] = PS113;
            _nextP[2][13] = PS138;
            _nextP[3][13] = PS158;
            _nextP[4][13] = PS177;
            _nextP[5][13] = PS206;
            _nextP[6][13] = PS221;
            _nextP[7][13] = _P[4][13]*_dt + _P[7][13];
            _nextP[8][13] = _P[5][13]*_dt + _P[8][13];
            _nextP[9][13] = _P[6][13]*_dt + _P[9][13];
            _nextP[10][13] = _P[10][13];
            _nextP[11][13] = _P[11][13];
            _nextP[12][13] = _P[12][13];
            _nextP[13][13] = _P[13][13];
            _nextP[0][14] = PS57;
            _nextP[1][14] = PS117;
            _nextP[2][14] = PS142;
            _nextP[3][14] = PS162;
            _nextP[4][14] = PS180;
            _nextP[5][14] = PS205;
            _nextP[6][14] = PS220;
            _nextP[7][14] = _P[4][14]*_dt + _P[7][14];
            _nextP[8][14] = _P[5][14]*_dt + _P[8][14];
            _nextP[9][14] = _P[6][14]*_dt + _P[9][14];
            _nextP[10][14] = _P[10][14];
            _nextP[11][14] = _P[11][14];
            _nextP[12][14] = _P[12][14];
            _nextP[13][14] = _P[13][14];
            _nextP[14][14] = _P[14][14];
            _nextP[0][15] = PS46;
            _nextP[1][15] = PS114;
            _nextP[2][15] = PS139;
            _nextP[3][15] = PS159;
            _nextP[4][15] = PS178;
            _nextP[5][15] = PS209;
            _nextP[6][15] = PS219;
            _nextP[7][15] = _P[4][15]*_dt + _P[7][15];
            _nextP[8][15] = _P[5][15]*_dt + _P[8][15];
            _nextP[9][15] = _P[6][15]*_dt + _P[9][15];
            _nextP[10][15] = _P[10][15];
            _nextP[11][15] = _P[11][15];
            _nextP[12][15] = _P[12][15];
            _nextP[13][15] = _P[13][15];
            _nextP[14][15] = _P[14][15];
            _nextP[15][15] = _P[15][15];

            if (_state_index_lim > 15) {
                _nextP[0][16] = -PS11*_P[1][16] - PS12*_P[2][16] - PS13*_P[3][16] + PS6*_P[10][16] + PS7*_P[11][16] + PS9*_P[12][16] + _P[0][16];
                _nextP[1][16] = PS11*_P[0][16] - PS12*_P[3][16] + PS13*_P[2][16] - PS34*_P[10][16] - PS7*_P[12][16] + PS9*_P[11][16] + _P[1][16];
                _nextP[2][16] = PS11*_P[3][16] + PS12*_P[0][16] - PS13*_P[1][16] - PS34*_P[11][16] + PS6*_P[12][16] - PS9*_P[10][16] + _P[2][16];
                _nextP[3][16] = -PS11*_P[2][16] + PS12*_P[1][16] + PS13*_P[0][16] - PS34*_P[12][16] - PS6*_P[11][16] + PS7*_P[10][16] + _P[3][16];
                _nextP[4][16] = -PS171*_P[15][16] + PS172*_P[14][16] + PS173*_P[1][16] + PS174*_P[0][16] + PS175*_P[2][16] - PS176*_P[3][16] + PS43*_P[13][16] + _P[4][16];
                _nextP[5][16] = PS190*_P[15][16] - PS193*_P[13][16] + PS201*_P[2][16] - PS202*_P[0][16] + PS203*_P[3][16] - PS204*_P[1][16] + PS75*_P[14][16] + _P[5][16];
                _nextP[6][16] = -PS197*_P[14][16] + PS199*_P[13][16] - PS214*_P[2][16] + PS215*_P[3][16] + PS216*_P[0][16] + PS217*_P[1][16] + PS87*_P[15][16] + _P[6][16];
                _nextP[7][16] = _P[4][16]*_dt + _P[7][16];
                _nextP[8][16] = _P[5][16]*_dt + _P[8][16];
                _nextP[9][16] = _P[6][16]*_dt + _P[9][16];
                _nextP[10][16] = _P[10][16];
                _nextP[11][16] = _P[11][16];
                _nextP[12][16] = _P[12][16];
                _nextP[13][16] = _P[13][16];
                _nextP[14][16] = _P[14][16];
                _nextP[15][16] = _P[15][16];
                _nextP[16][16] = _P[16][16];
                _nextP[0][17] = -PS11*_P[1][17] - PS12*_P[2][17] - PS13*_P[3][17] + PS6*_P[10][17] + PS7*_P[11][17] + PS9*_P[12][17] + _P[0][17];
                _nextP[1][17] = PS11*_P[0][17] - PS12*_P[3][17] + PS13*_P[2][17] - PS34*_P[10][17] - PS7*_P[12][17] + PS9*_P[11][17] + _P[1][17];
                _nextP[2][17] = PS11*_P[3][17] + PS12*_P[0][17] - PS13*_P[1][17] - PS34*_P[11][17] + PS6*_P[12][17] - PS9*_P[10][17] + _P[2][17];
                _nextP[3][17] = -PS11*_P[2][17] + PS12*_P[1][17] + PS13*_P[0][17] - PS34*_P[12][17] - PS6*_P[11][17] + PS7*_P[10][17] + _P[3][17];
                _nextP[4][17] = -PS171*_P[15][17] + PS172*_P[14][17] + PS173*_P[1][17] + PS174*_P[0][17] + PS175*_P[2][17] - PS176*_P[3][17] + PS43*_P[13][17] + _P[4][17];
                _nextP[5][17] = PS190*_P[15][17] - PS193*_P[13][17] + PS201*_P[2][17] - PS202*_P[0][17] + PS203*_P[3][17] - PS204*_P[1][17] + PS75*_P[14][17] + _P[5][17];
                _nextP[6][17] = -PS197*_P[14][17] + PS199*_P[13][17] - PS214*_P[2][17] + PS215*_P[3][17] + PS216*_P[0][17] + PS217*_P[1][17] + PS87*_P[15][17] + _P[6][17];
                _nextP[7][17] = _P[4][17]*_dt + _P[7][17];
                _nextP[8][17] = _P[5][17]*_dt + _P[8][17];
                _nextP[9][17] = _P[6][17]*_dt + _P[9][17];
                _nextP[10][17] = _P[10][17];
                _nextP[11][17] = _P[11][17];
                _nextP[12][17] = _P[12][17];
                _nextP[13][17] = _P[13][17];
                _nextP[14][17] = _P[14][17];
                _nextP[15][17] = _P[15][17];
                _nextP[16][17] = _P[16][17];
                _nextP[17][17] = _P[17][17];
                _nextP[0][18] = -PS11*_P[1][18] - PS12*_P[2][18] - PS13*_P[3][18] + PS6*_P[10][18] + PS7*_P[11][18] + PS9*_P[12][18] + _P[0][18];
                _nextP[1][18] = PS11*_P[0][18] - PS12*_P[3][18] + PS13*_P[2][18] - PS34*_P[10][18] - PS7*_P[12][18] + PS9*_P[11][18] + _P[1][18];
                _nextP[2][18] = PS11*_P[3][18] + PS12*_P[0][18] - PS13*_P[1][18] - PS34*_P[11][18] + PS6*_P[12][18] - PS9*_P[10][18] + _P[2][18];
                _nextP[3][18] = -PS11*_P[2][18] + PS12*_P[1][18] + PS13*_P[0][18] - PS34*_P[12][18] - PS6*_P[11][18] + PS7*_P[10][18] + _P[3][18];
                _nextP[4][18] = -PS171*_P[15][18] + PS172*_P[14][18] + PS173*_P[1][18] + PS174*_P[0][18] + PS175*_P[2][18] - PS176*_P[3][18] + PS43*_P[13][18] + _P[4][18];
                _nextP[5][18] = PS190*_P[15][18] - PS193*_P[13][18] + PS201*_P[2][18] - PS202*_P[0][18] + PS203*_P[3][18] - PS204*_P[1][18] + PS75*_P[14][18] + _P[5][18];
                _nextP[6][18] = -PS197*_P[14][18] + PS199*_P[13][18] - PS214*_P[2][18] + PS215*_P[3][18] + PS216*_P[0][18] + PS217*_P[1][18] + PS87*_P[15][18] + _P[6][18];
                _nextP[7][18] = _P[4][18]*_dt + _P[7][18];
                _nextP[8][18] = _P[5][18]*_dt + _P[8][18];
                _nextP[9][18] = _P[6][18]*_dt + _P[9][18];
                _nextP[10][18] = _P[10][18];
                _nextP[11][18] = _P[11][18];
                _nextP[12][18] = _P[12][18];
                _nextP[13][18] = _P[13][18];
                _nextP[14][18] = _P[14][18];
                _nextP[15][18] = _P[15][18];
                _nextP[16][18] = _P[16][18];
                _nextP[17][18] = _P[17][18];
                _nextP[18][18] = _P[18][18];
                _nextP[0][19] = -PS11*_P[1][19] - PS12*_P[2][19] - PS13*_P[3][19] + PS6*_P[10][19] + PS7*_P[11][19] + PS9*_P[12][19] + _P[0][19];
                _nextP[1][19] = PS11*_P[0][19] - PS12*_P[3][19] + PS13*_P[2][19] - PS34*_P[10][19] - PS7*_P[12][19] + PS9*_P[11][19] + _P[1][19];
                _nextP[2][19] = PS11*_P[3][19] + PS12*_P[0][19] - PS13*_P[1][19] - PS34*_P[11][19] + PS6*_P[12][19] - PS9*_P[10][19] + _P[2][19];
                _nextP[3][19] = -PS11*_P[2][19] + PS12*_P[1][19] + PS13*_P[0][19] - PS34*_P[12][19] - PS6*_P[11][19] + PS7*_P[10][19] + _P[3][19];
                _nextP[4][19] = -PS171*_P[15][19] + PS172*_P[14][19] + PS173*_P[1][19] + PS174*_P[0][19] + PS175*_P[2][19] - PS176*_P[3][19] + PS43*_P[13][19] + _P[4][19];
                _nextP[5][19] = PS190*_P[15][19] - PS193*_P[13][19] + PS201*_P[2][19] - PS202*_P[0][19] + PS203*_P[3][19] - PS204*_P[1][19] + PS75*_P[14][19] + _P[5][19];
                _nextP[6][19] = -PS197*_P[14][19] + PS199*_P[13][19] - PS214*_P[2][19] + PS215*_P[3][19] + PS216*_P[0][19] + PS217*_P[1][19] + PS87*_P[15][19] + _P[6][19];
                _nextP[7][19] = _P[4][19]*_dt + _P[7][19];
                _nextP[8][19] = _P[5][19]*_dt + _P[8][19];
                _nextP[9][19] = _P[6][19]*_dt + _P[9][19];
                _nextP[10][19] = _P[10][19];
                _nextP[11][19] = _P[11][19];
                _nextP[12][19] = _P[12][19];
                _nextP[13][19] = _P[13][19];
                _nextP[14][19] = _P[14][19];
                _nextP[15][19] = _P[15][19];
                _nextP[16][19] = _P[16][19];
                _nextP[17][19] = _P[17][19];
                _nextP[18][19] = _P[18][19];
                _nextP[19][19] = _P[19][19];
                _nextP[0][20] = -PS11*_P[1][20] - PS12*_P[2][20] - PS13*_P[3][20] + PS6*_P[10][20] + PS7*_P[11][20] + PS9*_P[12][20] + _P[0][20];
                _nextP[1][20] = PS11*_P[0][20] - PS12*_P[3][20] + PS13*_P[2][20] - PS34*_P[10][20] - PS7*_P[12][20] + PS9*_P[11][20] + _P[1][20];
                _nextP[2][20] = PS11*_P[3][20] + PS12*_P[0][20] - PS13*_P[1][20] - PS34*_P[11][20] + PS6*_P[12][20] - PS9*_P[10][20] + _P[2][20];
                _nextP[3][20] = -PS11*_P[2][20] + PS12*_P[1][20] + PS13*_P[0][20] - PS34*_P[12][20] - PS6*_P[11][20] + PS7*_P[10][20] + _P[3][20];
                _nextP[4][20] = -PS171*_P[15][20] + PS172*_P[14][20] + PS173*_P[1][20] + PS174*_P[0][20] + PS175*_P[2][20] - PS176*_P[3][20] + PS43*_P[13][20] + _P[4][20];
                _nextP[5][20] = PS190*_P[15][20] - PS193*_P[13][20] + PS201*_P[2][20] - PS202*_P[0][20] + PS203*_P[3][20] - PS204*_P[1][20] + PS75*_P[14][20] + _P[5][20];
                _nextP[6][20] = -PS197*_P[14][20] + PS199*_P[13][20] - PS214*_P[2][20] + PS215*_P[3][20] + PS216*_P[0][20] + PS217*_P[1][20] + PS87*_P[15][20] + _P[6][20];
                _nextP[7][20] = _P[4][20]*_dt + _P[7][20];
                _nextP[8][20] = _P[5][20]*_dt + _P[8][20];
                _nextP[9][20] = _P[6][20]*_dt + _P[9][20];
                _nextP[10][20] = _P[10][20];
                _nextP[11][20] = _P[11][20];
                _nextP[12][20] = _P[12][20];
                _nextP[13][20] = _P[13][20];
                _nextP[14][20] = _P[14][20];
                _nextP[15][20] = _P[15][20];
                _nextP[16][20] = _P[16][20];
                _nextP[17][20] = _P[17][20];
                _nextP[18][20] = _P[18][20];
                _nextP[19][20] = _P[19][20];
                _nextP[20][20] = _P[20][20];
                _nextP[0][21] = -PS11*_P[1][21] - PS12*_P[2][21] - PS13*_P[3][21] + PS6*_P[10][21] + PS7*_P[11][21] + PS9*_P[12][21] + _P[0][21];
                _nextP[1][21] = PS11*_P[0][21] - PS12*_P[3][21] + PS13*_P[2][21] - PS34*_P[10][21] - PS7*_P[12][21] + PS9*_P[11][21] + _P[1][21];
                _nextP[2][21] = PS11*_P[3][21] + PS12*_P[0][21] - PS13*_P[1][21] - PS34*_P[11][21] + PS6*_P[12][21] - PS9*_P[10][21] + _P[2][21];
                _nextP[3][21] = -PS11*_P[2][21] + PS12*_P[1][21] + PS13*_P[0][21] - PS34*_P[12][21] - PS6*_P[11][21] + PS7*_P[10][21] + _P[3][21];
                _nextP[4][21] = -PS171*_P[15][21] + PS172*_P[14][21] + PS173*_P[1][21] + PS174*_P[0][21] + PS175*_P[2][21] - PS176*_P[3][21] + PS43*_P[13][21] + _P[4][21];
                _nextP[5][21] = PS190*_P[15][21] - PS193*_P[13][21] + PS201*_P[2][21] - PS202*_P[0][21] + PS203*_P[3][21] - PS204*_P[1][21] + PS75*_P[14][21] + _P[5][21];
                _nextP[6][21] = -PS197*_P[14][21] + PS199*_P[13][21] - PS214*_P[2][21] + PS215*_P[3][21] + PS216*_P[0][21] + PS217*_P[1][21] + PS87*_P[15][21] + _P[6][21];
                _nextP[7][21] = _P[4][21]*_dt + _P[7][21];
                _nextP[8][21] = _P[5][21]*_dt + _P[8][21];
                _nextP[9][21] = _P[6][21]*_dt + _P[9][21];
                _nextP[10][21] = _P[10][21];
                _nextP[11][21] = _P[11][21];
                _nextP[12][21] = _P[12][21];
                _nextP[13][21] = _P[13][21];
                _nextP[14][21] = _P[14][21];
                _nextP[15][21] = _P[15][21];
                _nextP[16][21] = _P[16][21];
                _nextP[17][21] = _P[17][21];
                _nextP[18][21] = _P[18][21];
                _nextP[19][21] = _P[19][21];
                _nextP[20][21] = _P[20][21];
                _nextP[21][21] = _P[21][21];

                if (_state_index_lim > 21) {
                    _nextP[0][22] = -PS11*_P[1][22] - PS12*_P[2][22] - PS13*_P[3][22] + PS6*_P[10][22] + PS7*_P[11][22] + PS9*_P[12][22] + _P[0][22];
                    _nextP[1][22] = PS11*_P[0][22] - PS12*_P[3][22] + PS13*_P[2][22] - PS34*_P[10][22] - PS7*_P[12][22] + PS9*_P[11][22] + _P[1][22];
                    _nextP[2][22] = PS11*_P[3][22] + PS12*_P[0][22] - PS13*_P[1][22] - PS34*_P[11][22] + PS6*_P[12][22] - PS9*_P[10][22] + _P[2][22];
                    _nextP[3][22] = -PS11*_P[2][22] + PS12*_P[1][22] + PS13*_P[0][22] - PS34*_P[12][22] - PS6*_P[11][22] + PS7*_P[10][22] + _P[3][22];
                    _nextP[4][22] = -PS171*_P[15][22] + PS172*_P[14][22] + PS173*_P[1][22] + PS174*_P[0][22] + PS175*_P[2][22] - PS176*_P[3][22] + PS43*_P[13][22] + _P[4][22];
                    _nextP[5][22] = PS190*_P[15][22] - PS193*_P[13][22] + PS201*_P[2][22] - PS202*_P[0][22] + PS203*_P[3][22] - PS204*_P[1][22] + PS75*_P[14][22] + _P[5][22];
                    _nextP[6][22] = -PS197*_P[14][22] + PS199*_P[13][22] - PS214*_P[2][22] + PS215*_P[3][22] + PS216*_P[0][22] + PS217*_P[1][22] + PS87*_P[15][22] + _P[6][22];
                    _nextP[7][22] = _P[4][22]*_dt + _P[7][22];
                    _nextP[8][22] = _P[5][22]*_dt + _P[8][22];
                    _nextP[9][22] = _P[6][22]*_dt + _P[9][22];
                    _nextP[10][22] = _P[10][22];
                    _nextP[11][22] = _P[11][22];
                    _nextP[12][22] = _P[12][22];
                    _nextP[13][22] = _P[13][22];
                    _nextP[14][22] = _P[14][22];
                    _nextP[15][22] = _P[15][22];
                    _nextP[16][22] = _P[16][22];
                    _nextP[17][22] = _P[17][22];
                    _nextP[18][22] = _P[18][22];
                    _nextP[19][22] = _P[19][22];
                    _nextP[20][22] = _P[20][22];
                    _nextP[21][22] = _P[21][22];
                    _nextP[22][22] = _P[22][22];
                    _nextP[0][23] = -PS11*_P[1][23] - PS12*_P[2][23] - PS13*_P[3][23] + PS6*_P[10][23] + PS7*_P[11][23] + PS9*_P[12][23] + _P[0][23];
                    _nextP[1][23] = PS11*_P[0][23] - PS12*_P[3][23] + PS13*_P[2][23] - PS34*_P[10][23] - PS7*_P[12][23] + PS9*_P[11][23] + _P[1][23];
                    _nextP[2][23] = PS11*_P[3][23] + PS12*_P[0][23] - PS13*_P[1][23] - PS34*_P[11][23] + PS6*_P[12][23] - PS9*_P[10][23] + _P[2][23];
                    _nextP[3][23] = -PS11*_P[2][23] + PS12*_P[1][23] + PS13*_P[0][23] - PS34*_P[12][23] - PS6*_P[11][23] + PS7*_P[10][23] + _P[3][23];
                    _nextP[4][23] = -PS171*_P[15][23] + PS172*_P[14][23] + PS173*_P[1][23] + PS174*_P[0][23] + PS175*_P[2][23] - PS176*_P[3][23] + PS43*_P[13][23] + _P[4][23];
                    _nextP[5][23] = PS190*_P[15][23] - PS193*_P[13][23] + PS201*_P[2][23] - PS202*_P[0][23] + PS203*_P[3][23] - PS204*_P[1][23] + PS75*_P[14][23] + _P[5][23];
                    _nextP[6][23] = -PS197*_P[14][23] + PS199*_P[13][23] - PS214*_P[2][23] + PS215*_P[3][23] + PS216*_P[0][23] + PS217*_P[1][23] + PS87*_P[15][23] + _P[6][23];
                    _nextP[7][23] = _P[4][23]*_dt + _P[7][23];
                    _nextP[8][23] = _P[5][23]*_dt + _P[8][23];
                    _nextP[9][23] = _P[6][23]*_dt + _P[9][23];
                    _nextP[10][23] = _P[10][23];
                    _nextP[11][23] = _P[11][23];
                    _nextP[12][23] = _P[12][23];
                    _nextP[13][23] = _P[13][23];
                    _nextP[14][23] = _P[14][23];
                    _nextP[15][23] = _P[15][23];
                    _nextP[16][23] = _P[16][23];
                    _nextP[17][23] = _P[17][23];
                    _nextP[18][23] = _P[18][23];
                    _nextP[19][23] = _P[19][23];
                    _nextP[20][23] = _P[20][23];
                    _nextP[21][23] = _P[21][23];
                    _nextP[22][23] = _P[22][23];
                    _nextP[23][23] = _P[23][23];
                }
            }
        }
    }

    // add the general state process noise variances
    if (_state_index_lim > 9) {
        for (uint8_t i=10; i<=_state_index_lim; i++) {
            _nextP[i][i] = _nextP[i][i] + process_noise_variance[i-10];
        }
    }

    // inactive delta velocity bias states have all covariances zeroed to prevent
    // interacton with other states
    if (!_inhibit_del_vel_bias_states) {
        for (uint8_t index=0; index<3; index++) {
            const uint8_t stateIndex = index + 13;
            if (_dvel_bias_axis_inhibit[index]) {
                ekf3_core_zero_cols(&_nextP,stateIndex,stateIndex);
                _nextP[stateIndex][stateIndex] = ((float *)&_dvel_bias_axis_var_prev)[index];
            }
        }
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((_P[7][7] + _P[8][8]) > 1e4f) {
        for (uint8_t i=7; i<=8; i++)
        {
            for (uint8_t j=0; j<=_state_index_lim; j++)
            {
                _nextP[i][j] = _P[i][j];
                _nextP[j][i] = _P[j][i];
            }
        }
    }

    // covariance matrix is symmetrical, so copy diagonals and copy lower half in _nextP
    // to lower and upper half in P
    for (uint8_t row = 0; row <= _state_index_lim; row++) {
        // copy diagonals
        _P[row][row] = _nextP[row][row];
        // copy off diagonals
        for (uint8_t column = 0 ; column < row; column++) {
            _P[row][column] = _P[column][row] = _nextP[column][row];
        }
    }

    // constrain values to prevent ill-conditioning
    ekf3_core_constrain_variances();

    if (_vert_vel_var_clip_counter > 0) {
        _vert_vel_var_clip_counter--;
    }

    calc_tilt_error_variance();
}

void ekf3_core_covariance_init(void)
{
    memset(&_P[0][0], 0, sizeof(_P));
    vector3f_t rot_vec_var;
    rot_vec_var.x = rot_vec_var.y = rot_vec_var.z = sq(0.1f);
    ekf3_core_covariance_prediction(&rot_vec_var);
    _P[4][4] = sq(_gps_horiz_vel_noise);
    _P[5][5] = _P[4][4];
    _P[6][6] = sq(_gps_vert_vel_noise);
    _P[7][7] = sq(_gps_horiz_pos_noise);
    _P[8][8] = _P[7][7];
    _P[9][9] = sq(_baro_alt_noise);
    _P[10][10] = sq(radians(ekf3_core_initial_gyro_bias_uncertainty() * _dt_ekf_avg));
    _P[11][11] = _P[10][10];
    _P[12][12] = _P[10][10];
    _P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * _acc_bias_lim * _dt_ekf_avg);
    _P[14][14] = _P[13][13];
    _P[15][15] = _P[13][13];
    _P[16][16] = sq(_mag_noise);
    _P[17][17] = _P[16][16];
    _P[18][18] = _P[16][16];
    _P[19][19] = sq(_mag_noise);
    _P[20][20] = _P[19][19];
    _P[21][21] = _P[19][19];
    _P[22][22] = 0;
    _P[23][23] = _P[22][22];
    _P_opt = 0.25f;
}

void stored_output_reset(void)
{
    _output_data_new.quat = _s.state_struct.quat;
    //MY_LOG("stored output reset output.quat %f\n", _output_data_new.quat.q1);
    _output_data_new.velocity = _s.state_struct.velocity;
    _output_data_new.position = _s.state_struct.position;
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        *((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i)) = _output_data_new;
    }
    _output_data_delayed = _output_data_new;
    _vert_comp_filt_state.pos = _s.state_struct.position.z;
    _vert_comp_filt_state.vel = _s.state_struct.velocity.z;
}

bool init_ekf3_core_filter_bootstrap(void)
{
    //MY_LOG("ekf3 core bootstrap\n");
    ekf3_core_update_sensor_selection();
    if (ekf3_core_assume_zero_sideslip() && dal_gps_status() < DAL_GPS_OK_FIX_3D) {
        MY_LOG("EKF3 init failure: No GPS lock\n");
        _states_initialised = false;
        return false;
    }
    ekf3_core_read_imu_data();
    ekf3_core_read_mag_data();
    ekf3_core_read_gps_data();
    if (_states_initialised) {
        //MY_LOG("imu buffer is filled?\n");
        return ekf_imu_buffer_is_filled(&_stored_imu);
    }
    if (first_init_time_ms == 0) {
        //MY_LOG("first init time =0\n");
        first_init_time_ms = _imu_sample_time_ms;
        return false;
    } else if (_imu_sample_time_ms - first_init_time_ms < 1000) {
        MY_LOG("imu sample_time_ms < first init_time_ms\n");
        return false;
    }
    initialise_variables();
    vector3f_t init_acc_vec;
    init_acc_vec = dal_ins_get_accel();
    float __attribute__((unused))pitch = 0, __attribute__((unused))roll = 0;
    if (v3f_length(&init_acc_vec) > 0.001f) {
        init_acc_vec = v3f_normalized(&init_acc_vec);
        pitch = asinf(init_acc_vec.x);
        roll = atan2f(-init_acc_vec.y, -init_acc_vec.z);
    }
    quat_from_euler(&_s.state_struct.quat, roll, pitch, 0);
    v3f_zero(&_s.state_struct.velocity);
    v3f_zero(&_s.state_struct.position);
    v3f_zero(&_s.state_struct.gyro_bias);
    v3f_zero(&_s.state_struct.accel_bias);
    v2f_zero(&_s.state_struct.wind_vel);
    v3f_zero(&_s.state_struct.earth_magfield);
    v3f_zero(&_s.state_struct.body_magfield);

    ekf3_core_reset_velocity(RESET_DATA_SOURCE_DEFAULT);
    ekf3_core_reset_position(RESET_DATA_SOURCE_DEFAULT);
    ekf3_core_reset_height();
    _posxy_source_last = ekf_source_get_posxy_source();
    _yaw_source_last = ekf_source_get_yaw_source();

    ekf3_core_calc_earth_rate_ned(&_earth_rate_ned, dal_get_home()->lat);
    ekf3_core_covariance_init();
    stored_output_reset();
    _states_initialised = true;
    v3f_zero(&_inactive_bias.gyro_bias);
    v3f_zero(&_inactive_bias.accel_bias);
    MY_LOG("EKF3 IMU initialised\n");

    return false;
}

source_yaw_t ekf3_get_yaw_source(void)
{
    /* if ((_source_set.yaw == YAW_COMPASS) && (compass_enabled() == 0)) { */
    /*     return YAW_NONE; */
    /* } */
    return _source_set.yaw;
}

void ekf3_core_correct_delta_angle(__attribute__((unused))vector3f_t *del_ang,
                                   __attribute__((unused))float del_ang_dt)
{
    //MY_LOG("gyro bias %f %f %f, dt: %f, dt_ekf_avg: %f\n", _inactive_bias.gyro_bias.x,
    //       _inactive_bias.gyro_bias.y, _inactive_bias.gyro_bias.z, del_ang_dt, _dt_ekf_avg);
    vector3f_t tmp = v3f_uniform_scale(&_inactive_bias.gyro_bias, del_ang_dt / _dt_ekf_avg);
    *del_ang = v3f_sub(del_ang, &tmp);
    //MY_LOG("del ang: %f %f %f\n", del_ang->x, del_ang->y, del_ang->z);
}

void ekf3_core_correct_delta_velocity(vector3f_t *del_vel, float del_vel_dt)
{
    vector3f_t tmp = v3f_uniform_scale(&_inactive_bias.accel_bias, del_vel_dt / _dt_ekf_avg);
    *del_vel = v3f_sub(del_vel, &tmp);
}

void ekf3_core_calc_earth_rate_ned(vector3f_t *omega, int32_t latitude)
{
    float lat_rad = radians(latitude * 1.0e-7f);
    omega->x = earthRate * cosf(lat_rad);
    omega->y = 0;
    omega->z = -earthRate * sinf(lat_rad);
    //MY_LOG("lat: %f rate: %f %f %f\n", latitude*1e-7, omega->x, omega->y, omega->z); //0.000063 0 -0.000037
}

void ekf3_core_zero_range(float *v, uint8_t n1, uint8_t n2)
{
    memset(&v[n1], 0, sizeof(float) * (1 + n2 - n1));
}

void ekf3_core_zero_rows(Matrix24 *cov_mat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = first; row <= last; row++) {
        ekf3_core_zero_range((float *)cov_mat + row * 24, 0, 23);
    }
}

void ekf3_core_zero_cols(Matrix24 *cov_mat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row = 0; row <= 23; row++) {
        ekf3_core_zero_range((float *)cov_mat + row * 24, first, last);
    }
}

bool ekf3_core_get_pos_d(float *pos_d)
{
    if ((_origin_hgt_mode & (1 << 2)) == 0) {
        *pos_d = _output_data_new.position.z + _pos_offset_ned.z;
    } else {
        *pos_d = _output_data_new.position.z + _pos_offset_ned.z + 0.01 * (float)_ekf_origin.alt -
            (float)_ekf_gps_ref_hgt;
    }
    return _filter_status.flags.vert_pos;
}

bool ekf3_core_get_llh(__attribute__((unused))location_t *loc)
{
    location_t origin;
    if (ekf3_core_get_origin_llh(&origin)) {
        float pos_d;
        if (ekf3_core_get_pos_d(&pos_d) && _pv_aiding_mode != AID_NONE) {
            loc->alt = origin.alt - pos_d * 100;
            loc->relative_alt = 0;
            loc->terrain_alt = 0;
            if (_filter_status.flags.horiz_pos_abs || _filter_status.flags.horiz_pos_rel) {
                loc->lat = _ekf_origin.lat;
                loc->lng = _ekf_origin.lng;
                location_offset(loc, _output_data_new.position.x + _pos_offset_ned.x,
                                _output_data_new.position.y + _pos_offset_ned.y);
                //location_offset(loc, _s.state_struct.position.x, _s.state_struct.position.y);
                //MY_LOG("get llh\n");
                return true;
            } else {
                if (ekf3_core_get_gps_llh(loc)) {
                    return true;
                } else {
                    loc->alt = _ekf_origin.lat;
                    loc->lng = _ekf_origin.lng;
                    location_offset(loc, _output_data_new.position.x + _pos_offset_ned.x,
                                    _output_data_new.position.y + _pos_offset_ned.y);
                    return false;
                }
            }
        } else {
            if (ekf3_core_get_gps_llh(loc)) {
                return true;
            } else {
                loc->lat = _ekf_origin.lat;
                loc->lng = _ekf_origin.lng;
                location_offset(loc, _last_known_position_ne.x + _pos_offset_ned.x,
                                _last_known_position_ne.y + _pos_offset_ned.y);
                return false;
            }
        }
    } else {
        return ekf3_core_get_gps_llh(loc);
    }
}

void ekf3_core_force_symmetry(void)
{
    for (uint8_t i = 1; i <= _state_index_lim ; i++) {
        for (uint8_t j = 0; j <= i - 1; j++) {
            float temp = 0.5f * (_P[i][j] + _P[j][i]);
            _P[i][j] = temp;
            _P[j][i] = temp;
        }
    }
}

void ekf3_core_constrain_variances(void)
{
    for (uint8_t i = 0; i <= 3 ; i++) {
        _P[i][i] = constrain_float(_P[i][i], 0.0, 1.0);
    }
    for (uint8_t i = 4; i <= 5; i++) {
        _P[i][i] = constrain_float(_P[i][i], VEL_STATE_MIN_VARIANCE, 1.0e3);
    }
    if (_bad_imu_data) {
        _P[6][6] = fmaxf(_P[6][6], sq(_gps_vert_vel_noise));
        _P[9][9] = fmaxf(_P[9][9], sq(_baro_alt_noise));
    } else if (_P[6][6] < VEL_STATE_MIN_VARIANCE) {
        _P[6][6] = VEL_STATE_MIN_VARIANCE;
        _vert_vel_var_clip_counter += EKF_TARGET_RATE_HZ;
        if (_vert_vel_var_clip_counter > VERT_VEL_VAR_CLIP_COUNT_LIM) {
           ekf3_core_zero_rows(&_P, 6, 6);
           ekf3_core_zero_cols(&_P, 6, 6);
           _P[6][6] = _gps_vert_vel_noise;
           _vert_vel_var_clip_counter = 0;
        }
    }
    for (uint8_t i =7; i <= 9; i++) {
        _P[i][i] = constrain_float(_P[i][i], POS_STATE_MIN_VARIANCE, 1.0e6);
    }
    if (!_inhibit_del_ang_bias_states) {
        for (uint8_t i = 10; i <= 12; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, sq(0.175 * _dt_ekf_avg));
        }
    } else {
        ekf3_core_zero_cols(&_P, 10, 12);
        ekf3_core_zero_rows(&_P, 10, 12);
    }
    const float min_state_var_target = 1e-11;
    if (!_inhibit_del_vel_bias_states) {
        const float min_safe_state_var = min_state_var_target * 0.1f;
        float max_state_var = min_safe_state_var;
        bool reset_required = false;
        for (uint8_t state_index = 13; state_index <= 15; state_index++) {
            if (_P[state_index][state_index] > max_state_var) {
                max_state_var = _P[state_index][state_index];
            } else if (_P[state_index][state_index] < min_safe_state_var) {
                reset_required = true;
            }
        }
        float min_allowed_state_var = fmaxf(0.01f * max_state_var, min_state_var_target);
        for (uint8_t state_index = 13; state_index <= 15; state_index++) {
            _P[state_index][state_index] = constrain_float(_P[state_index][state_index],
                                                           min_allowed_state_var,
                                                           sq(10.0f * _dt_ekf_avg));
        }
        if (reset_required) {
            float del_vel_bias_var[3];
            for (uint8_t i = 0; i <= 2; i++) {
                del_vel_bias_var[i] = _P[i + 13][i + 13];
            }
            ekf3_core_zero_cols(&_P, 13, 15);
            ekf3_core_zero_rows(&_P, 13, 15);
            for (uint8_t i = 0; i <= 2; i++) {
                _P[i + 13][i + 13] = del_vel_bias_var[i];
            }
        }
    } else {
        ekf3_core_zero_cols(&_P, 13, 15);
        ekf3_core_zero_rows(&_P, 13, 15);
        for (uint8_t i = 0; i <= 2; i++) {
            const uint8_t state_index = i + 13;
            _P[state_index][state_index] = fmaxf(_P[state_index][state_index], min_state_var_target);
        }
    }
    if (!_inhibit_mag_states) {
        for (uint8_t i = 16; i <= 18; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, 0.01f);
        }
        for (uint8_t i = 19; i <= 21; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, 0.01f);
        }
    } else {
        ekf3_core_zero_cols(&_P, 16, 21);
        ekf3_core_zero_rows(&_P, 16, 21);
    }
    if (!_inhibit_wind_states) {
        for (uint8_t i = 22; i <= 23; i++) {
            _P[i][i] = constrain_float(_P[i][i], 0.0f, WIND_VEL_VARIANCE_MAX);
        }
    } else {
        ekf3_core_zero_cols(&_P, 22, 23);
        ekf3_core_zero_rows(&_P, 22, 23);

    }
}

bool ekf3_core_get_origin_llh(location_t *loc)
{
    if (_valid_origin) {
        *loc = *_public_origin;
        if ((_origin_hgt_mode & (1<<2)) == 0) {
            loc->alt = (int32_t)(100.0f * (float)_ekf_gps_ref_hgt);
        }
    }
    return _valid_origin;
}

uint8_t ekf3_core_get_frames_since_predict(void)
{
    return _frames_since_predict;
}

void ekf3_core_mag_table_constrain(void)
{
    float limit_ga = _mag_ef_limit * 0.001f;
    _s.state_struct.earth_magfield.x = constrain_float(_s.state_struct.earth_magfield.x,
                                                       _table_earth_field_ga.x - limit_ga,
                                                       _table_earth_field_ga.x + limit_ga);
    _s.state_struct.earth_magfield.y = constrain_float(_s.state_struct.earth_magfield.y,
                                                       _table_earth_field_ga.y - limit_ga,
                                                       _table_earth_field_ga.y + limit_ga);
    _s.state_struct.earth_magfield.z = constrain_float(_s.state_struct.earth_magfield.z,
                                                       _table_earth_field_ga.z - limit_ga,
                                                       _table_earth_field_ga.z + limit_ga);

}

static void constrain_states(void)
{
    for (uint8_t i = 0; i <=3; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -1.0f, 1.0f);
    }
    for (uint8_t i = 4; i <= 6; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -5.0e2f, 5.0e2f);
    }
    for (uint8_t i = 7; i <= 8; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -EKF3_POSXY_STATE_LIMIT, EKF3_POSXY_STATE_LIMIT);
    }
    _s.state_struct.position.z = constrain_float(_s.state_struct.position.z, -4.0e4f, 1.0e4f);
    for (uint8_t i = 10; i <= 12; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -GYRO_BIAS_LIMIT *_dt_ekf_avg, GYRO_BIAS_LIMIT *_dt_ekf_avg);
    }
    for (uint8_t i = 13; i <= 15; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -_acc_bias_lim * _dt_ekf_avg, _acc_bias_lim * _dt_ekf_avg);
    }
    if (_mag_ef_limit <= 0 || !_have_table_earth_field) {
        for (uint8_t i = 16; i <= 18; i++) {
            _s.states_array[i] = constrain_float(_s.states_array[i], -1.0f, 1.0f);
        }
    } else {
        ekf3_core_mag_table_constrain();
    }
    for (uint8_t i = 19; i <= 21; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -0.5f, 0.5f);
    }
    for (uint8_t i = 22; i <= 23; i++) {
        _s.states_array[i] = constrain_float(_s.states_array[i], -100, 100);
    }
    if (!_inhibit_gnd_state) {
        _terrain_state = MAX(_terrain_state, _s.state_struct.position.z + _rng_on_gnd);
    }
}

static void update_strap_down_equations_ned(void)
{
    vector3f_t tmp = m3f_multi_v(&_prev_tnb, &_earth_rate_ned);
    tmp = v3f_uniform_scale(&tmp, _imu_data_delayed.del_ang_dt);
    //MY_LOG("earth rate angle:%f %f %f\n", tmp.x, tmp.y, tmp.z);
    tmp = v3f_sub(&_del_ang_corrected, &tmp);
    //MY_LOG("delta angle:%f %f %f\n", tmp.x, tmp.y, tmp.z);
    quat_rotate_v(&_s.state_struct.quat, &tmp);
    quat_normalize(&_s.state_struct.quat);
    vector3f_t del_vel_nav;
    del_vel_nav = m3f_mul_transpose(&_prev_tnb, &_del_vel_corrected);
    del_vel_nav.z += GRAVITY_MSS * _imu_data_delayed.del_vel_dt;
    //MY_LOG("del_vel :%f %f %f\n", del_vel_nav.x, del_vel_nav.y, del_vel_nav.z);
    quaternionf_t quat_tmp = quat_inverse(&_s.state_struct.quat);
    quat_to_rotation_matrix(&quat_tmp, &_prev_tnb);
    _vel_dot_ned = v3f_div(&del_vel_nav, _imu_data_delayed.del_vel_dt);
    vector3f_t tmp1, tmp2;
    tmp1 = v3f_uniform_scale(&_vel_dot_ned, 0.05);
    tmp2 = v3f_uniform_scale(&_vel_dot_ned_filt, 0.95);
    _vel_dot_ned_filt = v3f_add(&tmp1, &tmp2);
    _acc_nav_mag = v3f_length(&_vel_dot_ned_filt);
    vector2f_t tmp3 = {_vel_dot_ned_filt.x, _vel_dot_ned_filt.y};
    _acc_nav_mag_horiz = v2f_length(&tmp3);
    if ((_pv_aiding_mode == AID_NONE) && (_acc_nav_mag_horiz > 5.0f)) {
        float gain = 5.0 / _acc_nav_mag_horiz;
        del_vel_nav.x *= gain;
        del_vel_nav.y *= gain;
    }
    vector3f_t last_velocity = _s.state_struct.velocity;
    _s.state_struct.velocity = v3f_add(&_s.state_struct.velocity, &del_vel_nav);
    tmp = v3f_add(&last_velocity, &_s.state_struct.velocity);
    tmp = v3f_uniform_scale(&tmp, _imu_data_delayed.del_vel_dt * 0.5f);
    _s.state_struct.position = v3f_add(&_s.state_struct.position, &tmp);
    /* MY_LOG("vel:%f %f %f, pos %f %f %f, p44 %f p55 %f\n", _s.state_struct.velocity.x, */
    /*        _s.state_struct.velocity.y, _s.state_struct.velocity.z, */
    /*        _s.state_struct.position.x, _s.state_struct.position.y, _s.state_struct.position.z, */
    /*        _P[4][4], _P[5][5]); */
    //_del_ang_body_of += _del_ang_corrected;
    //del_time_of += _imu_data_delayed.del_ang_dt;
    constrain_states();
    /* if (_filter_status.floag.horiz_vel) { */
    /* } */
}

static void move_ekf_origin(void)
{
    if (_common_origin_valid || !_filter_status.flags.using_gps) {
        return;
    }
    MY_LOG("ekf origin moved\n");
    location_t loc = _ekf_origin;
    location_offset(&loc, _s.state_struct.position.x, _s.state_struct.position.y);
    const vector2f_t diff_ne = location_get_distance_ne_float(&loc, &_ekf_origin);
    _ekf_origin = loc;
    _s.state_struct.position.x += diff_ne.x;
    _s.state_struct.position.y += diff_ne.y;
    _output_data_new.position.x += diff_ne.x;
    _output_data_new.position.y += diff_ne.y;
    _output_data_delayed.position.x += diff_ne.x;
    _output_data_delayed.position.y += diff_ne.y;
}

static uint32_t last_output_time_ms = 0;
static void calc_output_states(void)
{
    vector3f_t del_ang_new_corrected = _imu_data_new.del_ang;
    /* MY_LOG("imu data new.del ang %f %f %f\n", _imu_data_new.del_ang.x, */
    /*        _imu_data_new.del_ang.y, _imu_data_new.del_ang.z); */
    vector3f_t del_vel_new_corrected = _imu_data_new.del_vel;
    ekf3_core_correct_delta_angle(&del_ang_new_corrected, _imu_data_new.del_ang_dt);
    ekf3_core_correct_delta_velocity(&del_vel_new_corrected, _imu_data_new.del_vel_dt);
    /* MY_LOG("del ang new cor: %f %f %f, correction: %f %f %f\n", */
    /*        del_ang_new_corrected.x, del_ang_new_corrected.y, del_ang_new_corrected.z, */
    /*        _del_ang_correction.x, _del_ang_correction.y, _del_ang_correction.z); */
    /* MY_LOG("imu data new dvel: %f %f %f, corre: %f %f %f\n", _imu_data_new.del_vel.x, */
    /*        _imu_data_new.del_vel.y, _imu_data_new.del_vel.z, del_vel_new_corrected.x, */
    /*        del_vel_new_corrected.y, del_vel_new_corrected.z); */
    vector3f_t del_ang = v3f_add(&del_ang_new_corrected, &_del_ang_correction);
    quaternionf_t delta_quat;
    quat_from_axis_angle_v3f(&delta_quat, del_ang);
    _output_data_new.quat = quat_multi(&_output_data_new.quat, &delta_quat);
    quat_normalize(&_output_data_new.quat);
    matrix3f_t tbn_temp;
    quat_to_rotation_matrix(&_output_data_new.quat, &tbn_temp);
    vector3f_t del_vel_nav = m3f_multi_v(&tbn_temp, &del_vel_new_corrected);
    del_vel_nav.z += GRAVITY_MSS * _imu_data_new.del_vel_dt;
    vector3f_t last_velocity = _output_data_new.velocity;
    _output_data_new.velocity = v3f_add(&_output_data_new.velocity, &del_vel_nav);
    const float comp_filt_omega = M_2PI * constrain_float(_hrt_filt_freq, 0.1, 30);
    float omega2 = comp_filt_omega * comp_filt_omega;
    float pos_err = constrain_float(_output_data_new.position.z - _vert_comp_filt_state.pos,
                                    -1e5, 1e5);
    float integ1_input = pos_err * omega2 * comp_filt_omega * _imu_data_new.del_vel_dt;
    _vert_comp_filt_state.acc += integ1_input;
    float integ2_input = del_vel_nav.z + (_vert_comp_filt_state.acc + pos_err * omega2 * 3) *
        _imu_data_new.del_vel_dt;
    _vert_comp_filt_state.vel += integ2_input;
    float integ3_input = (_vert_comp_filt_state.vel + pos_err * comp_filt_omega * 3) *
        _imu_data_new.del_vel_dt;
    _vert_comp_filt_state.pos += integ3_input;
    vector3f_t vel_tmp = v3f_add(&_output_data_new.velocity, &last_velocity);
    vector3f_t pos_tmp = v3f_uniform_scale(&vel_tmp, _imu_data_new.del_vel_dt * 0.5);
    _output_data_new.position = v3f_add(&_output_data_new.position, &pos_tmp);
    if (!v3f_is_zero(&_accel_pos_offset)) {
        vector3f_t ang_rate = v3f_uniform_scale(&_imu_data_new.del_ang, 1/_imu_data_new.del_ang_dt);
        vector3f_t tmp = v3f_uniform_scale(&_accel_pos_offset, -1);
        vector3f_t vel_body_rel_imu = v3f_cross_product(&ang_rate, &tmp);
        _vel_offset_ned = m3f_multi_v(&tbn_temp, &vel_body_rel_imu);
        _pos_offset_ned = m3f_multi_v(&tbn_temp, &tmp);
    } else {
        v3f_zero(&_vel_offset_ned);
        v3f_zero(&_pos_offset_ned);
    }
    if (_run_updates) {
        uint8_t yng = ekf_imu_buffer_get_youngest_index(&_stored_imu);
        *((output_elements_t *)ekf_imu_buffer_get(&_stored_output, yng)) = _output_data_new;
        uint8_t old = ekf_imu_buffer_get_oldest_index(&_stored_imu);
        _output_data_delayed = *((output_elements_t *)ekf_imu_buffer_get(&_stored_output, old));
        quaternionf_t quat_err = quat_div(&_s.state_struct.quat, &_output_data_delayed.quat);
        quat_normalize(&quat_err);
        vector3f_t delta_ang_err;
        float scaler;
        if (quat_err.q1 >= 0.0f) {
            scaler = 2.0f;
        } else {
            scaler = -2.0f;
        }
        delta_ang_err.x = scaler * quat_err.q2;
        delta_ang_err.y = scaler * quat_err.q3;
        delta_ang_err.z = scaler * quat_err.q4;
        float time_delay = 1e-3f * (float)(_imu_data_new.time_ms - _imu_data_delayed.time_ms);
        time_delay = MAX(time_delay, _dt_imu_avg);
        float error_gain = 0.5 / time_delay;
        _del_ang_correction = v3f_uniform_scale(&delta_ang_err, error_gain * _dt_imu_avg);
        vector3f_t vel_err = v3f_sub(&_s.state_struct.velocity, &_output_data_delayed.velocity);
        vector3f_t pos_err = v3f_sub(&_s.state_struct.position, &_output_data_delayed.position);
        /* MY_LOG("yng %d old %d,new %f %f %f ,output data delayed %f %f %f\n", yng, old, */
        /*        _output_data_new.position.x, _output_data_new.position.y, _output_data_new.position.z, */
        /*        _output_data_delayed.position.x, _output_data_delayed.position.y, _output_data_delayed.position.z); */
        if (_bad_imu_data) {
            _bad_imu_vel_err_integral += (_s.state_struct.velocity.z - _output_data_new.velocity.z);
        } else {
            _bad_imu_vel_err_integral = _vel_err_integral.z;
        }
        _output_track_error.x = v3f_length(&delta_ang_err);
        _output_track_error.y = v3f_length(&vel_err);
        _output_track_error.z = v3f_length(&pos_err);
        float tau_pos_vel = constrain_float(0.01f * (float)_tau_vel_pos_output, 0.1, 0.5);
        float vel_pos_gain = _dt_ekf_avg / constrain_float(tau_pos_vel, _dt_ekf_avg, 10);
        _pos_err_integral = v3f_add(&_pos_err_integral, &pos_err);
        _vel_err_integral = v3f_add(&_vel_err_integral, &vel_err);
        vector3f_t vt1 = v3f_uniform_scale(&pos_err, vel_pos_gain);
        vector3f_t vt2 = v3f_uniform_scale(&_pos_err_integral, sq(vel_pos_gain) * 0.1);
        vector3f_t pos_correction = v3f_add(&vt1, &vt2);
        vector3f_t vel_correction;
        vel_correction.x = vel_err.x * vel_pos_gain + _vel_err_integral.x * sq(vel_pos_gain) * 0.1f;
        vel_correction.y = vel_err.y * vel_pos_gain + _vel_err_integral.y * sq(vel_pos_gain) * 0.1f;
        if (_bad_imu_data) {
            vel_correction.z = vel_err.z * vel_pos_gain + _bad_imu_vel_err_integral *
                sq(vel_pos_gain) * 0.07f;
            _vel_err_integral.z = _bad_imu_vel_err_integral;
        } else {
            vel_correction.z = vel_err.z * vel_pos_gain + _vel_err_integral.z * sq(vel_pos_gain) *
                0.1f;
        }
        output_elements_t output_states;
        for (uint8_t index = 0; index < _imu_buffer_length; index++) {
            output_states = *((output_elements_t *)ekf_imu_buffer_get(&_stored_output, index));
            output_states.velocity = v3f_add(&output_states.velocity, &vel_correction);
            output_states.position = v3f_add(&output_states.position, &pos_correction);
            *((output_elements_t *)ekf_imu_buffer_get(&_stored_output, index)) = output_states;
        }
        uint8_t y2 = ekf_imu_buffer_get_youngest_index(&_stored_imu);
        //MY_LOG("y2 %d\n", y2);
        _output_data_new = *((output_elements_t *)ekf_imu_buffer_get(&_stored_output, y2));
        /* MY_LOG("output_data_new: quat: %f %f %f %f, vleocity: %f %f %f, position: %f %f %f\n", */
        /*        _output_data_new.quat.q1, _output_data_new.quat.q2, _output_data_new.quat.q3, */
        /*        _output_data_new.quat.q4, _output_data_new.velocity.x, _output_data_new.velocity.y, */
        /*        _output_data_new.velocity.z, _output_data_new.position.x, _output_data_new.position.y, */
        /*        _output_data_new.position.z); */
    }
    if (xtimer_now().ticks32 / 1000 - last_output_time_ms > 1000) {
        float roll, pitch, yaw;
        quat_to_euler(&_s.state_struct.quat, &roll, &pitch, &yaw);
        MY_LOG("ekf roll %f pitch %f yaw %f, vel %f %f %f pos %f %f %f, index: %d\n",
               degrees(roll), degrees(pitch), degrees(yaw),
               _s.state_struct.velocity.x, _s.state_struct.velocity.y, _s.state_struct.velocity.z,
               _s.state_struct.position.x, _s.state_struct.position.y, _s.state_struct.position.z,
               _state_index_lim);
        quat_to_euler(&_output_data_new.quat, &roll, &pitch, &yaw);
        MY_LOG("output roll %f pitch %f yaw %f, vel %f %f %f pos %f %f %f\n",
               degrees(roll), degrees(pitch), degrees(yaw),
               _output_data_new.velocity.x, _output_data_new.velocity.y, _output_data_new.velocity.z,
               _output_data_new.position.x, _output_data_new.position.y, _output_data_new.position.z);
        last_output_time_ms = xtimer_now().ticks32 / 1000;
    }
}

void ekf3_core_update_filter(bool predict)
{
    _start_predict_enabled = predict;
    if (!_states_initialised) {
        MY_LOG("states init = false\n");
        return;
    }
    ekf3_core_control_filter_modes();
    ekf3_core_read_imu_data();
    if (_run_updates) {
        update_strap_down_equations_ned();
        ekf3_core_covariance_prediction(NULL);
        //run_yaw_estimator_prediction(); //gsf algorithm not used
        ekf3_core_select_mag_fusion();
        ekf3_core_select_vel_pos_fusion();
        //select_beta_drag_fusion;
        ekf3_core_update_filter_status();
        if (_imu_sample_time_ms - _last_one_hz_ms >= 1000) {
            _last_one_hz_ms = _imu_sample_time_ms;
            move_ekf_origin();
            ekf3_core_check_update_earth_field();
        }
    }
    calc_output_states();
    if (_filter_status.value != 0) {
        _last_filter_ok_ms = dal_millis();
    }
    if (_filter_status.value == 0 &&
        _last_filter_ok_ms != 0 &&
        dal_millis() - _last_filter_ok_ms > 5000
    ) {
        MY_LOG("EKF3 IMU0 forced reset\n");
        _last_filter_ok_ms = 0;
        _states_initialised = false;
        init_ekf3_core_filter_bootstrap();
    }
}

void ekf3_core_best_rotation_order(rotation_order_t *order)
{
    if (fabsf(_prev_tnb.c.x) < fabsf(_prev_tnb.c.y)) {
        *order = TAIT_BRYAN_321;
    } else {
        *order = TAIT_BRYAN_312;
    }
}

void ekf3_core_set_yaw_from_mag(void)
{
    if (!ekf3_core_use_compass()) {
        return;
    }
    ekf3_core_read_mag_data();
    rotation_order_t order;
    ekf3_core_best_rotation_order(&order);
    vector3f_t euler_angles;
    matrix3f_t tbn_zero_yaw;
    if (order == TAIT_BRYAN_321) {
        quat_to_euler(&_s.state_struct.quat, &euler_angles.x, &euler_angles.y, &euler_angles.z);
        m3f_from_euler(&tbn_zero_yaw, euler_angles.x, euler_angles.y, 0.0f);
    } else if (order == TAIT_BRYAN_312) {
        euler_angles = quat_to_vector312(&_s.state_struct.quat);
        m3f_from_euler312(&tbn_zero_yaw, euler_angles.x, euler_angles.y, 0.0f);
    } else {
        return;
    }
    vector3f_t mag_meas_ned = m3f_multi_v(&tbn_zero_yaw, &_mag_data_delayed.mag);
    /* MY_LOG("s.quat %f %f %f %f, declination %f\n", _s.state_struct.quat.q1, */
    /*        _s.state_struct.quat.q2, _s.state_struct.quat.q3, _s.state_struct.quat.q4, */
    /*        degrees(ekf3_core_mag_declination())); */
    float yaw_ang_measured = wrap_PI(-atan2f(mag_meas_ned.y, mag_meas_ned.x) + ekf3_core_mag_declination());
    ekf3_core_reset_quat_state_yaw_only(yaw_ang_measured, sq(MAX(_yaw_noise, 1.0e-2f)), order);
}

void ekf3_core_store_quat_rotate(const quaternionf_t *delta_quat)
{
    _output_data_new.quat = quat_multi(&_output_data_new.quat, delta_quat);
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->quat =
            quat_multi(&((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->quat,
                       delta_quat);
    }
    _output_data_delayed.quat = quat_multi(&_output_data_delayed.quat, delta_quat);
}

void ekf3_core_reset_mag_field_states(void)
{
    quaternionf_t tmp = quat_inverse(&_s.state_struct.quat);
    quat_to_rotation_matrix(&tmp, &_prev_tnb);
    if (_have_table_earth_field && _mag_ef_limit > 0) {
        _s.state_struct.earth_magfield = _table_earth_field_ga;
    } else {
        matrix3f_t mt = m3f_transposed(&_prev_tnb);
        _s.state_struct.earth_magfield = m3f_multi_v(&mt, &_mag_data_delayed.mag);
    }
    ekf3_core_align_mag_state_declination();
    ekf3_core_zero_rows(&_P, 18, 21);
    ekf3_core_zero_cols(&_P, 18, 21);
    _P[18][18] = _mag_noise;
    _P[19][19] = _P[18][18];
    _P[20][20] = _P[18][18];
    _P[21][21] = _P[18][18];
    ekf3_core_record_mag_reset();
}

void ekf3_core_get_euler_angles(vector3f_t *euler)
{
    quat_to_euler(&_output_data_new.quat, &euler->x, &euler->y, &euler->z);
}

void ekf3_core_get_vel_ned(vector3f_t *vel)
{
    *vel = v3f_add(&_output_data_new.velocity, &_vel_offset_ned);
    //MY_LOG("vel offset %f %f %f\n", _vel_offset_ned.x, _vel_offset_ned.y, _vel_offset_ned.z);
}
