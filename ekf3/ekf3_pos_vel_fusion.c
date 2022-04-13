#include "ekf3_core.h"
#include "fusion_math.h"
#include "ekf_buffer.h"
#include "uart_device.h"
#include "xtimer.h"
//#include "ekf_source.h"

#define M_SQRT1_2 0.707106781186

extern float _gps_noise_scaler;
extern Vector28 _Kfusion;
extern Matrix24 _KHP;
extern uint8_t _state_index_lim;
extern matrix3f_t _prev_tnb;
extern bool _inhibit_del_ang_bias_states;
extern bool _inhibit_del_vel_bias_states;
extern fault_status_t _fault_status;
extern bool _inhibit_mag_states;
extern bool _inhibit_wind_states;
extern bool _dvel_bias_axis_inhibit[3];
extern const uint16_t _hgt_retry_time_mode0_ms;
extern float _baro_gnd_effect_dead_zone;
extern int16_t _gps_vel_innov_gate;
extern const uint16_t _hgt_retry_time_mode12_ms;
extern float _hgt_innov_filt_state;
extern float _pos_test_ratio;
extern int16_t _hgt_innov_gate;
extern uint32_t _good_imu_data_ms;
extern float _hgt_test_ratio;
extern const float _gps_pos_var_acc_scale;
extern bool _tilt_align_complete;
extern bool _prev_motors_armed;
extern bool _motors_armed;
extern uint32_t _last_launch_accel_time_ms;
extern float _noaid_horiz_noise;
extern Vector6 _innov_vel_pos;
extern Vector6 _var_innov_vel_pos;
extern int16_t _gps_pos_innov_gate;
extern int8_t _gps_glitch_radius_max;

extern bool _yaw_align_complete;
extern uint16_t _hgt_retry_time_ms;
extern float _hgt_mea;
extern bool _use_gps_vert_vel;
extern float _gps_hgt_accuracy;
extern source_z_t _prev_hgt_source;
extern source_z_t _active_hgt_source;
extern bool _valid_origin;
extern bool _gps_accuracy_good;
extern vector2f_t _pos_reset_ne;
extern vector2f_t _vel_reset_ne;
extern uint32_t _last_pos_reset_ms;
extern uint32_t _last_vel_reset_ms;
extern state_var_t _s;
extern Matrix24 _P;
extern aiding_mode_t _pv_aiding_mode;
extern uint32_t _imu_sample_time_ms;
extern uint32_t _last_time_gps_received_ms;
extern float _gps_horiz_vel_noise;
extern gps_elements_t _gps_data_new;
extern float _gps_spd_accuracy;
extern bool _vel_timeout;
extern uint32_t _last_vel_pass_time_ms;
extern ekf_imu_buffer_t _stored_output;
extern output_elements_t _output_data_new;
extern output_elements_t _output_data_delayed;
extern uint8_t _imu_buffer_length;
extern vector2f_t _last_known_position_ne;
extern float _gps_horiz_pos_noise;
extern uint8_t _last_gps_idx;
extern location_t _ekf_origin;
extern imu_elements_t _imu_data_delayed;
extern float _gps_pos_accuracy;
extern bool _pos_timeout;
extern uint32_t _last_pos_pass_time_ms;
extern float _pos_reset_d;
extern float _hgt_mea;
extern bool _on_ground;
extern float _terrain_state;
extern float _rng_on_gnd;
extern vert_vel_t _vert_comp_filt_state;
extern uint32_t _last_pos_reset_d_ms;
extern bool _hgt_timeout;
extern uint32_t _last_hgt_pass_time_ms;
extern float _pos_down_obs_noise;
extern bool _in_flight;
extern bool _gps_in_use;
extern bool _bad_imu_data;
extern gps_elements_t _gps_data_delayed;
extern float _gps_vert_vel_noise;
extern uint32_t _vert_vel_var_clip_counter;
extern bool _mag_fuse_performed;
extern bool _pos_vel_fusion_delayed;
extern float _dt_imu_avg;
extern bool _gps_data_to_fuse;
extern ekf_ring_buffer_t _stored_gps;
extern float _acc_nav_mag;
extern const float _gps_ne_vel_var_acc_scale;
extern const float _gps_d_vel_var_acc_scale;
extern vector3f_t _gps_vel_innov;
extern vector3f_t _gps_vel_var_innov;
extern uint32_t _gps_vel_innov_time_ms;
extern source_xy_t _posxy_source_last;
extern bool _posxy_source_reset;
extern bool _fuse_vel_data;
extern bool _fuse_pos_data;
extern bool _fuse_hgt_data;
extern Vector6 _vel_pos_obs;
extern bool _gps_yaw_reset_request;
extern float _vel_test_ratio;

void ekf3_core_reset_velocity(reset_data_source_t vel_reset_source)
{
    //MY_LOG("reset velocity");
    _vel_reset_ne.x = _s.state_struct.velocity.x;
    _vel_reset_ne.y = _s.state_struct.velocity.y;
    ekf3_core_zero_rows(&_P, 4, 5);
    ekf3_core_zero_cols(&_P, 4, 5);
    if (_pv_aiding_mode != AID_ABSOLUTE) {
        v3f_zero(&_s.state_struct.velocity);
        _P[5][5] = _P[4][4] = sq(_gps_horiz_vel_noise);
    } else {
        if ((_imu_sample_time_ms - _last_time_gps_received_ms < 250 && vel_reset_source == RESET_DATA_SOURCE_DEFAULT) ||
                                                        vel_reset_source == RESET_DATA_SOURCE_GPS) {
            //MY_LOG("_s velocity %f %f\n", _gps_data_new.vel.x, _gps_data_new.vel.y);
            gps_elements_t gps_corrected = _gps_data_new;
            //correct_gps_for_antenna_offset();
            _s.state_struct.velocity.x = gps_corrected.vel.x;
            _s.state_struct.velocity.y = gps_corrected.vel.y;
            _P[5][5] = _P[4][4] = sq(MAX(_gps_horiz_vel_noise, _gps_spd_accuracy));
        } else {
            _s.state_struct.velocity.x = 0;
            _s.state_struct.velocity.y = 0;
            _P[5][5] = _P[4][4] = sq(25.0f);
        }
        _vel_timeout = false;
        _last_vel_pass_time_ms = _imu_sample_time_ms;
    }
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->velocity.x =
            _s.state_struct.velocity.x;
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->velocity.y =
            _s.state_struct.velocity.y;
    }
    _output_data_new.velocity.x = _s.state_struct.velocity.x;
    _output_data_new.velocity.y = _s.state_struct.velocity.y;
    _output_data_delayed.velocity.x = _s.state_struct.velocity.x;
    _output_data_delayed.velocity.y = _s.state_struct.velocity.y;

    _vel_reset_ne.x = _s.state_struct.velocity.x - _vel_reset_ne.x;
    _vel_reset_ne.y = _s.state_struct.velocity.y - _vel_reset_ne.y;
    _last_vel_reset_ms = _imu_sample_time_ms;
}

void ekf3_core_reset_position(reset_data_source_t pos_reset_source)
{
    _pos_reset_ne.x = _s.state_struct.position.x;
    _pos_reset_ne.y = _s.state_struct.position.y;
    ekf3_core_zero_rows(&_P, 7, 8);
    ekf3_core_zero_cols(&_P, 7, 8);
    if (_pv_aiding_mode != AID_ABSOLUTE) {
        MY_LOG("reset position pv aiding mode != absolute\n");
        _s.state_struct.position.x = _last_known_position_ne.x;
        _s.state_struct.position.y = _last_known_position_ne.y;
        _P[7][7] = _P[8][8] = sq(_gps_horiz_pos_noise);
    } else {
        if ((_imu_sample_time_ms - _last_time_gps_received_ms < 250 && pos_reset_source ==
            RESET_DATA_SOURCE_DEFAULT) || pos_reset_source == RESET_DATA_SOURCE_GPS) {
            gps_elements_t gps_corrected = _gps_data_new;
            //correct_gps_for_antenna_offset;
            _last_gps_idx = gps_corrected.sensor_idx;
            location_t gpsloc = set_location(gps_corrected.lat, gps_corrected.lng, 0,
                                             ALT_ABSOLUTE);
            //MY_LOG("gps corrected: %ld %ld, ekf origin %ld %ld\n", gps_corrected.lat,
            //       gps_corrected.lng, _ekf_origin.lat, _ekf_origin.lng);
            vector2f_t tmp = location_get_distance_ne_float(&_ekf_origin, &gpsloc);
            //MY_LOG("tmp %f %f\n", tmp.x, tmp.y);
            _s.state_struct.position.x = tmp.x;
            _s.state_struct.position.y = tmp.y;
            const int32_t tdiff = _imu_data_delayed.time_ms - gps_corrected.time_ms;
            _s.state_struct.position.x = _s.state_struct.position.x +
                gps_corrected.vel.x * 0.001 * tdiff;
            _s.state_struct.position.y = _s.state_struct.position.y +
                gps_corrected.vel.y * 0.001 * tdiff;
            //MY_LOG("reset position %f %f\n", _s.state_struct.position.x, _s.state_struct.position.y);
            _P[7][7] = _P[8][8] = sq(MAX(_gps_pos_accuracy, _gps_horiz_pos_noise));
        } else {
            MY_LOG("imu sample time %ld last time gps %ld\n", _imu_sample_time_ms,
                   _last_time_gps_received_ms);
            MY_LOG("RESET DATA SOURCE RNGBCN not support \n");
        }
    }
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->position.x =
            _s.state_struct.position.x;
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->position.y =
            _s.state_struct.position.y;
    }
    _output_data_new.position.x = _s.state_struct.position.x;
    _output_data_new.position.y = _s.state_struct.position.y;
    _output_data_delayed.position.x = _s.state_struct.position.x;
    _output_data_delayed.position.y = _s.state_struct.position.y;
    _pos_reset_ne.x = _s.state_struct.position.x - _pos_reset_ne.x;
    _pos_reset_ne.y = _s.state_struct.position.y - _pos_reset_ne.y;
    _last_pos_reset_ms = _imu_sample_time_ms;
    _pos_timeout = false;
    _last_pos_pass_time_ms = _imu_sample_time_ms;
}

static void reset_position_ne(float pos_n, float pos_e)
{
    const vector3f_t pos_orig = _s.state_struct.position;
    _s.state_struct.position.x = pos_n;
    _s.state_struct.position.y = pos_e;
    _pos_reset_ne.x = _s.state_struct.position.x - pos_orig.x;
    _pos_reset_ne.y = _s.state_struct.position.y - pos_orig.y;
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->position.x += _pos_reset_ne.x;
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->position.y += _pos_reset_ne.y;
    }
    _output_data_new.position.x += _pos_reset_ne.x;
    _output_data_new.position.y += _pos_reset_ne.y;
    _output_data_delayed.position.x += _pos_reset_ne.x;
    _output_data_delayed.position.y += _pos_reset_ne.y;
    _last_pos_reset_ms = _imu_sample_time_ms;
}

static void reset_position_d(float pos_d)
{
    const float pos_d_orig = _s.state_struct.position.z;
    _s.state_struct.position.z = pos_d;
    _pos_reset_d = _s.state_struct.position.z - pos_d_orig;
    _output_data_new.position.z += _pos_reset_d;
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->position.z += _pos_reset_d;
    }
    _last_pos_reset_d_ms = _imu_sample_time_ms;
}

void ekf3_core_reset_height(void)
{
    _pos_reset_d = _s.state_struct.position.z;
    _s.state_struct.position.z = -_hgt_mea;
    _output_data_new.position.z = _s.state_struct.position.z;
    _output_data_delayed.position.z = _s.state_struct.position.z;
    if (_on_ground) {
       _terrain_state = _s.state_struct.position.z + _rng_on_gnd;
    } else {
       _terrain_state = MAX(_s.state_struct.position.z + _rng_on_gnd, _terrain_state);
    }
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->position.z =
            _s.state_struct.position.z;
    }
    _vert_comp_filt_state.pos = _s.state_struct.position.z;
    _pos_reset_d = _s.state_struct.position.z - _pos_reset_d;
    _last_pos_reset_d_ms = _imu_sample_time_ms;
    _hgt_timeout = false;
    _last_hgt_pass_time_ms = _imu_sample_time_ms;
    ekf3_core_zero_rows(&_P, 9, 9);
    ekf3_core_zero_cols(&_P, 9, 9);
    _P[9][9] = _pos_down_obs_noise;
    if (_in_flight &&
        (_gps_in_use || _bad_imu_data) &&
        (ekf_use_vel_z_source(Z_GPS)) &&
        _gps_data_new.have_vz &&
        (_imu_sample_time_ms - _gps_data_delayed.time_ms < 500)) {
        _s.state_struct.velocity.z = _gps_data_new.vel.z;
    } else if (_on_ground) {
        _s.state_struct.velocity.z = 0;
    }
    for (uint8_t i = 0; i < _imu_buffer_length; i++) {
        ((output_elements_t *)ekf_imu_buffer_get(&_stored_output, i))->velocity.z =
            _s.state_struct.velocity.z;
    }
    _output_data_new.velocity.z = _s.state_struct.velocity.z;
    _output_data_delayed.velocity.z = _s.state_struct.velocity.z;
    _vert_comp_filt_state.vel = _output_data_new.velocity.z;
    ekf3_core_zero_rows(&_P, 6, 6);
    ekf3_core_zero_cols(&_P, 6, 6);
    _P[6][6] = sq(_gps_vert_vel_noise);
    _vert_vel_var_clip_counter = 0;
}

static void calculate_vel_innovations_and_variances(const vector3f_t *velocity,
                                                    float noise, float accel_scale,
                                                    vector3f_t *innovations, vector3f_t *variances)
{
    //MY_LOG("gps vel %f %f %f\n", velocity->x, velocity->y, velocity->z);
    *innovations = v3f_sub(&_s.state_struct.velocity, velocity);
    const float obs_data_chk = sq(constrain_float(noise, 0.05, 5.0)) + sq(accel_scale *
    _acc_nav_mag);
    /* MY_LOG("innov %f %f %f, obs_data_chk %f\n", innovations->x, innovations->y, innovations->z, */
    /*        obs_data_chk); */
    variances->x = _P[4][4] + obs_data_chk;
    variances->y = _P[5][5] + obs_data_chk;
    variances->z = _P[6][6] + obs_data_chk;
}

static void select_height_for_fusion(void)
{
    /* if (ekf_source_get_posz_source() == Z_GPS && ((_imu_sample_time_ms - _last_time_gps_received_ms) < 500) && */
    /* _valid_origin && _gps_accuracy_good) { */
    /*     _active_hgt_source = Z_GPS; */
    /* } */
    if (_gps_data_to_fuse && (_active_hgt_source == Z_GPS)) {
        //MY_LOG("hgt to fuse\n");
        _hgt_mea = _gps_data_delayed.hgt;
        _vel_pos_obs[5] = -_hgt_mea;
        //MY_LOG("pos z: %f\n", _vel_pos_obs[5]);
        _fuse_hgt_data = true;
        if (_gps_hgt_accuracy > 0) {
            _pos_down_obs_noise = sq(constrain_float(_gps_hgt_accuracy, 1.5 * _gps_horiz_pos_noise, 100));
        } else {
            _pos_down_obs_noise = sq(constrain_float(1.5 * _gps_horiz_pos_noise, 0.1, 10));
        }

    } else {
        _fuse_hgt_data = false;
    }
    if ((_active_hgt_source != _prev_hgt_source) && _fuse_hgt_data) {
        _prev_hgt_source = _active_hgt_source;
        MY_LOG("select hgt for fuse reset hgt source\n");
        reset_position_d(-_hgt_mea);
    }
    _hgt_retry_time_ms = ((_use_gps_vert_vel) && !_vel_timeout) ? _hgt_retry_time_mode0_ms :
        _hgt_retry_time_mode12_ms;
    if (_imu_sample_time_ms - _last_hgt_pass_time_ms > _hgt_retry_time_ms ||
    (_bad_imu_data && (_imu_sample_time_ms - _good_imu_data_ms > BAD_IMU_DATA_TIMEOUT_MS))) {
        //MY_LOG("select hgt hgt timeout\n");
        _hgt_timeout = true;
    } else {
        _hgt_timeout = false;
    }
}

static void fuse_vel_pos_ned(void)
{
    bool vel_check_passed = false;
    bool pos_check_passed = false;
    bool hgt_check_passed = false;
    bool fuse_data[6] = {0};
    uint8_t state_index;
    uint8_t obs_index;
    Vector6 R_OBS;
    Vector6 R_OBS_DATA_CHECKS;
    float SK;
    if (_fuse_vel_data || _fuse_pos_data || _fuse_hgt_data) {
        float pos_err = _gps_pos_var_acc_scale * _acc_nav_mag;
        if (_pv_aiding_mode == AID_NONE) {
            if (_tilt_align_complete && _motors_armed) {
                R_OBS[0] = sq(constrain_float(_noaid_horiz_noise, 0.5f, 50));
            } else {
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i = 0; i <= 2; i++) {
                R_OBS_DATA_CHECKS[i] = R_OBS[i];
            }
        } else {
            if (_gps_spd_accuracy > 0.0f) {
                R_OBS[0] = sq(constrain_float(_gps_spd_accuracy, _gps_horiz_vel_noise, 50));
                R_OBS[2] = sq(constrain_float(_gps_spd_accuracy, _gps_vert_vel_noise, 50));
            } else {
                R_OBS[0] = sq(constrain_float(_gps_horiz_vel_noise, 0.05, 5.0)) +
                    sq(_gps_ne_vel_var_acc_scale * _acc_nav_mag);
                //MY_LOG("R_OBS[0] = %f, accnavmag %f\n", R_OBS[0], _acc_nav_mag);
                R_OBS[2] = sq(constrain_float(_gps_vert_vel_noise, 0.05, 5.0)) +
                    sq(_gps_d_vel_var_acc_scale * _acc_nav_mag);
            }
            R_OBS[1] = R_OBS[0];
            if (_gps_pos_accuracy > 0.0f) {
                R_OBS[3] = sq(constrain_float(_gps_pos_accuracy, _gps_horiz_pos_noise, 100));
            } else {
                R_OBS[3] = sq(constrain_float(_gps_horiz_pos_noise, 0.1, 10)) + sq(pos_err);
            }
            R_OBS[4] = R_OBS[3];
            float obs_data_chk;
            obs_data_chk = sq(constrain_float(_gps_horiz_vel_noise, 0.05, 5.0)) +
                sq(_gps_ne_vel_var_acc_scale * _acc_nav_mag);
            R_OBS_DATA_CHECKS[0] = R_OBS_DATA_CHECKS[1] = R_OBS_DATA_CHECKS[2] = obs_data_chk;
        }
        R_OBS[5] = _pos_down_obs_noise;
        for (uint8_t i = 3; i <= 5; i++) {
            R_OBS_DATA_CHECKS[i] = R_OBS[i];
        }
        if (_fuse_pos_data) {
            _innov_vel_pos[3] = _s.state_struct.position.x - _vel_pos_obs[3];
            _innov_vel_pos[4] = _s.state_struct.position.y - _vel_pos_obs[4];
            _var_innov_vel_pos[3] = _P[7][7] + R_OBS_DATA_CHECKS[3];
            _var_innov_vel_pos[4] = _P[8][8] + R_OBS_DATA_CHECKS[4];
            float max_pos_innov2 = sq(MAX(0.01 * (float)_gps_pos_innov_gate, 1.0)) *
                (_var_innov_vel_pos[3] + _var_innov_vel_pos[4]);
            _pos_test_ratio = (sq(_innov_vel_pos[3]) + sq(_innov_vel_pos[4])) / max_pos_innov2;
            if (_pos_test_ratio < 1.0f || (_pv_aiding_mode == AID_NONE)) {
                pos_check_passed = true;
                _last_pos_pass_time_ms = _imu_sample_time_ms;
            }
            if (pos_check_passed || _pos_timeout || _bad_imu_data) {
                if (_pos_timeout || ((_P[8][8] + _P[7][7]) > sq((float)(_gps_glitch_radius_max)))) {
                    MY_LOG("pos timeout or _P[7][7] + _P[8][8] = %f\n", _P[7][7] + _P[8][8]);
                    ekf3_core_reset_position(RESET_DATA_SOURCE_DEFAULT);
                    _fuse_pos_data = false;
                    ekf3_core_zero_rows(&_P, 7, 8);
                    ekf3_core_zero_cols(&_P, 7, 8);
                    _P[7][7] = sq((float)(0.5f * _gps_glitch_radius_max));
                    _P[8][8] = _P[7][7];
                    _pos_test_ratio = 0;
                    if (_vel_timeout) {
                        MY_LOG("fuse pos vel timeout\n");
                        ekf3_core_reset_velocity(RESET_DATA_SOURCE_DEFAULT);
                        _fuse_vel_data = false;
                        _vel_test_ratio = 0;
                    }
                }
            } else {
                _fuse_pos_data = false;
            }
        }
        if (_fuse_vel_data) {
            uint8_t imax = 1;
            float innov_vel_sum_sq = 0;
            float var_vel_sum = 0;
            for (uint8_t i = 0; i <= imax; i++) {
                state_index = i + 4;
                const float innovation = ((float *)&_s.state_struct.velocity)[i] - _vel_pos_obs[i];
                //MY_LOG("vel %d innovation %f ", i, innovation);
                innov_vel_sum_sq += sq(innovation);
                _var_innov_vel_pos[i] = _P[state_index][state_index] + R_OBS_DATA_CHECKS[i];
                var_vel_sum += _var_innov_vel_pos[i];
            }
            //MY_LOG("_P[4][4] = %f P[5][5] = %f, ROBSCHE = %f %f\n", _P[4][4], _P[5][5],
            //       R_OBS_DATA_CHECKS[0], R_OBS_DATA_CHECKS[1]);
            _vel_test_ratio = innov_vel_sum_sq / (var_vel_sum * sq(MAX(0.01 * _gps_vel_innov_gate, 1)));
            //MY_LOG("vel test ratio = %f\n", _vel_test_ratio);
            if (_vel_test_ratio < 1.0) {
                vel_check_passed = true;
                _last_vel_pass_time_ms = _imu_sample_time_ms;
            }
            if (vel_check_passed || _vel_timeout || _bad_imu_data) {
                if (_pv_aiding_mode == AID_ABSOLUTE && _vel_timeout) {
                    MY_LOG("fuse vel vel timeout\n");
                    ekf3_core_reset_velocity(RESET_DATA_SOURCE_DEFAULT);
                    _fuse_vel_data = false;
                    _vel_test_ratio = 0;
                }
            } else {
                _fuse_vel_data = false;
            }
        }
        if (_fuse_hgt_data) {
            _innov_vel_pos[5] = _s.state_struct.position.z - _vel_pos_obs[5];
            _var_innov_vel_pos[5] = _P[9][9] + R_OBS_DATA_CHECKS[5];
            _hgt_test_ratio = sq(_innov_vel_pos[5]) / (sq(MAX(0.01 * (float)_hgt_innov_gate, 1)) *
                                                       _var_innov_vel_pos[5]);
            const float max_test_ratio = (_pv_aiding_mode == AID_NONE && _on_ground) ? 3 : 1;
            if (_hgt_test_ratio < max_test_ratio) {
                hgt_check_passed = true;
                _last_hgt_pass_time_ms = _imu_sample_time_ms;
            }
            if (hgt_check_passed || _hgt_timeout || _bad_imu_data) {
                if (_on_ground) {
                    float dt_baro = (_imu_sample_time_ms - _last_hgt_pass_time_ms) * 1.0e-3;
                    const float hgt_innov_filt_tc = 2.0;
                    float alpha = constrain_float(dt_baro / (dt_baro + hgt_innov_filt_tc), 0, 1);
                    _hgt_innov_filt_state += (_innov_vel_pos[5] - _hgt_innov_filt_state) * alpha;
                } else {
                    _hgt_innov_filt_state = 0;
                }
                if (_hgt_timeout) {
                    ekf3_core_reset_height();
                    _fuse_hgt_data = false;
                }
            } else {
                _fuse_hgt_data = false;
            }
        }
        if (_fuse_vel_data) {
            //MY_LOG("fuse vel data, acc %f %f gps %f %f\n", _s.state_struct.velocity.x,
            //       _s.state_struct.velocity.y,
            //       _vel_pos_obs[0], _vel_pos_obs[1]);
            fuse_data[0] = true;
            fuse_data[1] = true;
            if (_use_gps_vert_vel) {
                fuse_data[2] = true;
            }
        }
        if (_fuse_pos_data) {
            //MY_LOG("fuse pos data\n");
            fuse_data[3] = true;
            fuse_data[4] = true;
        }
        if (_fuse_hgt_data) {
            //MY_LOG("fuse hgt data\n");
            fuse_data[5] = true;
        }
        for (obs_index = 0; obs_index <= 5; obs_index++) {
            if (fuse_data[obs_index]) {
                state_index = 4 + obs_index;
                if (obs_index <= 2) {
                    _innov_vel_pos[obs_index] = ((float *)(&_s.state_struct.velocity))[obs_index] -
                        _vel_pos_obs[obs_index];
                    R_OBS[obs_index] *= sq(_gps_noise_scaler);
                } else if (obs_index == 3 || obs_index == 4) {
                    _innov_vel_pos[obs_index] = ((float *)(&_s.state_struct.position))[obs_index - 3] -
                        _vel_pos_obs[obs_index];
                    R_OBS[obs_index] *= sq(_gps_noise_scaler);
                } else if (obs_index == 5) {
                    _innov_vel_pos[obs_index] = _s.state_struct.position.z - _vel_pos_obs[obs_index];
                }
                _var_innov_vel_pos[obs_index] = _P[state_index][state_index] + R_OBS[obs_index];
                //MY_LOG("obs idx %d, R_OBS: %f\n", obs_index, R_OBS[obs_index]);
                SK = 1.0f / _var_innov_vel_pos[obs_index];
                for (uint8_t i = 0; i <= 9; i++) {
                    _Kfusion[i] = _P[i][state_index] * SK;
                }
                if (!_inhibit_del_ang_bias_states) {
                    for (uint8_t i = 10; i <= 12; i++) {
                        bool poor_observability = false;
                        if (_pv_aiding_mode == AID_NONE) {
                            const uint8_t axis_index = i - 10;
                            if (axis_index == 0) {
                                poor_observability = fabsf(_prev_tnb.a.z) > M_SQRT1_2;
                            } else if (axis_index == 1) {
                                poor_observability = fabsf(_prev_tnb.b.z) > M_SQRT1_2;
                            } else {
                                poor_observability = fabsf(_prev_tnb.c.z) > M_SQRT1_2;
                            }
                        }
                        if (poor_observability) {
                            _Kfusion[i] = 0;
                        } else {
                            _Kfusion[i] = _P[i][state_index] * SK;
                        }
                    }
                } else {
                    //MY_LOG("inhibit del ang bias\n");
                    ekf3_core_zero_range(&_Kfusion[0], 10, 12);
                }
                const bool horiz_inhibit = _pv_aiding_mode == AID_NONE && obs_index != 2
                    && obs_index != 5;
                if (!horiz_inhibit && !_inhibit_del_vel_bias_states && !_bad_imu_data) {
                    for (uint8_t i = 13; i <= 15; i++) {
                        if (!_dvel_bias_axis_inhibit[i - 13]) {
                            _Kfusion[i] = _P[i][state_index] * SK;
                        } else {
                            //MY_LOG("inhibit del vel %d, obs idx %d\n", i-13, obs_index);
                            _Kfusion[i] = 0;
                        }
                    }
                } else {
                    //MY_LOG("inhibit del vel bias\n");
                    ekf3_core_zero_range(&_Kfusion[0], 13, 15);
                }
                if (!_inhibit_mag_states) {
                    for (uint8_t i = 16; i <= 21; i++) {
                        _Kfusion[i] = _P[i][state_index] * SK;
                    }
                } else {
                    //MY_LOG("inhibit mag states\n");
                    ekf3_core_zero_range(&_Kfusion[0], 16, 21);
                }
                if (!_inhibit_wind_states) {
                    _Kfusion[22] = _P[22][state_index] * SK;
                    _Kfusion[23] = _P[23][state_index] * SK;
                } else {
                    //MY_LOG("inhibit wind \n");
                    ekf3_core_zero_range(&_Kfusion[0], 22, 23);
                }
                for (uint8_t i = 0; i <= _state_index_lim; i++) {
                    for (uint8_t j = 0; j <= _state_index_lim; j++) {
                        _KHP[i][j] = _Kfusion[i] * _P[state_index][j];
                    }
                }
                bool healthy_fusion = true;
                for (uint8_t i = 0; i <= _state_index_lim; i++) {
                    if (_KHP[i][i] > _P[i][i]) {
                        healthy_fusion = false;
                        MY_LOG("unhealthy index %d, obs index %d, KHP %f P %f\n", i, obs_index,
                               _KHP[i][i], _P[i][i]);
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
                        _s.states_array[i] = _s.states_array[i] - _Kfusion[i] * _innov_vel_pos[obs_index];
                        /* if (i == 4 || i==5 || i==6) { */
                        /*     MY_LOG("i %d k %f innov%f index %d", i, _Kfusion[i], */
                        /*            _innov_vel_pos[obs_index], obs_index); */
                        /* } */
                    }
                    quat_normalize(&_s.state_struct.quat);
                    if (obs_index == 0) {
                        _fault_status.bad_nvel = false;
                    } else if (obs_index == 1) {
                        _fault_status.bad_evel = false;
                    } else if (obs_index == 2) {
                        _fault_status.bad_dvel = false;
                    } else if (obs_index == 3) {
                        _fault_status.bad_npos = false;
                    } else if (obs_index == 4) {
                        _fault_status.bad_epos = false;
                    } else if (obs_index == 5) {
                        _fault_status.bad_dpos = false;
                    }
                } else {
                    //MY_LOG("pos vel health = false, idx %d\n", obs_index);
                    if (obs_index == 0) {
                        _fault_status.bad_nvel = true;
                    } else if (obs_index == 1) {
                        _fault_status.bad_evel = true;
                    } else if (obs_index == 2) {
                        _fault_status.bad_dvel = true;
                    } else if (obs_index == 3) {
                        _fault_status.bad_npos = true;
                    } else if (obs_index == 4) {
                        _fault_status.bad_epos = true;
                    } else if (obs_index == 5) {
                        _fault_status.bad_dpos = true;
                    }
                }
            }
        }
    }
}

void ekf3_core_select_vel_pos_fusion(void)
{
    /* if (_mag_fuse_performed && _dt_imu_avg < 0.005f && ! _pos_vel_fusion_delayed) { */
    /*     _pos_vel_fusion_delayed = true; */
    /*     return; */
    /* } else { */
    /*     //MY_LOG("1\n"); */
    /*     _pos_vel_fusion_delayed = false; */
    /* } */
    ekf3_core_read_gps_data();
    //read_gps_yaw_data();
    _gps_data_to_fuse = ekf_ring_buffer_recall(&_stored_gps, &_gps_data_delayed, _imu_data_delayed.time_ms);
    //MY_LOG("_stored_gps.head = %d tail = %d\n", _stored_gps._head, _stored_gps._tail);
    if (_gps_data_to_fuse) {
        /* MY_LOG("now %ld imu_delay %ld gps_delay %ld\n", xtimer_now().ticks32 / 1000, */
        /*        _imu_data_delayed.time_ms, _gps_data_delayed.time_ms); */
        /* MY_LOG("gps data delayed vel %f %f %f\n", _gps_data_delayed.vel.x, */
        /*        _gps_data_delayed.vel.y, _gps_data_delayed.vel.z); */
        calculate_vel_innovations_and_variances(&_gps_data_delayed.vel, _gps_horiz_vel_noise,
                                                _gps_ne_vel_var_acc_scale, &_gps_vel_innov, &_gps_vel_var_innov);
        _gps_vel_innov_time_ms = xtimer_now().ticks32 / 1000;
    }
    const source_xy_t posxy_source = ekf_source_get_posxy_source();
    if (posxy_source != _posxy_source_last) {
        MY_LOG("posxy source change\n");
        _posxy_source_reset = (posxy_source != XY_NONE);
        _posxy_source_last = posxy_source;
    }
    _fuse_pos_data = false;
    _fuse_vel_data = false;
    //MY_LOG("pv_aidingmode %d posxy_source %d\n", _pv_aiding_mode, posxy_source);
    if (_gps_data_to_fuse && (_pv_aiding_mode == AID_ABSOLUTE) && (posxy_source == XY_GPS)) {
        _fuse_vel_data = ekf_source_use_vel_xy_source(XY_GPS);
        _fuse_pos_data = true;
        if (_fuse_vel_data) {
            _vel_pos_obs[0] = _gps_data_delayed.vel.x;
            _vel_pos_obs[1] = _gps_data_delayed.vel.y;
            _vel_pos_obs[2] = _gps_data_delayed.vel.z;
        }
        const location_t gpsloc = set_location(_gps_data_delayed.lat, _gps_data_delayed.lng, 0,
                                               ALT_ABSOLUTE);
        /* MY_LOG("gps delayed %ld %ld, ekf origin %ld %ld\n", _gps_data_delayed.lat, */
        /*        _gps_data_delayed.lng, _ekf_origin.lat, _ekf_origin.lng); */
        const vector2f_t posxy = location_get_distance_ne_float(&_ekf_origin, &gpsloc);
        _vel_pos_obs[3] = posxy.x;
        _vel_pos_obs[4] = posxy.y;
        //MY_LOG("vel pos obs :%f %f\n", _vel_pos_obs[3], _vel_pos_obs[4]);
    }
    if (_gps_yaw_reset_request) {
        MY_LOG("select pos vel: realign yaw gps\n");
        ekf3_core_realign_yaw_gps();
    }
    select_height_for_fusion();
    if (_gps_data_to_fuse && (_pv_aiding_mode == AID_ABSOLUTE) && (posxy_source == XY_GPS) &&
    (_gps_data_delayed.sensor_idx != _last_gps_idx || _posxy_source_reset)) {
        MY_LOG("select vel pos posxy reset\n");
        _posxy_source_reset = false;
        _last_gps_idx = _gps_data_delayed.sensor_idx;
        const location_t gpsloc = set_location(_gps_data_delayed.lat, _gps_data_delayed.lng, 0,
                                               ALT_ABSOLUTE);
        const vector2f_t posxy = location_get_distance_ne_float(&_ekf_origin, &gpsloc);
        reset_position_ne(posxy.x, posxy.y);
        if (_active_hgt_source == Z_GPS) {
            reset_position_d(-_hgt_mea);
        }
    }
    if (_fuse_hgt_data && _pv_aiding_mode == AID_NONE) {
        //MY_LOG("fuse hgt data && aid none\n");
        if (ekf3_core_assume_zero_sideslip() && _tilt_align_complete && _motors_armed) {
            _fuse_pos_data = false;
            _vel_pos_obs[0] = 0;
            _vel_pos_obs[1] = 0;
            _vel_pos_obs[2] = _s.state_struct.velocity.z;
            bool reset_vel_ne = !_prev_motors_armed;
            if (_imu_data_delayed.del_vel.x > 1.1 * GRAVITY_MSS * _imu_data_delayed.del_vel_dt) {
                _last_launch_accel_time_ms = _imu_sample_time_ms;
                _fuse_vel_data = false;
                reset_vel_ne = true;
            } else if (_last_launch_accel_time_ms != 0 && (_imu_sample_time_ms - _last_launch_accel_time_ms) < 10000) {
                _fuse_vel_data = false;
                reset_vel_ne = true;
            } else {
                _fuse_vel_data = true;
            }
            if (reset_vel_ne) {
                _s.state_struct.velocity.x = 0;
                _s.state_struct.velocity.y = 0;
            }
        } else {
            _fuse_pos_data = true;
            _fuse_vel_data = false;
            _vel_pos_obs[3] = _last_known_position_ne.x;
            _vel_pos_obs[4] = _last_known_position_ne.y;
        }

    }
    if (_fuse_vel_data || _fuse_pos_data || _fuse_hgt_data) {
        fuse_vel_pos_ned();
        _fuse_vel_data = false;
        _fuse_pos_data = false;
        _fuse_hgt_data = false;
    }
}
