#include "ekf3_core.h"
#include "gps_dal.h"
#include "fusion_math.h"

extern float _vel_test_ratio;
extern float _pos_test_ratio;
extern float _hgt_test_ratio;
extern uint32_t _ekf_start_time_ms;
extern uint32_t _imu_sample_time_ms;
extern Vector6 _innov_vel_pos;
extern aiding_mode_t _pv_aiding_mode;
extern float _hgt_innov_filt_state;
extern bool _on_ground;
extern state_var_t _s;
extern fault_status_t _fault_status;
extern bool _states_initialised;

void ekf3_core_get_filter_faults(uint16_t *faults)
{
    *faults = (quat_isnan(&_s.state_struct.quat) << 0 |
               v3f_isnan(&_s.state_struct.velocity) << 1 |
               _fault_status.bad_xmag << 2 |
               _fault_status.bad_ymag << 3 |
               _fault_status.bad_zmag << 4 |
               _fault_status.bad_airspeed << 5 |
               _fault_status.bad_sideslip << 6 |
               !_states_initialised << 7);
}

bool ekf3_core_healthy(void)
{
    uint16_t fault_int;
    ekf3_core_get_filter_faults(&fault_int);
    if (fault_int > 0) {
        return false;
    }
    if (_vel_test_ratio > 1 && _pos_test_ratio > 1 && _hgt_test_ratio > 1) {
        return false;
    }
    if ((_imu_sample_time_ms - _ekf_start_time_ms) < 1000) {
        return false;
    }
    float horiz_err_sq = sq(_innov_vel_pos[3]) + sq(_innov_vel_pos[4]);
    if (_on_ground && (_pv_aiding_mode == AID_NONE) && ((horiz_err_sq > 1.0f) || (fabsf(_hgt_innov_filt_state) > 1.0f))) {
        return false;
    }
    return true;
}

bool ekf3_core_get_gps_llh(location_t *loc)
{
    if (dal_gps_status() >= DAL_GPS_OK_FIX_3D) {
        *loc = *dal_gps_location();
        return true;
    }
    return false;
}
