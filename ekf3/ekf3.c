#include <string.h>
#include "ekf3.h"
#include "ekf3_core.h"
#include "inertial_sensor.h"
#include "xtimer.h"
#include "fusion_math.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define VELNE_M_NSE_DEFAULT 0.5f
#define VELD_M_NSE_DEFAULT 0.7f
#define POSNE_M_NSE_DEFAULT 0.5f
#define ALT_M_NSE_DEFAULT 2.0f
#define MAG_M_NSE_DEFAULT 0.05f
#define GYRO_P_NSE_DEFAULT 1.5e-02f
#define ACC_P_NSE_DEFAULT 3.5E-01f
#define GBIAS_P_NSE_DEFAULT 1.0E-03f
#define ABIAS_P_NSE_DEFAULT 3.0E-03f
#define MAGB_P_NSE_DEFAULT 1.0E-04f
#define MAGE_P_NSE_DEFAULT 1.0E-03f
#define VEL_I_GATE_DEFAULT 500
#define POS_I_GATE_DEFAULT 500
#define HGT_I_GATE_DEFAULT 500
#define MAG_I_GATE_DEFAULT 300
#define MAG_CAL_DEFAULT 2
#define GLITCH_RADIUS_DEFAULT 25
#define FLOW_MEAS_DELAY 10
#define FLOW_M_NSE_DEFAULT 0.25f
#define FLOW_I_GATE_DEFAULT 300
#define CHECK_SCALER_DEFAULT 100
#define FLOW_USE_DEFAULT 1
#define WIND_P_NSE_DEFAULT 0.1
#define HAL_EKF_IMU_MASK_DEFAULT 1
#define EKF3_PRIMARY_DEFAULT 0;

float _gps_horiz_vel_noise = VELNE_M_NSE_DEFAULT;
float _gps_vert_vel_noise = VELD_M_NSE_DEFAULT;
int16_t _gps_vel_innov_gate = VEL_I_GATE_DEFAULT;
float _gps_horiz_pos_noise = POSNE_M_NSE_DEFAULT;
int16_t _gps_pos_innov_gate = POS_I_GATE_DEFAULT;
int8_t _gps_glitch_radius_max = GLITCH_RADIUS_DEFAULT;
float baro_alt_noise = ALT_M_NSE_DEFAULT;
int16_t _hgt_innov_gate = HGT_I_GATE_DEFAULT;
int16_t _hgt_delay_ms = 60;
float _mag_noise = MAG_M_NSE_DEFAULT;
int8_t _mag_cal = MAG_CAL_DEFAULT;
int16_t _mag_innov_gate = MAG_I_GATE_DEFAULT;
float _eas_noise = 1.4f;  //airspeed measurement noise
int16_t _tas_innov_gate = 400; //airspeed measurement gate size
float _rng_noise = 0.5f; //range finder
int16_t _rng_innov_gate = 500;
float _max_flow_rate = 2.5f;
float _flow_noise = FLOW_M_NSE_DEFAULT;
int16_t _flow_innov_gate = FLOW_I_GATE_DEFAULT;
int8_t _flow_delay_ms = FLOW_MEAS_DELAY;
float _gyr_noise = GYRO_P_NSE_DEFAULT;
float _acc_noise = ACC_P_NSE_DEFAULT;
float _gyro_bias_process_noise = GBIAS_P_NSE_DEFAULT;
float _accel_bias_process_noise = ABIAS_P_NSE_DEFAULT;
float _wind_vel_process_noise = WIND_P_NSE_DEFAULT;
float _wnd_var_hgt_rate_scale = 1.0f;
int8_t _gps_check = 31;
int8_t _imu_mask = HAL_EKF_IMU_MASK_DEFAULT;
int16_t _gps_check_scaler = CHECK_SCALER_DEFAULT;
float _noaid_horiz_noise = 10.0f;
int8_t _beta_mask = 0;
float _yaw_noise = 0.5f;
int16_t _yaw_innov_gate = 300;
int8_t _tau_vel_pos_output = 25;
float _mag_earth_process_noise = MAGE_P_NSE_DEFAULT;
float _mag_body_process_noise = MAGB_P_NSE_DEFAULT;
int8_t _use_rng_sw_hgt = -1;
float _terr_grad_max = 0.1f;
float _rng_bcn_noise = 1.0f;
int16_t _rng_bcn_innov_gate = 500;
int8_t _rng_bcn_delay_ms = 50;
float _use_rng_sw_spd = 2.0f;
float _acc_bias_lim = 1.0f;
int8_t _mag_mask = 0;
int8_t _origin_hgt_mode = 0;
float _vis_odm_vel_err_min = 0.1f;
float _vis_odm_vel_err_max = 0.9f;
float _wenc_odm_vel_err = 0.1f;
int8_t _flow_use = FLOW_USE_DEFAULT;
float _hrt_filt_freq = 2.0f;
int16_t _mag_ef_limit = 50;
int8_t _gsf_run_mask = 3;
int8_t _gsf_use_mask = 3;
int8_t _gsf_reset_max_count = 2;
float _err_thresh = 0.2;
int32_t _affinity = 0;
float _ballistic_coef_x = 0.0f;
float _ballistic_coef_y = 0.0f;
float _drag_obs_noise = 0.5f;
float _momentum_drag_coef = 0.0f;
float _ognm_test_scale_factor = 2.0f;
float _baro_gnd_effect_dead_zone = 4.0f;
int8_t _primary_core = EKF3_PRIMARY_DEFAULT;

const float gps_nevel_var_acc_scale = 0.05f;
const float gps_dvel_var_acc_scale = 0.07f;
const float gps_pos_var_acc_scale = 0.05f;
const float ext_nav_vel_var_acc_scale = 0.05f;
const uint16_t mag_delay_ms = 60;
const uint16_t tas_delay_ms = 100;
const uint16_t tilt_drift_time_max_ms = 15000;
const uint16_t pos_retry_time_use_vel_ms = 10000;
const uint16_t pos_retry_time_no_vel_ms = 7000;
const uint16_t hgt_retry_time_mode0_ms = 10000;
const uint16_t hgt_retry_time_mode12_ms = 5000;
const uint16_t tas_retry_time_ms = 5000;
const uint32_t mag_fail_time_limit_ms = 10000;
const float mag_var_rate_scale = 0.005f;
const float gyro_bias_noise_scaler = 2.0f;
const uint16_t hgt_avg_ms = 100;
const uint16_t beta_avg_ms = 100;
const float cov_time_step_max = 0.1f;
const float cov_del_ang_max = 0.05f;
const float dcm33_flow_min = 0.71f;
const float fscale_factor_pnoise = 1e-10f;
const uint8_t flow_time_delta_avg_ms = 100;
const uint32_t flow_interval_max_ms = 100;
const float gnd_effect_baro_scaler = 4.0f;
const uint8_t gnd_gradient_sigma = 50;
const uint16_t fusion_time_step_ms = 10;
const uint8_t sensor_interval_min_ms = 50;
const uint8_t flow_interval_min_ms = 20;
const uint8_t ext_nav_interval_min_ms = 20;
const float max_yaw_est_vel_innov = 2.0f;
const uint16_t dead_reckon_declare_ms = 1000;


yaw_reset_data_t yaw_reset_data;
pos_reset_data_t pos_reset_data;
pos_down_reset_data_t pos_down_reset_data;

static bool ekf_enable;
uint64_t _imu_sample_time_us;
static uint32_t frame_time_usec;
static uint8_t frames_per_prediction;
static bool common_origin_valid;
static uint64_t core_last_time_primary_us;

bool core_setup_required;
static float core_relative_errors;
__attribute__((unused))static float core_error_scores;
uint8_t core_imu_index;
uint8_t num_cores;

void set_ekf_enable(bool enable)
{
    ekf_enable = enable;
}

static void reset_core_errors(void)
{
    core_relative_errors = 0;
}

void ekf3_init(void)
{

}

bool ekf_initialise_filter(void)
{
    if (ekf_enable == 0) {
        return false;
    }
    _imu_sample_time_us = xtimer_now64().ticks64;
    const float loop_rate = get_ins_loop_rate_hz();
    if (!float_is_positive(loop_rate)) {
        return false;
    }
    DEBUG("loop rate = %d\n", (int)loop_rate);
    frame_time_usec = 1e6 / loop_rate;
    frames_per_prediction = (uint8_t)(EKF_TARGET_DT / (frame_time_usec * 1.0e-6) + 0.5);
    DEBUG("frame per prediction = %d\n", frames_per_prediction);
    if (true) {
        core_setup_required = true;
        core_imu_index = 0;
        num_cores = 1;
        init_ekf3_core();
    }
    bool core_setup_success = ekf3_setup_core();
    if (!core_setup_success) {
        return false;
    }
    reset_core_errors();
    common_origin_valid = false;
    bool ret = init_ekf3_core_filter_bootstrap();
    core_last_time_primary_us = 0;
    memset(&yaw_reset_data, 0, sizeof(yaw_reset_data));
    memset(&pos_reset_data, 0, sizeof(pos_reset_data));
    memset(&pos_down_reset_data, 0, sizeof(pos_down_reset_data));
    return ret;
}
