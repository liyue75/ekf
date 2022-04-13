#ifndef EKF3_CORE_H_
#define EKF3_CORE_H_

#include <stdint.h>
#include <stdbool.h>
#include "ekf_source.h"
#include "nav_common.h"
#include "quaternion.h"
#include "vector3f.h"
#include "vector2f.h"
#include "matrix3f.h"
#include "location.h"

#define MASK_GPS_NSATS (1<<0)
#define MASK_GPS_HDOP (1<<1)
#define MASK_GPS_SPD_ERR (1<<2)
#define MASK_GPS_POS_ERR (1<<3)
#define MASK_GPS_YAW_ERR (1<<4)
#define MASK_GPS_POS_DRIFT (1<<5)
#define MASK_GPS_VERT_SPD (1<<6)
#define MASK_GPS_HORIZ_SPD (1<<7)

#define earthRate 0.000072921f

#define GYRO_BIAS_LIMIT 0.5f
//#define ACCEL_BIAS_LIM_SCALER 0.2f
#define ACCEL_BIAS_LIM_SCALER 0.5f

#define EKF_TARGET_DT_MS 12
#define EKF_TARGET_DT 0.012f
#define EKF3_MAG_FINAL_RESET_ALT 2.5f
#define MAG_ANOMALY_RESET_MAX 2
#define VEL_STATE_MIN_VARIANCE 1E-4
#define POS_STATE_MIN_VARIANCE 1E-4

#define EKF_TARGET_RATE_HZ (uint32_t)(1.0 / EKF_TARGET_DT)
#define VERT_VEL_VAR_CLIP_COUNT_LIM (5 * EKF_TARGET_RATE_HZ)

#define EKF3_POSXY_STATE_LIMIT 1.0e6

#define BAD_IMU_DATA_ACC_P_NSE 5.0f
#define BAD_IMU_DATA_TIMEOUT_MS 1000
#define BAD_IMU_DATA_HOLD_MS 1000

#define WIND_VEL_VARIANCE_MAX 400.0f
#define WIND_VEL_VARIANCE_MIN 0.25f

typedef float Vector2[2];
typedef float Vector3[3];
typedef float Vector4[4];
typedef float Vector5[5];
typedef float Vector6[6];
typedef float Vector7[7];
typedef float Vector8[8];
typedef float Vector9[9];
typedef float Vector10[10];
typedef float Vector11[11];
typedef float Vector12[12];
typedef float Vector13[13];
typedef float Vector14[14];
typedef float Vector15[15];
typedef float Vector16[16];
typedef float Vector17[17];
typedef float Vector18[18];
typedef float Vector19[29];
typedef float Vector20[20];
typedef float Vector21[21];
typedef float Vector22[22];
typedef float Vector23[23];
typedef float Vector24[24];
typedef float Vector25[25];
typedef float Matirx3[3][3];
typedef float Matrix24[24][24];
typedef float Vector28[28];
typedef float Matrix34_50[34][50];
typedef float Vector_u32_50[50];

typedef struct state_elements {
    quaternionf_t quat;
    vector3f_t velocity;
    vector3f_t position;
    vector3f_t gyro_bias;
    vector3f_t accel_bias;
    vector3f_t earth_magfield;
    vector3f_t body_magfield;
    vector2f_t wind_vel;
} state_elements_t;

typedef union {
    float states_array[24];
    state_elements_t state_struct;
} state_var_t;

typedef struct output_elements {
    quaternionf_t quat;
    vector3f_t velocity;
    vector3f_t position;
} output_elements_t;

typedef struct imu_elements {
    vector3f_t del_ang;
    vector3f_t del_vel;
    float del_ang_dt;
    float del_vel_dt;
    uint32_t time_ms;
    uint8_t gyro_index;
    uint8_t accel_index;
} imu_elements_t;

typedef struct gps_elements {
    uint32_t time_ms;
    int32_t lat, lng;
    float hgt;
    vector3f_t vel;
    uint8_t sensor_idx;
    bool corrected;
    bool have_vz;
} gps_elements_t;

typedef struct mag_elements {
    uint32_t time_ms;
    vector3f_t mag;
} mag_elements_t;

typedef struct {
    vector3f_t gyro_bias;
    vector3f_t accel_bias;
} inactive_bias_t;

typedef struct {
    bool bad_xmag:1;
    bool bad_ymag:1;
    bool bad_zmag:1;
    bool bad_airspeed:1;
    bool bad_sideslip:1;
    bool bad_nvel:1;
    bool bad_evel:1;
    bool bad_dvel:1;
    bool bad_npos:1;
    bool bad_epos:1;
    bool bad_dpos:1;
    bool bad_yaw:1;
    bool bad_decl:1;
    bool bad_xflow:1;
    bool bad_yflow:1;
    bool bad_rngbcn:1;
    bool bad_xvel:1;
    bool bad_yvel:1;
    bool bad_zvel:1;
} fault_status_t;

typedef union {
    struct {
        bool bad_s_acc:1;
        bool bad_h_acc:1;
        bool bad_v_acc:1;
        bool bad_yaw:1;
        bool bad_sats:1;
        bool bad_vz:1;
        bool bad_horiz_drift:1;
        bool bad_hdop:1;
        bool bad_vert_vel:1;
        bool bad_fix:1;
        bool bad_horiz_vel:1;
    };
    uint16_t value;
} gps_check_status_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float mag_n;
    float mag_e;
    float mag_d;
    float mag_x_bias;
    float mag_y_bias;
    float mag_z_bias;
    matrix3f_t DCM;
    vector3f_t mag_pred;
    float R_mag;
    Vector9 SH_mag;
} mag_state_t;

typedef enum {
AID_ABSOLUTE = 0,
AID_NONE = 1,
AID_RELATIVE = 2
} aiding_mode_t;

typedef struct {
    float pos;
    float vel;
    float acc;
} vert_vel_t;

typedef enum {
MAG_CAL_WHEN_FLYING = 0,
MAG_CAL_WHEN_MANOEUVRING = 1,
MAG_CAL_NEVER = 2,
MAG_CAL_AFTER_FIRST_CLIMB,
MAG_CAL_ALWAYS = 4
} mag_cal_t;

typedef enum {
RESET_DATA_SOURCE_DEFAULT = 0,
RESET_DATA_SOURCE_GPS = 1,
RESET_DATA_SOURCE_RNGBCN = 2,
RESET_DATA_SOURCE_FLOW = 3,
RESET_DATA_SOURCE_BARO = 4,
RESET_DATA_SOURCE_MAG = 5,
RESET_DATA_SOURCE_RNGFND = 6,
RESET_DATA_SOURCE_EXTNAV = 7
} reset_data_source_t;

typedef enum {
TAIT_BRYAN_321 = 0,
TAIT_BRYAN_312 = 1
} rotation_order_t;

typedef struct {
    uint32_t time_ms;
    float yaw_ang;
    float yaw_ang_err;
    rotation_order_t order;
} yaw_elements_t;

typedef enum {
YAW_FUSION_MAGNETOMETER = 0,
YAW_FUSION_GPS = 1,
YAW_FUSION_GSF = 2,
YAW_FUSION_STATIC = 3,
YAW_FUSION_PREDICTED = 4,
YAW_FUSION_EXTNAV = 5,
} yaw_fusion_method_t;

void init_ekf3_core(void);
bool ekf3_setup_core(void);
bool init_ekf3_core_filter_bootstrap(void);
void ekf3_core_update_sensor_selection(void);
void ekf3_core_read_imu_data(void);
void ekf3_core_read_mag_data(void);
void ekf3_core_read_gps_data(void);
source_yaw_t ekf3_get_yaw_source(void);
bool ekf3_core_use_compass(void);
void ekf3_core_correct_delta_angle(vector3f_t *del_ang, float del_ang_dt);
void ekf3_core_correct_delta_velocity(vector3f_t *del_vel, float del_vel_dt);
void ekf3_core_calc_gps_good_to_align(void);
void ekf3_core_calc_gps_good_for_flight(void);
void ekf3_core_calc_earth_rate_ned(vector3f_t *omega, int32_t latitude);
bool ekf3_core_set_origin(const location_t *loc);
void ekf3_core_align_mag_state_declination(void);
float ekf3_core_mag_declination(void);
void ekf3_core_zero_range(float *v, uint8_t n1, uint8_t n2);
void ekf3_core_zero_rows(Matrix24 *cov_mat, uint8_t first, uint8_t last);
void ekf3_core_zero_cols(Matrix24 *cov_mat, uint8_t first, uint8_t last);
bool ekf3_core_get_llh(location_t *loc);
void ekf3_core_force_symmetry(void);
void ekf3_core_constrain_variances(void);
void ekf3_core_reset_gyro_bias(void);
bool ekf3_core_get_origin_llh(location_t *loc);
mag_cal_t ekf3_core_effective_mag_cal(void);
void ekf3_core_reset_velocity(reset_data_source_t vel_reset_source);
void ekf3_core_reset_position(reset_data_source_t pos_reset_source);
void ekf3_core_reset_height(void);
void ekf3_core_fuse_declination(float decl_err);
void ekf3_core_reset_gyro_bias(void);
float ekf3_core_initial_gyro_bias_uncertainty(void);
uint8_t ekf3_core_get_frames_since_predict(void);
void ekf3_core_update_filter(bool predict);
void ekf3_core_control_filter_modes(void);
void ekf3_core_detect_flight(void);
bool ekf3_core_assume_zero_sideslip(void);
void ekf3_core_select_mag_fusion(void);
void ekf3_core_set_yaw_from_mag(void);
void ekf3_core_best_rotation_order(rotation_order_t *order);
void ekf3_core_reset_quat_state_yaw_only(float yaw, float yaw_variance, rotation_order_t order);
void ekf3_core_store_quat_rotate(const quaternionf_t *delta_quat);
void ekf3_core_covariance_prediction(vector3f_t *rot_var_vec_ptr);
void ekf3_core_record_yaw_reset(void);
void ekf3_core_reset_mag_field_states(void);
void ekf3_core_record_mag_reset(void);
void ekf3_core_covariance_init(void);
void ekf3_core_mag_table_constrain(void);
void ekf3_core_select_vel_pos_fusion(void);
void ekf3_core_realign_yaw_gps(void);
void ekf3_core_update_filter_status(void);
void ekf3_core_get_filter_faults(uint16_t *faults);
bool ekf3_core_healthy(void);
void ekf3_core_check_update_earth_field(void);
void ekf3_core_get_euler_angles(vector3f_t *euler);
bool ekf3_core_get_pos_d(float *pos_d);
bool ekf3_core_get_gps_llh(location_t *loc);
void ekf3_core_get_vel_ned(vector3f_t *vel);

#endif // EKF3_CORE_H_
