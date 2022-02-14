#ifndef EKF3_CORE_H_
#define EKF3_CORE_H_

#include <stdint.h>
#include <stdbool.h>
#include "ekf_source.h"
#include "quaternion.h"
#include "vector3f.h"
#include "vector2f.h"
#include "matrix3f.h"

#define earthRate 0.000072921f

#define EKF_TARGET_DT_MS 12
#define EKF_TARGET_DT 0.012f

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
    int32_t lat, lng;
    float hgt;
    vector3f_t vel;
    uint8_t sensor_idx;
    bool corrected;
    bool have_vz;
} gps_elements_t;

typedef struct mag_elements {
    vector3f_t mag;
} mag_elements_t;

typedef struct {
    source_xy_t posxy;
    source_xy_t velxy;
    source_z_t posz;
    source_z_t velz;
    source_yaw_t yaw;
} source_set_t;

typedef struct {
    uint32_t count;
    float dt_imu_avg_min;
    float dt_imu_avg_max;
    float dt_ekf_avg_min;
    float dt_ekf_avg_max;
    float del_ang_dt_max;
    float del_ang_dt_min;
    float del_vel_dt_max;
    float del_vel_dt_min;
} ekf_timing_t;

typedef struct {
    vector3f_t gyro_bias;
    vector3f_t accel_bias;
} inactive_bias_t;

void init_ekf3_core(void);
bool ekf3_setup_core(void);
bool init_ekf3_core_filter_bootstrap(void);
void ekf3_core_update_sensor_selection(void);
void ekf3_core_read_imu_data(void);
void ekf3_core_read_gps_data(void);
source_yaw_t ekf3_get_yaw_source(void);
bool ekf3_core_use_compass(void);
void ekf3_core_correct_delta_angle(vector3f_t *del_ang, float del_ang_dt);
void ekf3_core_correct_delta_velocity(vector3f_t *del_vel, float del_vel_dt);
//void ekf3_core_read_mag_date(void);
void ekf3_core_read_gps_data(void);
#endif // EKF3_CORE_H_
