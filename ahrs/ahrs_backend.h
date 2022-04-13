#ifndef AHRS_BACKEND_H_
#define AHRS_BACKEND_H_

#include "vector3f.h"
#include "matrix3f.h"

#define AHRS_TRIM_LIMIT 10.0f
#define AHRS_RP_P_MIN 0.05f
#define AHRS_YAW_P_MIN 0.05f

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    matrix3f_t dcm_matrix;
    vector3f_t gyro_estimate;
    vector3f_t gyro_drift;
    vector3f_t accel_ef;
    vector3f_t accel_ef_blended;
    vector3f_t accel_bias;
} ahrs_estimates_t;

void ahrs_backend_update_trig(void);

#endif // AHRS_BACKEND_H_
