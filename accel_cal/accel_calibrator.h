#ifndef ACCEL_CALIBRATOR_H_
#define ACCEL_CALIBRATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "vector3f.h"

#define ACCEL_CAL_MAX_NUM_PARAMS 9
#define ACCEL_CAL_TOLERANCE 0.1
#define MAX_ITERATIONS 50

typedef enum accel_cal_status {
ACCEL_CAL_NOT_STARTED = 0,
ACCEL_CAL_WAITING_FOR_ORIENTATION = 1,
ACCEL_CAL_COLLECTING_SAMPLE = 2,
ACCEL_CAL_SUCCESS = 3,
ACCEL_CAL_FAILED = 4
} accel_cal_status_t;

typedef enum {
ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID = 0,
ACCEL_CAL_ELLIPSOID = 1
} accel_cal_fit_type_t;

typedef struct {
    vector3f_t delta_velocity;
    float delta_time;
} cal_accel_sample_t;

typedef struct {
    float conf_tolerance;
    cal_accel_sample_t *sample_buffer;
} accel_calibrator_t;

typedef struct {
    vector3f_t offset;
    vector3f_t diag;
    vector3f_t offdiag;
} acal_cal_param_t;

typedef union {
    acal_cal_param_t s;
    float a[ACCEL_CAL_MAX_NUM_PARAMS];
} acal_cal_param_u;

typedef enum {
ACCELCAL_VEHICLE_POS_FAILED = 0,
ACCELCAL_VEHICLE_POS_LEVEL = 1,
ACCELCAL_VEHICLE_POS_LEFT = 2,
ACCELCAL_VEHICLE_POS_RIGHT = 3,
ACCELCAL_VEHICLE_POS_NOSEDOWN = 4,
ACCELCAL_VEHICLE_POS_NOSEUP = 5,
ACCELCAL_VEHICLE_POS_BACK = 6,
ACCELCAL_VEHICLE_POS_SUCCESS = 7
} accelcal_pos_t;

void acal_cal_clear(void);
accel_cal_status_t acal_cal_get_status(void);
void acal_cal_new_sample(const vector3f_t* delta_velocity, float dt);
void acal_cal_start(accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time);
void acal_cal_collect_sample(void);
uint8_t acal_cal_get_num_samples_collected(void);
void acal_cal_check_for_timeout(void);
void acal_cal_get_calibration(vector3f_t *offset, vector3f_t *diag);

#endif // ACCEL_CALIBRATOR_H_
