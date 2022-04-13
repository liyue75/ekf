#ifndef COMPASS_CALIBRATOR_H_
#define COMPASS_CALIBRATOR_H_

#include <stdint.h>
#include <stddef.h>
#include "vector3f.h"

#define COMPASS_CAL_NUM_SPHERE_PARAMS 4
#define COMPASS_CAL_NUM_ELLIPSOID_PARAMS 9
#define COMPASS_CAL_NUM_SAMPLES 300

#define COMPASS_MAX_SCALE_FACTOR 1.5
#define COMPASS_MIN_SCALE_FACTOR (1.0 / COMPASS_MAX_SCALE_FACTOR)

typedef enum {
COMPASS_CAL_NOT_STARTED = 0,
COMPASS_CAL_WAITING_TO_START = 1,
COMPASS_CAL_RUNNING_STEP_ONE = 2,
COMPASS_CAL_RUNNING_STEP_TWO = 3,
COMPASS_CAL_SUCCESS = 4,
COMPASS_CAL_FAILED = 5,
COMPASS_CAL_BAD_ORIENTATION = 6,
COMPASS_CAL_BAD_RADIUS = 7,
} compass_cal_status_t;

typedef uint8_t completion_mask_t[10];

typedef struct {
    compass_cal_status_t status;
    uint8_t attempt;
    float completion_pct;
    completion_mask_t completion_mask;
} compass_cal_state_t;

typedef struct {
    compass_cal_status_t status;
    float fitness;
    vector3f_t ofs;
    vector3f_t diag;
    vector3f_t offdiag;
    float orientation_confidence;
    Rotation_t original_orientation;
    Rotation_t orientation;
    float scale_factor;
    bool check_orientation;
} compass_report_t;

typedef struct {
    float tolerance;
    bool check_orientation;
    Rotation_t orientation;
    Rotation_t orig_orientation;
    bool is_external;
    bool fix_orientation;
    uint16_t offset_max;
    uint8_t attempt;
    bool retry;
    float delay_start_sec;
    uint32_t start_time_ms;
    uint8_t compass_idx;
    bool always_45_deg;
} compass_cal_settings_t;

typedef struct {
    float radius;
    vector3f_t offset;
    vector3f_t diag;
    vector3f_t offdiag;
    float scale_factor;
} compass_cal_param_t;

typedef struct {
    int8_t roll;
    int8_t pitch;
    int8_t yaw;
} compass_cal_attitude_sample_t;

typedef struct {
    compass_cal_attitude_sample_t att;
    int16_t x;
    int16_t y;
    int16_t z;
} compass_cal_compass_sample_t;

compass_cal_state_t compass_calibrator_get_state(void);
void compass_calibrator_init(void);
bool compass_calibrator_set_status(compass_cal_status_t _status);
void compass_calibrator_new_sample(const vector3f_t *sample);
void compass_calibrator_set_orientation(Rotation_t orientation, bool fix_orientation,
                                        bool always_45_deg);
void compass_calibrator_start(bool retry, float delay, uint16_t offset_max, float tolerance);
void compass_calibrator_update(void);
bool compass_calibrator_failed(void);
bool compass_calibrator_running(void);
compass_report_t compass_calibrator_get_report(void);
void compass_calibrator_stop(void);
#endif // COMPASS_CALIBRATOR_H_
