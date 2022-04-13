#ifndef COMPASS_H_
#define COMPASS_H_

#include <stdint.h>
#include <stdbool.h>
#include "fusion_math.h"
#include "vector3f.h"
#include "matrix3f.h"
#include "rotation.h"

#define COMPASS_MOT_COMP_DISABLED 0X00
#define COMPASS_MOT_COMP_THROTTLE 0X01
#define COMPASS_MOT_COMP_CURRENT 0X02
#define COMPASS_MOT_COMP_PER_MOTOR 0x03

#define COMPASS_CAL_ENABLED 1
#define COMPASS_MOT_ENABLED 0
#define COMPASS_LEARN_ENABLED 0

#define COMPASS_CALIBRATION_FITNESS_DEFAULT 16.0f
#define COMPASS_MAX_XYZ_ANG_DIFF radians(90.0f)
#define COMPASS_MAX_XY_ANG_DIFF radians(60.0f)
#define COMPASS_MAX_XY_LENGTH_DIFF 200.0f

#define HAL_COMPASS_MAX_SENSORS 1

#if HAL_COMPASS_MAX_SENSORS > 1
#define COMPASS_MAX_UNREG_DEV 5
#else
#define COMPASS_MAX_UNREG_DEV 0
#endif

#define COMPASS_MAX_INSTANCES HAL_COMPASS_MAX_SENSORS
#define COMPASS_MAX_BACKEND HAL_COMPASS_MAX_SENSORS

typedef uint8_t Priority;
typedef uint8_t StateIndex;

typedef enum {
DRIVER_HMC5843 = 0,
DRIVER_LSM303D = 1,
DRIVER_AK8963 = 2,
} compass_drive_type_t;

typedef struct {
    int8_t external;
    bool healthy;
    bool registered;
    Priority priority;
    int8_t orientation;
    vector3f_t offset;
    vector3f_t diagonals;
    vector3f_t offdiagonals;
    float scale_factor;
    int32_t dev_id;
    int32_t detected_dev_id;
    int32_t expected_dev_id;
    vector3f_t motor_compensation;
    vector3f_t motor_offset;
    vector3f_t field; // corrected magnetic field strength
    uint32_t last_update_ms;
    uint32_t last_update_usec;
    Rotation_t rotation;
    vector3f_t accum;
    uint32_t accum_count;
} mag_state_t;

typedef enum {
CAL_REQUIRE_GPS = (1U<<0)
} compass_offset_t;

typedef enum {
COMPASS_LEARN_NONE = 0,
COMPASS_LEARN_INTERNAL = 1,
COMPASS_LEARN_EKF = 2,
COMPASS_LEARN_INFLIGHT = 3
} compass_learn_type_t;

void compass_init(void);
bool compass_read(void);
bool compass_healthy(void);
bool compass_use_for_yaw(void);
bool compass_have_scale_factor(void);
float compass_get_declination(void);
bool compass_auto_declination_enabled(void);
bool compass_consistent(void);
vector3f_t *compass_get_field(void);
uint32_t compass_last_update_usec(void);
bool compass_available(void);
bool compass_start_calibration(void);
bool compass_is_calibrating(void);
float compass_calculate_heading(const matrix3f_t *dcm_matrix);
void compass_cal_update(void);
vector3f_t compass_get_offsets(void);
uint8_t compass_get_num_enabled(void);
#endif // COMPASS_H_
