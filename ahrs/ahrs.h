#ifndef AHRS_H_
#define AHRS_H_

#include <stdbool.h>
#include "location.h"
#include "matrix3f.h"

typedef enum vehicle_class {
UNKNOWN,
GROUND
} vehicle_class_t;

void ahrs_init(void);
bool get_takeoff_expected(void);
bool get_fly_forward(void);
void set_fly_forward(bool b);
void ahrs_reset(void);
bool ahrs_get_position(location_t *loc);
location_t ahrs_get_home(void);
matrix3f_t *get_rotation_autopilot_body_to_vehicle_body(void);
matrix3f_t *get_rotation_vehicle_body_to_autopilot_body(void);
void ahrs_reset_gyro_drift(void);
void ahrs_update(void);
bool ahrs_get_origin(location_t *ret);
const matrix3f_t* ahrs_get_dcm_rotation_body_to_ned(void);
matrix3f_t *ahrs_get_rotation_body_to_ned(void);
bool ahrs_home_is_set(void);
bool ahrs_set_home(const location_t *loc);
void ahrs_lock_home(void);
#endif // AHRS_H_
