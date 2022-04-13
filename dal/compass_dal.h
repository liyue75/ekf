#ifndef COMPASS_DAL_H_
#define COMPASS_DAL_H_
#include <stdbool.h>
#include <stdint.h>
#include "vector3f.h"

void dal_compass_start_frame(void);
float dal_compass_get_declination(void);
bool dal_compass_healthy(void);
bool dal_compass_use_for_yaw(void);
bool dal_compass_have_scale_factor(void);
bool dal_compass_auto_declination_enabled(void);
bool dal_compass_available(void);
uint8_t dal_compass_get_count(void);
uint32_t dal_compass_last_update_usec(void);
vector3f_t dal_compass_get_offsets(void);
vector3f_t dal_compass_get_field(void);
bool dal_compass_consistent(void);
uint8_t dal_compass_get_num_enabled(void);
#endif // COMPASS_DAL_H_
