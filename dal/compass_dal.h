#ifndef COMPASS_DAL_H_
#define COMPASS_DAL_H_
#include <stdbool.h>

void dal_compass_start_frame(void);
float dal_compass_get_declination(void);
bool dal_compass_healthy(void);
bool dal_compass_use_for_yaw(void);
bool dal_compass_have_scale_factor(void);
bool dal_compass_auto_declination_enabled(void);

#endif // COMPASS_DAL_H_
