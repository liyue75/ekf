#include "ekf3_core.h"

bool ekf3_core_use_compass(void)
{
    source_yaw_t yaw_source = ekf3_get_yaw_source();
    if (yaw_source != YAW_COMPASS && yaw_source != YAW_GPS_COMPASS_FALLBACK) {
        return false;
    }
    return true;
}
