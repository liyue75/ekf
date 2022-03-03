#include "ekf_source.h"
source_set_t _source_set = {
XY_GPS,
XY_GPS,
//Z_BARO,
Z_GPS,
Z_GPS,
YAW_COMPASS
};

bool ekf_use_vel_z_source(source_z_t velz_source)
{
    if (velz_source == _source_set.velz) {
        return true;
    }
    return false;
}
