#include "ekf_source.h"
#include "compass_dal.h"

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

source_xy_t ekf_source_get_posxy_source(void)
{
    return _source_set.posxy;
}

source_yaw_t ekf_source_get_yaw_source(void)
{
    if (_source_set.yaw == YAW_COMPASS && (dal_compass_get_num_enabled() == 0)) {
        return YAW_NONE;
    }
    return _source_set.yaw;
}

bool ekf_source_use_vel_xy_source(source_xy_t velxy_source)
{
    if (velxy_source == _source_set.velxy) {
        return true;
    }
    return false;
}

source_z_t ekf_source_get_posz_source(void)
{
    return _source_set.posz;
}

bool ekf_have_vel_z_source(void)
{
    if (_source_set.velz != Z_NONE) {
        return true;
    }
    return false;
}
