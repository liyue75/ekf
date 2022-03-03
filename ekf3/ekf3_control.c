#include "ekf3_core.h"
#include "location.h"
//#include "compass.h"
#include "compass_dal.h"

extern bool _valid_origin;
extern location_t _ekf_origin;
extern location_t *_public_origin;
extern double _ekf_gps_ref_hgt;
extern vector3f_t _earth_rate_ned;
extern bool _common_origin_valid;
static bool all_mag_sensors_failed;

bool ekf3_core_use_compass(void)
{
    source_yaw_t yaw_source = ekf3_get_yaw_source();
    if (yaw_source != YAW_COMPASS && yaw_source != YAW_GPS_COMPASS_FALLBACK) {
        return false;
    }
    return dal_compass_use_for_yaw() && !all_mag_sensors_failed;
}

bool ekf3_core_set_origin(const location_t *loc)
{
    if (_valid_origin) {
        return false;
    }
    _ekf_origin = *loc;
    _ekf_gps_ref_hgt = (double)0.01 * (double)_ekf_origin.alt;
    ekf3_core_calc_earth_rate_ned(&_earth_rate_ned, _ekf_origin.lat);
    _valid_origin = true;
    if (!_common_origin_valid) {
        _common_origin_valid = true;
        _public_origin = &_ekf_origin;
    }
    return true;
}
