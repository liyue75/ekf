#include "location.h"
#include "fusion_math.h"
#include "definitions.h"
#include "ahrs.h"

const float LOCATION_SCALING_FACTOR = LATLON_TO_M;
const float LOCATION_SCALING_FACTOR_INV = LATLON_TO_M_INV;

location_t _location;

void init_location(location_t *location)
{
    location->alt = 0;
    location->lat = 0;
    location->lng = 0;
}

int32_t diff_longitude(int32_t lon1, int32_t lon2)
{
    if ((lon1 & 0x80000000) == (lon2 & 0x80000000)) {
        return lon1 - lon2;
    }
    int64_t dlon = (int64_t)(lon1) - (int64_t)(lon2);
    if (dlon > 1800000000LL) {
        dlon -= 3600000000LL;
    } else if (dlon < -1800000000LL) {
        dlon += 3600000000LL;
    }
    return (int32_t)(dlon);
}

float longitude_scale(int32_t lat)
{
    float scale = cosf(lat * (1.0e-7 * DEG_TO_RAD));
    return MAX(scale, 0.01);
}

float location_get_distance(const location_t *loc1, const location_t *loc2)
{
    float dlat = (float)(loc2->lat - loc1->lat);
    float dlng = (float)diff_longitude(loc2->lng, loc1->lng) *
        longitude_scale((loc1->lat + loc2->lat) / 2);
    return norm_2f(dlat, dlng) * LOCATION_SCALING_FACTOR;
}

static alt_frame_t get_alt_frame(location_t *loc)
{
    if (loc->terrain_alt) {
        return ALT_ABOVE_TERRAIN;
    }
    if (loc->origin_alt) {
        return ALT_ABOVE_ORIGIN;
    }
    if (loc->relative_alt) {
        return ALT_ABOVE_HOME;
    }
    return ALT_ABSOLUTE;
}

static bool get_alt_cm(location_t *loc, alt_frame_t desired_frame, int32_t *ret_alt_cm)
{
    alt_frame_t frame = get_alt_frame(loc);
    if (desired_frame == frame) {
        *ret_alt_cm = loc->alt;
        return true;
    }
    if (frame == ALT_ABOVE_TERRAIN || desired_frame == ALT_ABOVE_TERRAIN) {
        return false;
    }
    int32_t alt_abs = 0;
    location_t ekf_origin;
    switch (frame) {
        case ALT_ABSOLUTE:
            alt_abs = loc->alt;
            break;
        case ALT_ABOVE_HOME:
            if (!ahrs_home_is_set()) {
                return false;
            }
            alt_abs = loc->alt + ahrs_get_home().alt;
            break;
        case ALT_ABOVE_ORIGIN:
            if (!ahrs_get_origin(&ekf_origin)) {
                return false;
            }
            alt_abs = loc->alt + ekf_origin.alt;
            break;
        case ALT_ABOVE_TERRAIN:
            return false;
    }
    switch (desired_frame) {
        case ALT_ABSOLUTE:
            *ret_alt_cm = alt_abs;
            return true;
        case ALT_ABOVE_HOME:
            if (!ahrs_home_is_set()) {
                return false;
            }
            *ret_alt_cm = alt_abs - ahrs_get_home().alt;
            return true;
        case ALT_ABOVE_ORIGIN:
            if (!ahrs_get_origin(&ekf_origin)) {
                return false;
            }
            *ret_alt_cm = alt_abs - ekf_origin.alt;
            return true;
        case ALT_ABOVE_TERRAIN:
            return false;
    }
    return false;
}


static void set_alt_cm(location_t *loc, int32_t alt_in_cm, alt_frame_t frame)
{
    loc->alt = alt_in_cm;
    loc->relative_alt = false;
    loc->terrain_alt = false;
    loc->origin_alt = false;
    switch (frame) {
        case ALT_ABSOLUTE:
            break;
        case ALT_ABOVE_HOME:
            loc->relative_alt = true;
            break;
        case ALT_ABOVE_ORIGIN:
            loc->origin_alt = true;
            break;
        case ALT_ABOVE_TERRAIN:
            loc->relative_alt = true;
            loc->terrain_alt = true;
            break;
    }
}

location_t set_location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, alt_frame_t frame)
{
    location_t tmp = {.alt = 0, .lat = latitude, .lng = longitude};
    set_alt_cm(&tmp, alt_in_cm, frame);
    return tmp;
}

vector2f_t location_get_distance_ne_float(const location_t *loc1, const location_t *loc2)
{
    vector2f_t tmp = {(loc2->lat - loc1->lat) * LOCATION_SCALING_FACTOR,
    diff_longitude(loc2->lng, loc1->lng) * LOCATION_SCALING_FACTOR *
    longitude_scale((loc1->lat + loc2->lat) / 2)};
    return tmp;
}

static int32_t limit_lattitude(int32_t lat)
{
    if (lat > 900000000L) {
        lat = 1800000000LL - lat;
    } else if (lat < -900000000L) {
        lat = -(1800000000LL + lat);
    }
    return lat;
}

static int32_t wrap_longitude(int64_t lon)
{
    if (lon > 1800000000L) {
        lon = (int32_t)(lon - 3600000000LL);
    } else if (lon < -1800000000L) {
        lon = (int32_t)(lon + 3600000000LL);
    }
    return (int32_t)lon;
}

static void offset_latlng(location_t *loc, float ofs_north, float ofs_east)
{
    const int32_t dlat  = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int64_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) /
        longitude_scale(loc->lat + dlat / 2);
    loc->lat = loc->lat + dlat;
    loc->lat = limit_lattitude(loc->lat);
    loc->lng = wrap_longitude(dlng + loc->lng);
}

void location_offset(location_t *loc, float ofs_north, float ofs_east)
{
    offset_latlng(loc, ofs_north, ofs_east);
}

static bool check_lat(int32_t lat)
{
    return fabsf(lat) <= 90*1e7;
}

static bool check_lng(int32_t  lng)
{
    return fabsf(lng) <= 180*1e7;
}

bool location_check_latlng(const location_t *loc)
{
    return check_lat(loc->lat) && check_lng(loc->lng);
}

bool location_change_alt_frame(location_t *loc, alt_frame_t  desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(loc, desired_frame, &new_alt_cm)) {
        return false;
    }
    set_alt_cm(loc, new_alt_cm, desired_frame);
    return true;
}
