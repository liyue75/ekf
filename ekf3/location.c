#include "location.h"
#include "fusion_math.h"
#include "definitions.h"

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
