#include "location.h"
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
