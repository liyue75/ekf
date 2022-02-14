#include "location.h"
#include "definitions.h"

__attribute__((unused))static const float LOCATION_SCALING_FACTOR = LATLON_TO_M;
__attribute__((unused))static const float LOCATION_SCALING_FACTOR_INV = LATLON_TO_M_INV;
static int32_t alt;
static int32_t lat;
static int32_t lng;

void init_location(void)
{
    alt = 0;
    lat = 0;
    lng = 0;
}
