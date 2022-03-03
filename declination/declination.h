#ifndef DECLINATION_H_
#define DECLINATION_H_

#include <stdbool.h>
#include "vector3f.h"
#include "location.h"

bool get_mag_field_ef(float latitude_deg, float longitude_deg, float *intensity_gauss,
                      float *declination_deg, float *inclination_deg);
float get_declination(float latitude_deg, float longitude_deg);
vector3f_t get_earth_field_ga(const location_t *loc);
#endif // DECLINATION_H_
