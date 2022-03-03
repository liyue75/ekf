#include <math.h>
#include "matrix3f.h"
#include "declination.h"
#include "common.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static const float SAMPLING_RES = 10;
static const float SAMPLING_MIN_LAT = -90;
static const float SAMPLING_MAX_LAT = 90;
static const float SAMPLING_MIN_LON = -180;
static const float SAMPLING_MAX_LON = 180;
extern float declination_table[19][37];
extern float inclination_table[19][37];
extern float intensity_table[19][37];

bool get_mag_field_ef(__attribute__((unused))float latitude_deg,
                      __attribute__((unused))float longitude_deg,
                      __attribute__((unused))float *intensity_gauss,
                      __attribute__((unused))float *declination_deg,
                      __attribute__((unused))float *inclination_deg)
{
    bool valid_input_data = true;
    int32_t min_lat = (int32_t)((int32_t)(floorf(latitude_deg / SAMPLING_RES)) * SAMPLING_RES);
    int32_t min_lon = (int32_t)((int32_t)(floorf(longitude_deg / SAMPLING_RES)) * SAMPLING_RES);
    if (latitude_deg <= SAMPLING_MIN_LAT) {
        min_lat = (int32_t)SAMPLING_MIN_LAT;
        valid_input_data = false;
    }
    if (latitude_deg >= SAMPLING_MAX_LAT) {
        min_lat = (int32_t)((int32_t)(latitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }
    if (longitude_deg <= SAMPLING_MIN_LON) {
        min_lon = (int32_t)SAMPLING_MIN_LON;
        valid_input_data = false;
    }
    if (longitude_deg >= SAMPLING_MAX_LON) {
        min_lon = (int32_t)((int32_t)(longitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES);
        valid_input_data = false;
    }
    uint32_t min_lat_index = (uint32_t)((-(SAMPLING_MIN_LAT) + min_lat) / SAMPLING_RES);
    uint32_t min_lon_index = (uint32_t)((-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES);
    //printf("min_lat_index = %ld, min_long_index = %ld\n", min_lat_index, min_lon_index);
    float data_sw = intensity_table[min_lat_index][min_lon_index];
    float data_se = intensity_table[min_lat_index][min_lon_index + 1];
    float data_ne = intensity_table[min_lat_index + 1][min_lon_index + 1];
    float data_nw = intensity_table[min_lat_index + 1][min_lon_index];
    float data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    float data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;
    *intensity_gauss = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;
    data_sw = declination_table[min_lat_index][min_lon_index];
    data_se = declination_table[min_lat_index][min_lon_index + 1];
    data_ne = declination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = declination_table[min_lat_index + 1][min_lon_index];
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;
    //printf("sw = %d, se = %d\n", (int)data_sw, (int)data_se);
    *declination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;
    data_sw = inclination_table[min_lat_index][min_lon_index];
    data_se = inclination_table[min_lat_index][min_lon_index + 1];
    data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = inclination_table[min_lat_index + 1][min_lon_index];
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;
    *inclination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;
    return valid_input_data;
}

float get_declination(float latitude_deg, float longitude_deg)
{
    float declination_deg = 0, inclination_deg = 0, intensity_gauss = 0;
    get_mag_field_ef(latitude_deg, longitude_deg, &intensity_gauss, &declination_deg,
                     &inclination_deg);
    return declination_deg;
}

vector3f_t get_earth_field_ga(const location_t *loc)
{
    float declination_deg = 0, inclination_deg = 0, intensity_gauss = 0;
    get_mag_field_ef(loc->lat * 1.0e-7f, loc->lng * 1.0e-7f, &intensity_gauss,
                     &declination_deg, &inclination_deg);
    vector3f_t mag_ef = {intensity_gauss, 0, 0};
    matrix3f_t R;
    m3f_from_euler(&R, 0.0f, -ToRad(inclination_deg), ToRad(declination_deg));
    mag_ef = m3f_multi_v(&R, &mag_ef);
    return mag_ef;
}
