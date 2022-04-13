#ifndef LOCATION_H_
#define LOCATION_H_
#include <stdint.h>
#include <stdbool.h>
#include "vector2f.h"

typedef enum alt_frame {
ALT_ABSOLUTE = 0,
ALT_ABOVE_HOME = 1,
ALT_ABOVE_ORIGIN = 2,
ALT_ABOVE_TERRAIN = 3
} alt_frame_t;

typedef struct {
    int32_t alt;
    int32_t lat;
    int32_t lng;
    uint8_t relative_alt:1;
    uint8_t loiter_ccw:1;
    uint8_t terrain_alt:1;
    uint8_t origin_alt:1;
    uint8_t loiter_xtrack:1;
} location_t;

void init_location(location_t *loc);
location_t set_location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, alt_frame_t frame);
float location_get_distance(const location_t *loc1, const location_t *loc2);
int32_t diff_longitude(int32_t lon1, int32_t lon2);
float longitude_scale(int32_t lat);
vector2f_t location_get_distance_ne_float(const location_t *loc1, const location_t *loc2);
void location_offset(location_t *loc, float ofs_north, float ofs_east);
bool location_check_latlng(const location_t *loc);
bool location_change_alt_frame(location_t *loc, alt_frame_t  desired_frame);
#endif // LOCATION_H_
