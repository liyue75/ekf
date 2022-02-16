#ifndef LOCATION_H_
#define LOCATION_H_
#include <stdint.h>

typedef enum alt_frame {
ABSOLUTE = 0,
ABOVE_HOME = 1,
ABOVE_ORIGIN = 2,
ABOVE_TERRAIN = 3
} alt_frame_t;

typedef struct {
    int32_t alt;
    int32_t lat;
    int32_t lng;
} location_t;

void init_location(location_t *loc);

#endif // LOCATION_H_
