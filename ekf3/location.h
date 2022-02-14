#ifndef LOCATION_H_
#define LOCATION_H_
#include <stdint.h>

typedef enum alt_frame {
ABSOLUTE = 0,
ABOVE_HOME = 1,
ABOVE_ORIGIN = 2,
ABOVE_TERRAIN = 3
} alt_frame_t;

void init_location(void);

#endif // LOCATION_H_
