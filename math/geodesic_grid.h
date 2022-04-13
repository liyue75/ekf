#ifndef GEODESIC_GRID_H_
#define GEODESIC_GRID_H_

#include <stdint.h>
#include "vector3f.h"

typedef struct {
    uint8_t components[5];
    uint8_t v0_c0;
    uint8_t v1_c1;
    uint8_t v2_c1;
    uint8_t v4_c4;
    uint8_t v0_c4;
} neighbor_umbrella_t;

int geodesic_grid_section(const vector3f_t *v, bool inclusive);

#endif // GEODESIC_GRID_H_
