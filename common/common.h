#ifndef COMMON_H_
#define COMMON_H_

#include <stdbool.h>
#include <stdint.h>
#include "fusion_math.h"

#define ToRad(x) radians(x)
#define ToDeg(x) degrees(x)

bool hex_to_uint8(uint8_t a, uint8_t *res);

#endif // COMMON_H_
