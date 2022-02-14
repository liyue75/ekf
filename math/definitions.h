#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <math.h>

#ifdef M_PI
# undef M_PI
#endif // M_PI

#define M_PI  (3.141592653589793)

#ifdef M_PI_2
#undef M_PI_2
#endif // M_PI_2

#define M_PI_2 (M_PI / 2)

#define M_GOLDEN 1.6180339f

#define M_2PI (M_PI * 2)

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#define GRAVITY_MSS 9.80665f

#define LATLON_TO_M 0.011131884502145034
#define LATLON_TO_M_INV 89.83204953368922
#define LATLON_TO_CM 1.1131884502145034

#endif // DEFINITIONS_H_
