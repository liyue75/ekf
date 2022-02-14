#ifndef EKF_COURCE_H_
#define EKF_COURCE_H_

#include <stdint.h>

typedef enum {
XY_NONE = 0,
XY_GPS = 3,
XY_BEACON = 4,
XY_OPTFLOW = 5,
XY_EXTNAV = 6,
XY_WHEEL_ENCODER = 7
} source_xy_t;

typedef enum {
Z_NONE = 0,
Z_BARO = 1,
Z_RANGEFINDER = 2,
Z_GPS = 3,
Z_BEACON = 4,
Z_EXTNAV = 6
} source_z_t;

typedef enum {
YAW_NONE = 0,
YAW_COMPASS = 1,
YAW_GPS = 2,
YAW_GPS_COMPASS_FALLBACK = 3,
YAW_EXTNAV = 6,
YAW_GSF = 8
} source_yaw_t;


#endif // EKF_COURCE_H_
