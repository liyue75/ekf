#ifndef EKF_COURCE_H_
#define EKF_COURCE_H_

#include <stdint.h>
#include <stdbool.h>

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

typedef struct {
    source_xy_t posxy;
    source_xy_t velxy;
    source_z_t posz;
    source_z_t velz;
    source_yaw_t yaw;
} source_set_t;

bool ekf_use_vel_z_source(source_z_t velz_source);
bool ekf_have_vel_z_source(void);
source_xy_t ekf_source_get_posxy_source(void);
source_z_t ekf_source_get_posz_source(void);
source_yaw_t ekf_source_get_yaw_source(void);
bool ekf_source_use_vel_xy_source(source_xy_t velxy_source);

#endif // EKF_COURCE_H_
