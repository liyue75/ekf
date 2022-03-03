#ifndef NAV_COMMON_H_
#define NAV_COMMON_H_
#include <stdint.h>
#include <stdbool.h>
typedef union {
    struct {
        bool attitude:1;
        bool horiz_vel:1;
        bool vert_vel:1;
        bool horiz_pos_rel:1;
        bool horiz_pos_abs:1;
        bool vert_pos:1;
        bool terrain_alt:1;
        bool const_pos_mode:1;
        bool pred_horiz_pos_rel:1;
        bool pred_horiz_pos_abs:1;
        bool takeoff_detected:1;
        bool takeoff:1;
        bool touchdown:1;
        bool using_gps:1;
        bool gps_glitching:1;
        bool gps_quality_good:1;
        bool initialized:1;
    } flags;
    uint32_t value;
} nav_filter_status_t;

typedef union {
    struct {
        bool bad_sacc:1;
        bool bad_hacc:1;
        bool bad_yaw:1;
        bool bad_sats:1;
        bool bad_vz:1;
        bool bad_horiz_drift:1;
        bool bad_hdop:1;
        bool bad_vert_vel:1;
        bool bad_fix:1;
        bool bad_horiz_vel:1;
        bool bad_vacc:1;
    } flags;
    uint16_t value;
} nav_gps_status_t;


typedef struct {
    uint32_t count;
    float dt_imu_avg_min;
    float dt_imu_avg_max;
    float dt_ekf_avg_min;
    float dt_ekf_avg_max;
    float del_ang_dt_max;
    float del_ang_dt_min;
    float del_vel_dt_max;
    float del_vel_dt_min;
} ekf_timing_t;

#endif // NAV_COMMON_H_
