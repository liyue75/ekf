#ifndef EKF3_H_
#define EKF3_H_

#include <stdbool.h>
#include <stdint.h>
#include "vector2f.h"
#include "location.h"
#include "vector3f.h"
#include "matrix3f.h"

#define MASK_GPS_NSATS (1<<0)
#define MASK_GPS_HDOP (1<<1)
#define MASK_GPS_SPD_ERR (1<<2)
#define MASK_GPS_POS_ERR (1<<3)
#define MASK_GPS_YAW_ERR (1<<4)
#define MASK_GPS_POS_DRIFT (1<<5)
#define MASK_GPS_VERT_SPD (1<<6)
#define MASK_GPS_HORIZ_SPD (1<<7)

typedef struct {
    uint32_t last_function_call;
    bool core_changed;
    uint32_t last_primary_change;
    float core_delta;
} yaw_reset_data_t;

typedef struct {
    uint32_t last_function_call;
    bool core_changed;
    uint32_t last_primary_change;
    vector2f_t core_delta;         //NE position change
} pos_reset_data_t;

typedef struct {
    uint32_t last_function_call;
    bool core_changed;
    uint32_t last_primary_change;
    float core_delta;        //D position change
} pos_down_reset_data_t;

void set_ekf_enable(bool enable);

void ekf3_init(void);
bool ekf3_initialise_filter(void);
bool ekf3_get_llh(location_t *loc);
void ekf3_reset_gyro_bias(void);
bool ekf3_get_origin_llh(location_t *loc);
void ekf3_update_filter(void);
void ekf3_get_rotation_body_to_ned(matrix3f_t *mat);
void ekf3_get_euler_angles(vector3f_t *euler);
void ekf3_get_vel_ned(vector3f_t *vel);
#endif // EKF3_H_
