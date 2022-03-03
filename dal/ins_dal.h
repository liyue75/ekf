#ifndef INS_DAL_H_
#define INS_DAL_H_

#include <stdint.h>
#include "vector3f.h"

void dal_ins_start_frame(void);
void dal_ins_update_filtered(void);

vector3f_t dal_ins_get_gyro(void);
vector3f_t dal_ins_get_accel(void);

bool dal_ins_get_delta_velocity(vector3f_t *delta_velocity, float *delta_velocity_dt);
bool dal_ins_get_delta_angle(vector3f_t *delta_angle, float *delta_angle_dt);
float dal_ins_get_loop_delta_t(void);
vector3f_t dal_ins_get_imu_pos_offset(void);
uint16_t dal_ins_get_loop_rate_hz(void);
#endif // INS_DAL_H_
