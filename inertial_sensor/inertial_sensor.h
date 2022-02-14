#ifndef INERTIAL_SENSOR_H_
#define INERTIAL_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "vector3f.h"

void ins_init(uint16_t loop_rate);
bool inertial_sensor_init(void);

void set_accel_peak_hold(const vector3f_t *accel);
void calc_vibration_and_clipping(vector3f_t *accel, float dt);

bool register_accel(uint16_t raw_sample_rate_hz);
bool register_gyro(uint16_t raw_sample_rate_hz);
void notify_gyro_fifo_reset(void);
void notify_accel_fifo_reset(void);
uint16_t get_ins_loop_rate_hz(void);

void set_accel_oversampling(uint8_t n);
void set_gyro_oversampling(uint8_t n);
void set_raw_sample_accel_multiplier(uint16_t mul);
float get_loop_delta_t(void);

uint32_t get_last_update_usec(void);
vector3f_t* get_gyro(void);
vector3f_t* get_accel(void);
bool get_accel_health(void);
bool get_gyro_health(void);
bool get_delta_velocity(vector3f_t *delta_velocity, float *delta_velocity_dt);
bool get_delta_angle(vector3f_t *delta_angle, float *delta_angle_dt);
vector3f_t *get_imu_pos_offset(void);
#endif // INERTIAL_SENSOR_H_
