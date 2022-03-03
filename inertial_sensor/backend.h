#ifndef BACKEND_H_
#define BACKEND_H_

#include <stdint.h>
#include "vector3f.h"
#include "rotation.h"

void imu_backend_init(void);
uint16_t get_ins_loop_rate_hz(void);
uint8_t get_fast_sampling_rate(void);
void rotate_and_correct_accel(vector3f_t *accel, Rotation_t orientation);
void rotate_and_correct_gyro(vector3f_t *gyro, Rotation_t orientation);
void set_gyro_accel_orientation(Rotation_t *accel_orientation,
                                Rotation_t *gyro_orientation,
                                Rotation_t rotation);

void notify_new_accel_raw_sample(vector3f_t *accel, uint64_t sample_us, bool fsync_set);
void notify_new_gyro_raw_sample(vector3f_t *gyro, uint64_t sample_us);
void backend_update_accel(void);
void backend_update_gyro(void);
void backend_publish_temperature(float temperature);
#endif // BACKEND_H_
