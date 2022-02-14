#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <stdbool.h>
#include "matrix3f.h"

typedef struct quaternionf {
    float q1;
    float q2;
    float q3;
    float q4;
} quaternionf_t;

bool quat_isnan(const quaternionf_t *q);
void quat_to_rotation_matrix(const quaternionf_t *q, matrix3f_t *m);
void quat_from_rotation_matrix(quaternionf_t *q, const matrix3f_t *m);
void quat_from_rotation(quaternionf_t *q, Rotation_t rotation);
float quat_length(const quaternionf_t *q);
quaternionf_t quat_inverse(const quaternionf_t *q);
void quat_invert(quaternionf_t *q);
void quat_normalize(quaternionf_t *q);
quaternionf_t quat_multi(const quaternionf_t *q, const quaternionf_t *v);
vector3f_t quat_multi_v(const quaternionf_t *q, const vector3f_t *v);
quaternionf_t quat_div(const quaternionf_t *q, const quaternionf_t *v);
quaternionf_t quat_angular_difference(const quaternionf_t *q, const quaternionf_t *v);
float quat_roll_pitch_difference(const quaternionf_t *q, const quaternionf_t *v);
void quat_rotate(quaternionf_t *q, Rotation_t rotation);
void quat_earth_to_body(const quaternionf_t *q, vector3f_t *v);
void quat_from_euler(quaternionf_t *q, const float roll, const float pitch, const float yaw);
void quat_from_euler_v3f(quaternionf_t *q, const vector3f_t *v);
void quat_from_vector312(quaternionf_t *q, const float roll, const float pitch, const float yaw);
void quat_from_axis_angle(quaternionf_t *q, const vector3f_t *axis, float theta);
void quat_from_axis_angle_v3f(quaternionf_t *q, vector3f_t *v);
void quat_rotate_v(quaternionf_t *q, vector3f_t *v);
void quat_to_axis_angle(const quaternionf_t *q, vector3f_t *v);
void quat_from_axis_angle_fast(quaternionf_t *q, const vector3f_t *axis, float theta);
void quat_from_axis_angle_fast_v3f(quaternionf_t *q, vector3f_t *v);
void quat_rotate_v_fast(quaternionf_t *q, const vector3f_t *v);
float quat_get_euler_roll(const quaternionf_t *q);
float quat_get_euler_pitch(const quaternionf_t *q);
float quat_get_euler_yaw(const quaternionf_t *q);
void quat_to_euler(const quaternionf_t *q, float *roll, float *pitch, float *yaw);
vector3f_t quat_to_vector312(const quaternionf_t *q);
#endif // QUATERNION_H_
