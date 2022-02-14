#ifndef MATRIX3F_H_
#define MATRIX3F_H_

#include "vector2f.h"
#include "vector3f.h"
#include "rotation.h"

typedef struct matrix3f {
    vector3f_t a;
    vector3f_t b;
    vector3f_t c;
} matrix3f_t;


matrix3f_t m3f_add(const matrix3f_t *m1, const matrix3f_t *m2);

void m3f_from_euler(matrix3f_t *m, float roll, float pitch, float yaw);

void m3f_to_euler(const matrix3f_t *m, float *roll, float *pitch, float *yaw);

matrix3f_t m3f_transpose(const matrix3f_t *m);

void m3f_from_rotation(matrix3f_t *m, Rotation_t rotation);

vector3f_t m3f_to_euler312(const matrix3f_t *m);

void m3f_from_euler312(matrix3f_t *m, float roll, float pitch, float yaw);

void m3f_rotate(matrix3f_t *m, const vector3f_t g);

vector3f_t m3f_multi_v(const matrix3f_t *m, const vector3f_t *v);
matrix3f_t m3f_multi_m(const matrix3f_t *m1, const matrix3f_t *m2);

vector2f_t m3f_mulXY(const matrix3f_t *m, const vector3f_t *v);
vector3f_t m3f_mul_transpose(const matrix3f_t *m, const vector3f_t *v);

float m3f_det(const matrix3f_t *m);
bool m3f_inverse(matrix3f_t *m, matrix3f_t *inv);
bool m3f_invert(matrix3f_t *m);

void m3f_zero(matrix3f_t *m);

matrix3f_t m3f_from_axis_angle(const vector3f_t *v, float theta);

#endif // MATRIX3F_H_
