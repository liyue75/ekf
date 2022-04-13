#ifndef VECTOR3F_H_
#define VECTOR3F_H_
#include <stdbool.h>

#include "rotation.h"

typedef struct vector3f {
    float x;
    float y;
    float z;
} vector3f_t;

void rotate_vec(vector3f_t *accel, Rotation_t rotation);

void v3f_zero(vector3f_t *accel);
bool v3f_is_zero(const vector3f_t *v);

float v3f_dot_product(const vector3f_t *v1, const vector3f_t *v2);
vector3f_t v3f_cross_product(const vector3f_t *v1, const vector3f_t *v2);

vector3f_t v3f_uniform_scale(const vector3f_t *v, const float num);
vector3f_t v3f_div(const vector3f_t *v, const float num);

vector3f_t v3f_add(const vector3f_t *v1, const vector3f_t *v2);
vector3f_t v3f_sub(const vector3f_t *v1, const vector3f_t *v2);
bool v3f_isnan(const vector3f_t *v3f);

bool v3f_isinf(const vector3f_t *v3f);

float v3f_length(const vector3f_t *v);
vector3f_t v3f_normalized(const vector3f_t *v);
bool v3f_not_equal(const vector3f_t *v1, const vector3f_t *v2);
#endif // VECTOR3F_H_
