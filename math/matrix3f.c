#include <stddef.h>
#include <math.h>
#include "matrix3f.h"
#include "fusion_math.h"
//body to ned
void m3f_from_euler(matrix3f_t *m, float roll, float pitch, float yaw)
{
    const float cp = cosf(pitch);
    const float sp = sinf(pitch);
    const float sr = sinf(roll);
    const float cr = cosf(roll);
    const float sy = sinf(yaw);
    const float cy = cosf(yaw);
    m->a.x = cp * cy;
    m->a.y = (sr * sp * cy) - (cr * sy);
    m->a.z = (cr * sp * cy) + (sr * sy);
    m->b.x = cp * sy;
    m->b.y = (sr * sp * sy) + (cr * cy);
    m->b.z = (cr * sp * sy) - (sr * cy);
    m->c.x = -sp;
    m->c.y = sr * cp;
    m->c.z = cr * cp;
}

void m3f_to_euler(const matrix3f_t *m, float *roll, float *pitch, float *yaw)
{
    if (pitch != NULL) {
        *pitch = -safe_asin(m->c.x);
    }
    if (roll != NULL) {
        *roll = atan2f(m->c.y, m->c.z);
    }
    if (yaw != NULL) {
        *yaw = atan2f(m->b.x, m->a.x);
    }
}

matrix3f_t m3f_transposed(const matrix3f_t *m)
{
    matrix3f_t temp = {
    {m->a.x, m->b.x, m->c.x},
    {m->a.y, m->b.y, m->c.y},
    {m->a.z, m->b.z, m->c.z}
};
    return temp;
}

void m3f_from_rotation(matrix3f_t *m, Rotation_t rotation)
{
    vector3f_t a = {1, 0, 0};
    vector3f_t b = {0, 1, 0};
    vector3f_t c = {0, 0, 1};
    m->a = a;
    m->b = b;
    m->c = c;
    rotate_vec(&m->a, rotation);
    rotate_vec(&m->b, rotation);
    rotate_vec(&m->c, rotation);
    *m = m3f_transposed(m);
}

vector3f_t m3f_to_euler312(const matrix3f_t *m)
{
    vector3f_t temp = {asinf(m->c.y),
    atan2f(-m->c.x, m->c.z),
    atan2f(-m->a.y, m->b.y)};
    return temp;
}

void m3f_from_euler312(matrix3f_t *m, float roll, float pitch, float yaw)
{
    const float c3 = cosf(pitch);
    const float s3 = sinf(pitch);
    const float s2 = sinf(roll);
    const float c2 = cosf(roll);
    const float s1 = sinf(yaw);
    const float c1 = cosf(yaw);
    m->a.x = c1 * c3 - s1 * s2 * s3;
    m->b.y = c1 * c2;
    m->c.z = c3 * c2;
    m->a.y = -c2 * s1;
    m->a.z = s3 * c1 + c3 * s2 * s1;
    m->b.x = c3 * s1 + s3 * s2 * c1;
    m->b.z = s1 * s3 - s2 * c1 * c3;
    m->c.x = -s3 * c2;
    m->c.y = s2;
}

matrix3f_t m3f_add(const matrix3f_t *m1, const matrix3f_t *m2)
{
    matrix3f_t temp = {
    v3f_add(&m1->a, &m2->a),
    v3f_add(&m1->b, &m2->b),
    v3f_add(&m1->c, &m2->c)
};
    return temp;
}

void m3f_rotate(matrix3f_t *m, const vector3f_t *g)
{
    matrix3f_t temp = {
    {m->a.y * g->z - m->a.z * g->y, m->a.z * g->x - m->a.x * g->z, m->a.x * g->y - m->a.y * g->x},
    {m->b.y * g->z - m->b.z * g->y, m->b.z * g->x - m->b.x * g->z, m->b.x * g->y - m->b.y * g->x},
    {m->c.y * g->z - m->c.z * g->y, m->c.z * g->x - m->c.x * g->z, m->c.x * g->y - m->c.y * g->x}
};
    *m = m3f_add(m, &temp);
}

void m3f_normalize(matrix3f_t *m)
{
    const float error = v3f_dot_product(&m->a, &m->b);
    vector3f_t ta = v3f_uniform_scale(&m->b, 0.5f * error);
    vector3f_t tb = v3f_uniform_scale(&m->a, 0.5f * error);
    const vector3f_t t0 = v3f_sub(&m->a, &ta);
    const vector3f_t t1 = v3f_sub(&m->b, &tb);
    const vector3f_t t2 = v3f_cross_product(&t0, &t1);
    vector3f_t tmp_a = v3f_uniform_scale(&t0, 1.0f / v3f_length(&t0));
    vector3f_t tmp_b = v3f_uniform_scale(&t1, 1.0f / v3f_length(&t1));
    vector3f_t tmp_c = v3f_uniform_scale(&t2, 1.0f / v3f_length(&t2));
    m->a = tmp_a;
    m->b = tmp_b;
    m->c = tmp_c;
}

vector3f_t m3f_multi_v(const matrix3f_t *m, const vector3f_t *v)
{
    vector3f_t tmp = {
    m->a.x * v->x + m->a.y * v->y + m->a.z * v->z,
    m->b.x * v->x + m->b.y * v->y + m->b.z * v->z,
    m->c.x * v->x + m->c.y * v->y + m->c.z * v->z
};
    return tmp;
}

matrix3f_t m3f_multi_m(const matrix3f_t *m1, const matrix3f_t *m2)
{
    matrix3f_t temp = {
    {m1->a.x * m2->a.x + m1->a.y * m2->b.x + m1->a.z * m2->c.x,
     m1->a.x * m2->a.y + m1->a.y * m2->b.y + m1->a.z * m2->c.y,
     m1->a.x * m2->a.z + m1->a.y * m2->b.z + m1->a.z * m2->c.z},
    {m1->b.x * m2->a.x + m1->b.y * m2->b.x + m1->b.z * m2->c.x,
     m1->b.x * m2->a.y + m1->b.y * m2->b.y + m1->b.z * m2->c.y,
     m1->b.x * m2->a.z + m1->b.y * m2->b.z + m1->b.z * m2->c.z},
    {m1->c.x * m2->a.x + m1->c.y * m2->b.x + m1->c.z * m2->c.x,
     m1->c.x * m2->a.y + m1->c.y * m2->b.y + m1->c.z * m2->c.y,
     m1->c.x * m2->a.z + m1->c.y * m2->b.z + m1->c.z * m2->c.z}
};
        return temp;
}

vector2f_t m3f_mulXY(const matrix3f_t *m, const vector3f_t *v)
{
    vector2f_t tmp = {m->a.x * v->x + m->a.y * v->y + m->a.z * v->z,
    m->b.x * v->x + m->b.y * v->y + m->b.z * v->z};
    return tmp;
}

vector3f_t m3f_mul_transpose(const matrix3f_t *m, const vector3f_t *v)
{
    vector3f_t tmp = {
    m->a.x * v->x + m->b.x * v->y + m->c.x * v->z,
    m->a.y * v->x + m->b.y * v->y + m->c.y * v->z,
    m->a.z * v->x + m->b.z * v->y + m->c.z * v->z
};
    return tmp;
}

float m3f_det(const matrix3f_t *m)
{
    return m->a.x * (m->b.y * m->c.z - m->b.z * m->c.y) +
        m->a.y * (m->b.z * m->c.x - m->b.x * m->c.x) +
        m->a.z * (m->b.x * m->c.y - m->b.y * m->c.x);
}

bool m3f_inverse(matrix3f_t *m, matrix3f_t *inv)
{
    const float d = m3f_det(m);
    if (float_is_zero(d)) {
        return false;
    }
    inv->a.x = (m->b.y * m->c.z - m->c.y * m->b.z) / d;
    inv->a.y = (m->a.z * m->c.y - m->a.y * m->c.z) / d;
    inv->a.z = (m->a.y * m->b.z - m->a.z * m->b.y) / d;
    inv->b.x = (m->b.z * m->c.x - m->b.x * m->c.z) / d;
    inv->b.y = (m->a.x * m->c.z - m->a.z * m->c.x) / d;
    inv->b.z = (m->b.x * m->a.z - m->a.x * m->b.z) / d;
    inv->c.x = (m->b.x * m->c.y - m->c.x * m->b.y) / d;
    inv->c.y = (m->c.x * m->a.y - m->a.x * m->c.y) / d;
    inv->c.z = (m->a.x * m->b.y - m->b.x * m->a.y) / d;
    return true;
}

bool m3f_invert(matrix3f_t *m)
{
    matrix3f_t inv;
    bool success = m3f_inverse(m, &inv);
    if (success) {
        *m = inv;
    }
    return success;
}

void m3f_zero(matrix3f_t *m)
{
    m->a.x = m->a.y = m->a.z = 0;
    m->b.x = m->b.y = m->b.z = 0;
    m->c.x = m->c.y = m->c.z = 0;
}

matrix3f_t m3f_from_axis_angle(__attribute__((unused))const vector3f_t *v, float theta)
{
    const float C = cosf(theta);
    const float S = sinf(theta);
    const float t = 1.0f - C;
    const vector3f_t normv = v3f_normalized(v);
    const float x = normv.x;
    const float y = normv.y;
    const float z = normv.z;

    matrix3f_t tmp = {{t * x * x + C, t * x * y - z * S, t * x * z + y * S},
                      {t * x * y + z * S, t * y * y + C, t * y * z - x * S},
                      {t * x * z - y * S, t * y * z + x * S, t * z * z + C}
};
    return tmp;
}

void m3f_identity(matrix3f_t *m)
{
    m->a.x = m->b.y = m->c.z = 1;
    m->a.y = m->a.z = 0;
    m->b.x = m->b.z = 0;
    m->c.x = m->c.y = 0;
}

vector3f_t m3f_colx(const matrix3f_t *m)
{
    vector3f_t tmp = {m->a.x, m->b.x, m->c.x};
    return tmp;
}
