#include <math.h>
#include "quaternion.h"
#include "matrix3f.h"
#include "fusion_math.h"
#include "rotation.h"
#include "board_led.h"
#include "uart_device.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define HALF_SQRT_2_PlUS_SQRT_2 0.92387953251128673848313610506011 // sqrt(2 + sqrt(2)) / 2
#define HALF_SQRT_2_MINUS_SQTR_2 0.38268343236508972626808144923416
#define HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO 0.65328148243818828788676000084096 // sqrt((2 + sqrt(2))/2) /2
#define HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO 0.27059805007309845059637609665515

bool quat_isnan(const quaternionf_t *q)
{
    return isnan(q->q1) || isnan(q->q1) || isnan(q->q2) || isnan(q->q3);
}

void quat_to_rotation_matrix(const quaternionf_t *q, matrix3f_t *m)
{
    const float q3q3 = q->q3 * q->q3;
    const float q3q4 = q->q3 * q->q4;
    const float q2q2 = q->q2 * q->q2;
    const float q2q3 = q->q2 * q->q3;
    const float q2q4 = q->q2 * q->q4;
    const float q1q2 = q->q1 * q->q2;
    const float q1q3 = q->q1 * q->q3;
    const float q1q4 = q->q1 * q->q4;
    const float q4q4 = q->q4 * q->q4;

    m->a.x = 1.0f - 2.0f * (q3q3 + q4q4);
    m->a.y = 2.0f * (q2q3 - q1q4);
    m->a.z = 2.0f * (q2q4 + q1q3);
    m->b.x = 2.0f * (q2q3 + q1q4);
    m->b.y = 1.0f - 2.0f * (q2q2 + q4q4);
    m->b.z = 2.0f * (q3q4 - q1q2);
    m->c.x = 2.0f * (q2q4 - q1q3);
    m->c.y = 2.0f * (q3q4 + q1q2);
    m->c.z = 1.0f - 2.0f * (q2q2 + q3q3);
}

void quat_from_rotation_matrix(quaternionf_t *q, const matrix3f_t *m)
{
    const float *m00 = &m->a.x;
    const float *m11 = &m->b.y;
    const float *m22 = &m->c.z;
    const float *m10 = &m->b.x;
    const float *m01 = &m->a.y;
    const float *m20 = &m->c.x;
    const float *m02 = &m->a.z;
    const float *m21 = &m->c.y;
    const float *m12 = &m->b.z;
    float *qw = &q->q1;
    float *qx = &q->q2;
    float *qy = &q->q3;
    float *qz = &q->q4;
    const float tr = *m00 + *m11 + *m22;
    if (tr > 0) {
        const float S = sqrtf(tr + 1) * 2;
        *qw = 0.25f * S;
        *qx = (*m21 - *m12) / S;
        *qy = (*m02 - *m20) / S;
        *qz = (*m10 - *m01) / S;
    } else if ((*m00 > *m11) && (*m00 > *m22)) {
        const float S = sqrtf(1.0f + *m00 - *m11 - *m22) * 2.0f;
        *qw = (*m21 - *m12) / S;
        *qx = 0.25f * S;
        *qy = (*m01 + *m10) / S;
        *qz = (*m02 + *m20) / S;
    } else if (*m11 > *m22){
        const float S = sqrtf(1.0f + *m11 - *m00 - *m22) * 2.0f;
        *qw = (*m02 - *m20) / S;
        *qx = (*m01 + *m10) / S;
        *qy = 0.25f * S;
        *qz = (*m12 + *m21) / S;
    } else {
        const float S = sqrtf(1.f + *m22 - *m00 - *m11) * 2.0f;
        *qw = (*m10 - *m01) / S;
        *qx = (*m02 + *m20) / S;
        *qy = (*m12 + *m21) / S;
        *qz = 0.25f * S;
    }
}

void quat_from_rotation(quaternionf_t *q, Rotation_t rotation)
{
    // the constants below can be calculated using the following formula:
    //     Matrix3f m_from_rot;
    //     m_from_rot.from_rotation(rotation);
    //     Quaternion q_from_m;
    //     from_rotation_matrix(m_from_rot);

    switch (rotation) {
    case ROTATION_NONE:
        q->q1 = 1;
        q->q2 = q->q3 = q->q4 = 0;
        return;

    case ROTATION_YAW_45:
        q->q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q2 = q->q3 = 0;
        q->q4 = HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_YAW_90:
        q->q1 = HALF_SQRT_2;
        q->q2 = q->q3 = 0;
        q->q4 = HALF_SQRT_2;
        return;

    case ROTATION_YAW_135:
        q->q1 = HALF_SQRT_2_MINUS_SQTR_2;
        q->q2 = q->q3 = 0;
        q->q4 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_YAW_180:
        q->q1 = q->q2 = q->q3 = 0;
        q->q4=1;
        return;

    case ROTATION_YAW_225:
        q->q1 = -HALF_SQRT_2_MINUS_SQTR_2;
        q->q2 = q->q3 = 0;
        q->q4 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_YAW_270:
        q->q1 = HALF_SQRT_2;
        q->q2 = q->q3 = 0;
        q->q4 = -HALF_SQRT_2;
        return;

    case ROTATION_YAW_315:
        q->q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q2 = q->q3 = 0;
        q->q4 = -HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_180:
        q->q1 = q->q3 = q->q4 = 0;
        q->q2 = 1;
        return;

    case ROTATION_ROLL_180_YAW_45:
        q->q1 = q->q4 = 0;
        q->q2 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q3 = HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_180_YAW_90:
    case ROTATION_PITCH_180_YAW_270:
        q->q1 = q->q4 = 0;
        q->q2 = q->q3 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_180_YAW_135:
        q->q1 = q->q4 = 0;
        q->q2 = HALF_SQRT_2_MINUS_SQTR_2;
        q->q3 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_PITCH_180:
        q->q1 = q->q2 = q->q4 = 0;
        q->q3 = 1;
        return;

    case ROTATION_ROLL_180_YAW_225:
        q->q1 = q->q4 = 0;
        q->q2 = -HALF_SQRT_2_MINUS_SQTR_2;
        q->q3 = HALF_SQRT_2_PlUS_SQRT_2;
        return;

    case ROTATION_ROLL_180_YAW_270:
    case ROTATION_PITCH_180_YAW_90:
        q->q1 = q->q4 = 0;
        q->q2 = -HALF_SQRT_2;
        q->q3 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_180_YAW_315:
        q->q1 = q->q4 = 0;
        q->q2 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q3 = -HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_90:
        q->q1 = q->q2 = HALF_SQRT_2;
        q->q3 = q->q4 = 0;
        return;

    case ROTATION_ROLL_90_YAW_45:
        q->q1 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q2 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q3 = q->q4 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        return;

    case ROTATION_ROLL_90_YAW_90:
        q->q1 = q->q2 = q->q3 = q->q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_YAW_135:
        q->q1 = q->q2 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q->q3 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q4 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        return;

    case ROTATION_ROLL_270:
        q->q1 = HALF_SQRT_2;
        q->q2 = -HALF_SQRT_2;
        q->q3 = q->q4 = 0;
        return;

    case ROTATION_ROLL_270_YAW_45:
        q->q1 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q2 = -HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q3 = -HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q->q4 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        return;

    case ROTATION_ROLL_270_YAW_90:
        q->q1 = q->q4 = 0.5f;
        q->q2 = q->q3 = -0.5f;
        return;

    case ROTATION_ROLL_270_YAW_135:
        q->q1 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q->q2 = -HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q->q3 = -HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q4 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        return;

    case ROTATION_PITCH_90:
        q->q1 = q->q3 = HALF_SQRT_2;
        q->q2 = q->q4 = 0;
        return;

    case ROTATION_PITCH_270:
        q->q1 = HALF_SQRT_2;
        q->q2 = q->q4 = 0;
        q->q3 = -HALF_SQRT_2;
        return;

    case ROTATION_ROLL_90_PITCH_90:
        q->q1 = q->q2 = q->q3 = -0.5f;
        q->q4 = 0.5f;
        return;

    case ROTATION_ROLL_180_PITCH_90:
        q->q1 = q->q3 = 0;
        q->q2 = -HALF_SQRT_2;
        q->q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_270_PITCH_90:
        q->q1 = q->q3 = q->q4 = 0.5f;
        q->q2 = -0.5f;
        return;

    case ROTATION_ROLL_90_PITCH_180:
        q->q1 = q->q2 = 0;
        q->q3 = -HALF_SQRT_2;
        q->q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_270_PITCH_180:
        q->q1 = q->q2 = 0;
        q->q3 = q->q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_90_PITCH_270:
        q->q1 = q->q2 = q->q4 = 0.5f;
        q->q3 = -0.5;
        return;

    case ROTATION_ROLL_180_PITCH_270:
        q->q1 = q->q3 = 0;
        q->q2 = q->q4 = HALF_SQRT_2;
        return;

    case ROTATION_ROLL_270_PITCH_270:
        q->q1 = -0.5f;
        q->q2 = q->q3 = q->q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_PITCH_180_YAW_90:
        q->q1 = q->q3 = -0.5f;
        q->q2 = q->q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_YAW_270:
        q->q1 = q->q2 = -0.5f;
        q->q3 = q->q4 = 0.5f;
        return;

    case ROTATION_ROLL_90_PITCH_68_YAW_293:
        q->q1 = 0.26774500501681575137524760066299;
        q->q2 = 0.70698804688952421315661922562867;
        q->q3 = 0.012957683254962659713527273197542;
        q->q4 = -0.65445596665363614530264158020145;
        return;

    case ROTATION_PITCH_315:
        q->q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q2 = q->q4 = 0;
        q->q3 = -HALF_SQRT_2_MINUS_SQTR_2;
        return;

    case ROTATION_ROLL_90_PITCH_315:
        q->q1 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q2 = HALF_SQRT_HALF_TIMES_TWO_PLUS_SQRT_TWO;
        q->q3 = -HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        q->q4 = HALF_SQRT_HALF_TIMES_TWO_MINUS_SQRT_TWO;
        return;

    case ROTATION_PITCH_7:
        q->q1 = 0.99813479842186692003735970502021;
        q->q2 = q->q4 = 0;
        q->q3 = 0.061048539534856872956769535676358;
        return;

    case ROTATION_ROLL_45:
        q->q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q2 = HALF_SQRT_2_MINUS_SQTR_2;
        q->q3 = q->q4 = 0.0;
        return;

    case ROTATION_ROLL_315:
        q->q1 = HALF_SQRT_2_PlUS_SQRT_2;
        q->q2 = -HALF_SQRT_2_MINUS_SQTR_2;
        q->q3 = q->q4 = 0.0;
        return;

    case ROTATION_CUSTOM:
        break;

    case ROTATION_MAX:
        break;
    }
    // rotation invalid
    DEBUG("Bad rotation\n");
    led_on(LED_1);
}

float quat_length(const quaternionf_t *q)
{
    return sqrtf(sq(q->q1) + sq(q->q2) + sq(q->q3) + sq(q->q4));
}

quaternionf_t quat_inverse(const quaternionf_t *q)
{
    quaternionf_t tmp = {
    q->q1, -q->q2, -q->q3, -q->q4
};
    return tmp;
}

void quat_invert(quaternionf_t *q)
{
    q->q2 = -q->q2;
    q->q3 = -q->q3;
    q->q4 = -q->q4;
}

void quat_normalize(quaternionf_t *q)
{
    const float quat_mag = quat_length(q);
    if (!float_is_zero(quat_mag)) {
        const float quat_mag_inv = 1.0f / quat_mag;
        q->q1 *= quat_mag_inv;
        q->q2 *= quat_mag_inv;
        q->q3 *= quat_mag_inv;
        q->q4 *= quat_mag_inv;
    } else {
        MY_LOG("quat length is zero\n");
        //led_on(LED_3);
    }
}

quaternionf_t quat_multi(const quaternionf_t *q, const quaternionf_t *v)
{
    quaternionf_t ret = {};
    const float w1 = q->q1;
    const float x1 = q->q2;
    const float y1 = q->q3;
    const float z1 = q->q4;

    const float w2 = v->q1;
    const float x2 = v->q2;
    const float y2 = v->q3;
    const float z2 = v->q4;

    ret.q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    ret.q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    ret.q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    ret.q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    return ret;

}

vector3f_t quat_multi_v(const quaternionf_t *q, const vector3f_t *v)
{
    // This uses the formula
    //
    //    v2 = v1 + 2 q1 * qv x v1 + 2 qv x qv x v1
    //
    // where "x" is the cross product (explicitly inlined for performance below),
    // "q1" is the scalar part and "qv" is the vector part of this quaternion

    vector3f_t ret = *v;

    // Compute and cache "qv x v1"
    float uv[] = {q->q3 * v->z - q->q4 * v->y,
    q->q4 * v->x - q->q2 * v->z,
    q->q2 * v->y - q->q3 * v->x};

    uv[0] += uv[0];
    uv[1] += uv[1];
    uv[2] += uv[2];
    ret.x += q->q1 * uv[0] + q->q3 * uv[2] - q->q4 * uv[1];
    ret.y += q->q1 * uv[1] + q->q4 * uv[0] - q->q2 * uv[2];
    ret.z += q->q1 * uv[2] + q->q2 * uv[1] - q->q3 * uv[0];
    return ret;
}

quaternionf_t quat_div(const quaternionf_t *q, const quaternionf_t *v)
{
    quaternionf_t ret;
    const float quat0 = q->q1;
    const float quat1 = q->q2;
    const float quat2 = q->q3;
    const float quat3 = q->q4;

    const float quatMag = quat_length(q);
    if (float_is_zero(quatMag)) {
        // floathe code goes here if the quaternion is [0,0,0,0]. This shouldn't happen.
        led_on(LED_1);
    }

    const float rquat0 = v->q1;
    const float rquat1 = v->q2;
    const float rquat2 = v->q3;
    const float rquat3 = v->q4;

    ret.q1 = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3);
    ret.q2 = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2);
    ret.q3 = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1);
    ret.q4 = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0);
    return ret;
}

quaternionf_t quat_angular_difference(const quaternionf_t *q, const quaternionf_t *v)
{
    quaternionf_t tmp = quat_inverse(v);
    return quat_multi(&tmp, q);
}

float quat_roll_pitch_difference(const quaternionf_t *q, const quaternionf_t *v)
{
    // convert Quaternions to rotation matrices
    matrix3f_t m, vm;
    quat_to_rotation_matrix(q, &m);
    quat_to_rotation_matrix(v, &vm);

    // rotate earth frame vertical vector by each rotation matrix
    const vector3f_t z_unit_vec = {0,0,1};
    const vector3f_t z_unit_m = m3f_mul_transpose(&m, &z_unit_vec);
    const vector3f_t z_unit_vm = m3f_mul_transpose(&vm, &z_unit_vec);
    const vector3f_t vec_diff = v3f_sub(&z_unit_vm, &z_unit_m);
    const float vec_len_div2 = constrain_float(v3f_length(&vec_diff) * 0.5, 0.0, 1.0);

    // calculate and return angular difference
    return (2.0 * asinf(vec_len_div2));
}

void quat_rotate(quaternionf_t *q, Rotation_t rotation)
{
    quaternionf_t q_from_rot;
    quat_from_rotation(&q_from_rot, rotation);
    *q = quat_multi(q, &q_from_rot);
}

void quat_earth_to_body(const quaternionf_t *q, vector3f_t *v)
{
    matrix3f_t m;
    quat_to_rotation_matrix(q, &m);
    *v = m3f_multi_v(&m, v);
}

void quat_from_euler(quaternionf_t *q, const float roll, const float pitch, const float yaw)
{
    const float cr2 = cosf(roll*0.5);
    const float cp2 = cosf(pitch*0.5);
    const float cy2 = cosf(yaw*0.5);
    const float sr2 = sinf(roll*0.5);
    const float sp2 = sinf(pitch*0.5);
    const float sy2 = sinf(yaw*0.5);

    q->q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q->q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q->q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q->q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

void quat_from_euler_v3f(quaternionf_t *q, const vector3f_t *v)
{
    quat_from_euler(q, v->x, v->y, v->z);
}

void quat_from_vector312(quaternionf_t *q, const float roll,
                         const float pitch, const float yaw)
{
    matrix3f_t m;
    m3f_from_euler312(&m, roll, pitch, yaw);
    quat_from_rotation_matrix(q, &m);
}

void quat_from_axis_angle(quaternionf_t *q, const vector3f_t *axis, float theta)
{
    if (float_is_zero(theta)) {
        q->q1 = 1.0f;
        q->q2 = q->q3 = q->q4 = 0.0f;
        return;
    }
    const float st2 = sinf(0.5 * theta);
    q->q1 = cosf(0.5 * theta);
    q->q2 = axis->x * st2;
    q->q3 = axis->y * st2;
    q->q4 = axis->z * st2;
}

void quat_from_axis_angle_v3f(quaternionf_t *q, vector3f_t v)
{
    const float theta = v3f_length(&v);
    if (float_is_zero(theta)) {
        q->q1 = 1.0f;
        q->q2 = q->q3 = q->q4 = 0.0f;
        return;
    }
    v = v3f_div(&v, theta);
    quat_from_axis_angle(q, &v, theta);
}

void quat_rotate_v(quaternionf_t *q, vector3f_t *v)
{
    quaternionf_t r;
    quat_from_axis_angle_v3f(&r, *v);
    *q = quat_multi(q, &r);
}

void quat_to_axis_angle(const quaternionf_t *q, vector3f_t *v)
{
    const float l = sqrtf(sq(q->q2) + sq(q->q3) + sq(q->q4));
    vector3f_t tmp = {q->q2, q->q3, q->q4};
    *v = tmp;
    if (!float_is_zero(l)) {
        *v = v3f_div(v, l);
        *v = v3f_uniform_scale(v, wrap_PI(2.0f * atan2f(l, q->q1)));
    }
}

void quat_from_axis_angle_fast(quaternionf_t *q, const vector3f_t *axis, float theta)
{
    const float t2 = 0.5 * theta;
    const float sqt2 = sq(t2);
    const float st2 = t2 - sqt2 * t2 / 6.0f;
    q->q1 = 1.0f - (0.5 * sqt2) + sq(sqt2) / 24.0f;
    q->q2 = axis->x * st2;
    q->q3 = axis->y * st2;
    q->q4 = axis->z * st2;
}

void quat_from_axis_angle_fast_v3f(quaternionf_t *q, vector3f_t *v)
{
    const float theta = v3f_length(v);
    if (float_is_zero(theta)) {
        q->q1 = 1.0f;
        q->q2 = q->q3 = q->q4 = 0.0f;
        return;
    }
    *v = v3f_div(v, theta);
    quat_from_axis_angle_fast(q, v, theta);
}

void quat_rotate_v_fast(quaternionf_t *q, const vector3f_t *v)
{
    const float theta = v3f_length(v);
    if (float_is_zero(theta)) {
        return;
    }
    const float t2 = 0.5 * theta;
    const float sqt2 = sq(t2);
    float st2 = t2 - sqt2 * t2 / 6.0f;
    st2 /= theta;

    const float w2 = 1.0f - (0.5*sqt2) + sq(sqt2) / 24.0f;
    const float x2 = v->x * st2;
    const float y2 = v->y * st2;
    const float z2 = v->z * st2;
    const float w1 = q->q1;
    const float x1 = q->q2;
    const float y1 = q->q3;
    const float z1 = q->q4;
    q->q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q->q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q->q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q->q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

}

float quat_get_euler_roll(const quaternionf_t *q)
{
    return (atan2f(2.0f*(q->q1*q->q2 + q->q3*q->q4), 1.0f - 2.0f*(q->q2*q->q2 + q->q3*q->q3)));
}

float quat_get_euler_pitch(const quaternionf_t *q)
{
    return safe_asin(2.0f*(q->q1*q->q3 - q->q4*q->q2));
}

float quat_get_euler_yaw(const quaternionf_t *q)
{
    return atan2f(2.0f*(q->q1*q->q4 + q->q2*q->q3), 1.0f - 2.0f*(q->q3*q->q3 + q->q4*q->q4));
}

void quat_to_euler(const quaternionf_t *q, float *roll, float *pitch, float *yaw)
{
    *roll = quat_get_euler_roll(q);
    *pitch = quat_get_euler_pitch(q);
    *yaw = quat_get_euler_yaw(q);
}

vector3f_t quat_to_vector312(const quaternionf_t *q)
{
    matrix3f_t m;
    quat_to_rotation_matrix(q, &m);
    return m3f_to_euler312(&m);
}

void quat_initialise(quaternionf_t *q)
{
    q->q1 = 1;
    q->q2 = q->q3 = q->q4 = 0;
}

float quat_idx(const quaternionf_t *q, uint8_t idx)
{
    const float *_v = &(q->q1);
    return _v[idx];
}
