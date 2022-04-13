#include <math.h>
#include "ahrs.h"
#include "ahrs_backend.h"
#include "vector2f.h"
#include "fusion_math.h"

static float cos_roll = 1.0f;
static float cos_pitch = 1.0f;
static float cos_yaw = 1.0f;
static float sin_roll = 0;
static float sin_pitch = 0;
static float sin_yaw = 0;

static void calc_trig(const matrix3f_t *rot,
                      float *cr, float *cp, float *cy,
                      float *sr, float *sp, float *sy)
{
    vector2f_t yaw_vector = {rot->a.x, rot->b.x};
    if (fabsf(yaw_vector.x) > 0 ||
        fabsf(yaw_vector.y) > 0) {
        yaw_vector = v2f_normalized(&yaw_vector);
    }
    *sy = constrain_float(yaw_vector.y, -1.0, 1.0);
    *cy = constrain_float(yaw_vector.x, -1.0, 1.0);
    if (isinf(yaw_vector.x) || isinf(yaw_vector.y) ||
    isnan(yaw_vector.x) || isnan(yaw_vector.y)) {
        *sy = 0;
        *cy = 1.0f;
    }
    const float cx2 = rot->c.x * rot->c.x;
    if (cx2 >= 1.0f) {
        *cp = 0;
        *cr = 1.0f;
    } else {
        *cp = safe_sqrt(1 - cx2);
        *cr = rot->c.z / *cp;
    }
    *cp = constrain_float(*cp, 0, 1);
    *cr = constrain_float(*cr, 0, 1);
    *sp = -rot->c.x;
    if (!float_is_zero(*cp)) {
        *sr = rot->c.y / *cp;
    }
    if (float_is_zero(*cp) || isinf(*cr) || isnan(*cr) || isinf(*sr) || isnan(*sr)) {
        float r, p, y;
        m3f_to_euler(rot, &r, &p, &y);
        *cr = cosf(r);
        *sr = sinf(r);
    }
}


void ahrs_backend_update_trig(void)
{
    calc_trig(ahrs_get_rotation_body_to_ned(),
              &cos_roll, &cos_pitch, &cos_yaw,
              &sin_roll, &sin_pitch, &sin_yaw);
}
