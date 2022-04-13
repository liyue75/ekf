#include "ins_dal.h"
#include "dal.h"
#include "vector3f.h"
#include "inertial_sensor.h"
#include "fusion_math.h"
#include "uart_device.h"

static log_rish_t rish = {.primary_gyro = 0, .primary_accel = 0, .accel_count = 1,
.gyro_count = 1};
static log_risi_t risi = {};
__attribute__((unused))static float alpha;
__attribute__((unused))static vector3f_t pos = {0, 0, 0};
__attribute__((unused))static vector3f_t gyro_filtered;
__attribute__((unused))static vector3f_t accel_filtered;

void dal_ins_update_filtered(void)
{
    if (!float_is_positive(alpha)) {
        const float cutoff_hz = 20.0;
        alpha = calc_lowpass_alpha_dt(get_loop_delta_t(), cutoff_hz);
    }
    if (float_is_positive(risi.delta_angle_dt)) {
        vector3f_t tmp = v3f_div(&risi.delta_angle, risi.delta_angle_dt);
        tmp = v3f_sub(&tmp, &gyro_filtered);
        tmp = v3f_uniform_scale(&tmp, alpha);
        gyro_filtered = v3f_add(&gyro_filtered, &tmp);
    }
    if (float_is_positive(risi.delta_velocity_dt)) {
        vector3f_t tmp = v3f_div(&risi.delta_velocity, risi.delta_velocity_dt);
        tmp = v3f_sub(&tmp, &accel_filtered);
        tmp = v3f_uniform_scale(&tmp, alpha);
        accel_filtered = v3f_add(&accel_filtered, &tmp);
    }
}

void dal_ins_start_frame(void)
{
    rish.loop_rate_hz = get_ins_loop_rate_hz();
    rish.loop_delta_t = get_loop_delta_t();
    risi.use_accel = get_accel_health();
    if (risi.use_accel) {
    risi.get_delta_velocity_ret = get_delta_velocity(&risi.delta_velocity, &risi.delta_velocity_dt);
    }
    risi.use_gyro = get_gyro_health();
    if (risi.use_gyro) {
        risi.get_delta_velocity_ret = get_delta_angle(&risi.delta_angle, &risi.delta_angle_dt);
        //MY_LOG("risi.delta_angle %f %f %f\n", risi.delta_angle.x,
        //       risi.delta_angle.y, risi.delta_angle.z);
    }
    dal_ins_update_filtered();
}

vector3f_t dal_ins_get_gyro(void)
{
    return gyro_filtered;
}

vector3f_t dal_ins_get_accel(void)
{
    return accel_filtered;
}

bool dal_ins_get_delta_velocity(vector3f_t *delta_velocity, float *delta_velocity_dt)
{
    *delta_velocity = risi.delta_velocity;
    *delta_velocity_dt = risi.delta_velocity_dt;
    return risi.get_delta_velocity_ret;
}

bool dal_ins_get_delta_angle(vector3f_t *delta_angle, float *delta_angle_dt)
{
    //MY_LOG("ins dal delta angle: %f %f %f\n", risi.delta_angle.x, risi.delta_angle.y, risi.delta_angle.z);
    *delta_angle = risi.delta_angle;
    *delta_angle_dt = risi.delta_angle_dt;
    return risi.get_delta_angle_ret;
}

float dal_ins_get_loop_delta_t(void)
{
    return rish.loop_delta_t;
}

vector3f_t dal_ins_get_imu_pos_offset(void)
{
    return pos;
}

uint16_t dal_ins_get_loop_rate_hz(void)
{
    return rish.loop_rate_hz;
}
