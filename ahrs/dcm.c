#include <string.h>
#include <math.h>
#include "dcm.h"
#include "matrix3f.h"
#include "vector3f.h"
#include "vector2f.h"
#include "inertial_sensor.h"
#include "fusion_math.h"
#include "xtimer.h"
#include "uart_device.h"
#include "compass.h"
#include "ahrs.h"
#include "gps.h"
#include "common.h"
#include "ahrs_backend.h"
#include "location.h"

#define GPS_SPEED_MIN 3
#define SPIN_RATE_LIMIT 20

static const float ki = 0.0087f;
static const float ki_yaw = 0.01f;
static vector3f_t accel_ef;
static vector3f_t accel_ef_blended;
static float kp_yaw = 0.2f;
static float kp = 0.2f;
static float gps_gain = 1.0f;
__attribute__((unused))static float beta = 0.1f;
__attribute__((unused))static GPSUse_t gps_use  = GPSUse_Enable;
static int8_t gps_minsats = 6;
static matrix3f_t dcm_matrix;
static matrix3f_t body_dcm_matrix;

static float roll;
static float pitch;
static float yaw;
static float sin_yaw;
static float cos_yaw;

static float renorm_val_sum;
static uint16_t renorm_val_count;
static float error_rp = 1.0f;
static float error_yaw = 1.0f;

static vector3f_t ra_sum;
vector3f_t last_velocity;
static float ra_deltat;
static uint32_t ra_sum_start;
static uint8_t active_accel_instance;
__attribute__((unused))static vector3f_t omega_P;
__attribute__((unused))static vector3f_t omega_yaw_P;
__attribute__((unused))static vector3f_t omega_I;
__attribute__((unused))static vector3f_t omega_I_sum;
static float omega_I_sum_time;
static vector3f_t omega;
static bool have_initial_yaw;
static vector3f_t ra_delay_buffer;
static uint32_t last_startup_ms;
static uint32_t last_failure_ms;

//earths magnetic field
static float last_declination;
vector2f_t mag_earth = {1, 0};

static bool have_gps_lock;
static int32_t last_lat;
static int32_t last_lng;
static uint32_t last_pos_ms;
static float position_offset_north;
static float position_offset_east;
static bool have_position;
static uint32_t last_consistent_heading;
__attribute__((unused))static vector3f_t wind;
static uint32_t compass_last_update;
static uint32_t gps_last_update;

static float last_airspeed;
location_t last_origin;

void dcm_init(void)
{
    m3f_identity(&dcm_matrix);
}

static void reset(__attribute__((unused))bool recover_eulers)
{
    //MY_LOG("dcm reset\n");
    v3f_zero(&omega_I);
    v3f_zero(&omega_P);
    v3f_zero(&omega_yaw_P);
    v3f_zero(&omega);
    if (recover_eulers && !isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
        MY_LOG("dcm recover_euldrs\n");
        m3f_from_euler(&dcm_matrix, roll, pitch, yaw);
    } else {
        vector3f_t init_acc_vec = get_accel();
        uint8_t counter = 0;
        while((v3f_length(&init_acc_vec) < 9.0f || v3f_length(&init_acc_vec) > 11) &&
              counter++ < 20) {
            MY_LOG("dcm init acc vec bad\n");
            ins_wait_for_sample();
            ins_update();
            init_acc_vec = get_accel();
        }
        if (v3f_length(&init_acc_vec) > 5.0f) {
            pitch = atan2f(init_acc_vec.x, norm_2f(init_acc_vec.y, init_acc_vec.z));
            roll = atan2f(-init_acc_vec.y, -init_acc_vec.z);
        } else {
            roll =  0.0f;
            pitch = 0.0f;
        }
        m3f_from_euler(&dcm_matrix, roll, pitch, 0);
        /* for (uint8_t i = 0; i < 3; i++) { */
        /*     for (uint8_t j = 0; j < 3; j++) { */
        /*         MY_LOG("%f ", ((float *)&dcm_matrix)[i * 3 + j]); */
        /*     } */
        /*     MY_LOG("\n"); */
        /* } */
    }
    cos_yaw = cosf(yaw);
    sin_yaw = sinf(yaw);
    //MY_LOG("yaw: %f\n", yaw);
    last_startup_ms = xtimer_now().ticks32 / 1000;
}

void dcm_reset(void)
{
    reset(false);
}

static void matrix_update(float G_Dt)
{
    v3f_zero(&omega);
    vector3f_t delta_angle = {0, 0, 0};
    if (get_gyro_health()) {
        float dangle_dt;
        get_delta_angle(&delta_angle, &dangle_dt);
    }
    if (G_Dt > 0) {
        omega = v3f_div(&delta_angle, G_Dt);
        //MY_LOG("omega:%f %f %f\n", omega.x, omega.y, omega.z);
        omega = v3f_add(&omega, &omega_I);
        vector3f_t tmp = v3f_add(&omega, &omega_P);
        tmp = v3f_add(&tmp, &omega_yaw_P);
        tmp = v3f_uniform_scale(&tmp, G_Dt);
        /* MY_LOG("omega_I:%f %f %f, omega_P:%f %f %f, omega_P_yaw:%f %f %f\n", */
        /*        omega_I.x, omega_I.y, omega_I.z, omega_P.x, omega_P.y, omega_P.z, */
        /*        omega_yaw_P.x, omega_yaw_P.y, omega_yaw_P.z); */
        //MY_LOG("tmp:%f %f %f\n", tmp.x, tmp.y, tmp.z);
        m3f_rotate(&dcm_matrix, &tmp);
        /* for (uint8_t i = 0; i < 3; i++) { */
        /*     for (uint8_t j = 0; j < 3; j++) { */
        /*         MY_LOG("%f ", ((float *)&dcm_matrix)[i * 3 + j]); */
        /*     } */
        /*     MY_LOG("\n"); */
        /* } */

    }
}


static bool renorm(const vector3f_t *a, vector3f_t *result)
{
    const float renorm_val = 1.0f / v3f_length(a);
    renorm_val_sum += renorm_val;
    renorm_val_count++;
    if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
        if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
            MY_LOG("ERROR: DCM renorm\n");
            return false;
        }
    }
    *result = v3f_uniform_scale(a, renorm_val);
    return true;
}

static void normalize(void)
{
    /* for (uint8_t i = 0; i < 3; i++) { */
    /*         for (uint8_t j = 0; j < 3; j++) { */
    /*             MY_LOG("%f ", ((float *)&dcm_matrix)[i * 3 + j]); */
    /*         } */
    /*         MY_LOG("\n"); */
    /* } */
    //MY_LOG("a: %f %f %f\n", dcm_matrix.a.x, dcm_matrix.a.y, dcm_matrix.a.z);
    const float error = v3f_dot_product(&dcm_matrix.a, &dcm_matrix.b);
    //MY_LOG("error: %f\n", error);
    vector3f_t tmp = v3f_uniform_scale(&dcm_matrix.b, 0.5f * error);
    const vector3f_t t0 = v3f_sub(&dcm_matrix.a, &tmp);
    //MY_LOG("t0: %f %f %f\n", t0.x, t0.y, t0.z);
    tmp = v3f_uniform_scale(&dcm_matrix.a, 0.5f * error);
    const vector3f_t t1 = v3f_sub(&dcm_matrix.b, &tmp);
    const vector3f_t t2 = v3f_cross_product(&t0, &t1);
    if (!renorm(&t0, &dcm_matrix.a) ||
        !renorm(&t1, &dcm_matrix.b) ||
        !renorm(&t2, &dcm_matrix.c)) {
        MY_LOG("renorm failed\n");
        last_failure_ms = xtimer_now().ticks32;
        reset(true);
    }
    //MY_LOG("rn a: %f %f %f\n", dcm_matrix.a.x, dcm_matrix.a.y, dcm_matrix.a.z);
}

static bool have_gps(void)
{
    if (gps_status() <= NO_FIX) {
        return false;
    }
    return true;
}

bool dcm_use_compass(void)
{
    if (!compass_use_for_yaw()) {
        return false;
    }
    if (!get_fly_forward() || !have_gps()) {
        return true;
    }
    if (gps_ground_speed() < GPS_SPEED_MIN) {
        return true;
    }
    const float error = fabsf(wrap_180(degrees(yaw) - gps_ground_course()));
    if (error > 45) {
        if (xtimer_now().ticks32 / 1000 - last_consistent_heading > 2000) {
            return false;
        }
    } else {
        last_consistent_heading = xtimer_now().ticks32 / 1000;
    }
    return true;
}

static float yaw_error_compass(void)
{
    const vector3f_t *mag = compass_get_field();
    vector2f_t rb  = m3f_mulXY(&dcm_matrix, mag);
    if (v2f_length(&rb) < FLT_EPSILON) {
       return 0.0f;
    }
    rb = v2f_normalized(&rb);
    if (isinf(rb.x) || isinf(rb.y)) {
        return 0;
    }
    if (!float_is_equal(last_declination, compass_get_declination())) {
        last_declination = compass_get_declination();
        mag_earth.x = cosf(last_declination);
        mag_earth.y = sinf(last_declination);
    }
    return v2f_cross_product(&rb, &mag_earth);
}

static float P_gain(float spin_rate)
{
    if (spin_rate < ToRad(50)) {
        return 1.0f;
    }
    if (spin_rate > ToRad(500)) {
        return 10.0f;
    }
    return spin_rate / ToRad(50);
}

static float yaw_gain(void)
{
    vector2f_t tmp = {accel_ef.x, accel_ef.y};
    const float VdotEFmag = v2f_length(&tmp);
    if (VdotEFmag <= 4.0f) {
        return 0.2f * (4.5f - VdotEFmag);
    }
    return 0.1f;
}

static bool use_fast_gains(void)
{
    return (xtimer_now().ticks32 / 1000 - last_startup_ms) < 20000U;
}

static void check_matrix(void)
{
    if (v3f_isnan(&dcm_matrix.a) || v3f_isnan(&dcm_matrix.b) || v3f_isnan(&dcm_matrix.c)) {
        MY_LOG("ERROR: DCM matrix NAN\n");
        reset(true);
        return;
    }
    if (!(dcm_matrix.c.x < 1.0f && dcm_matrix.c.x > -1.0f)) {
        normalize();
        if (v3f_isnan(&dcm_matrix.a) || v3f_isnan(&dcm_matrix.b) ||
            v3f_isnan(&dcm_matrix.c) || fabsf(dcm_matrix.c.x) > 1.0f) {
            MY_LOG("ERROR: cdm matrix.c.x = %f\n", dcm_matrix.c.x);
            reset(true);
        }
    }
}

__attribute__((unused))static void drift_correction_yaw(void)
{
    bool new_value = false;
    float yaw_error;
    float yaw_deltat;
    if (compass_is_calibrating()) {
        //MY_LOG("dcm compass is calibrating\n");
        return;
    }
    if (dcm_use_compass()) {
        uint32_t compass_update_t = compass_last_update_usec();
        if (compass_update_t != compass_last_update) {
            yaw_deltat = (compass_update_t - compass_last_update) * 1.0e-6f;
            compass_last_update = compass_update_t;
            if (!have_initial_yaw && compass_read()) {
                const float heading = compass_calculate_heading(&dcm_matrix);
                //MY_LOG("heading: %f\n", heading);
                m3f_from_euler(&dcm_matrix, roll, pitch, heading);
                v3f_zero(&omega_yaw_P);
                have_initial_yaw = true;
            }
            new_value = true;
            yaw_error = yaw_error_compass();
            gps_last_update = gps_last_fix_time_ms();
        }
    } else if (get_fly_forward() && have_gps()) {
        if (gps_last_fix_time_ms() != gps_last_update &&
        gps_ground_speed() >= GPS_SPEED_MIN) {
            yaw_deltat = (gps_last_fix_time_ms() - gps_last_update) * 1.0e-3f;
            gps_last_update = gps_last_fix_time_ms();
            new_value = true;
            const float gps_course_rad = ToRad(gps_ground_course());
            const float yaw_error_rad = wrap_PI(gps_course_rad - yaw);
            yaw_error = sinf(yaw_error_rad);
            if (!have_initial_yaw ||
                yaw_deltat > 20 ||
                (gps_ground_speed() >= 3 * GPS_SPEED_MIN && fabsf(yaw_error_rad) >= 1.047f)) {
                MY_LOG("DCM use gps drift correction yaw\n");
                m3f_from_euler(&dcm_matrix, roll, pitch, gps_course_rad);
                v3f_zero(&omega_yaw_P);
                have_initial_yaw = true;
                yaw_error = 0;
            }
        }
    }
    if (!new_value) {
        omega_yaw_P = v3f_uniform_scale(&omega_yaw_P, 0.97f);
        return;
    }
    const float error_z = dcm_matrix.c.z * yaw_error;
    const float spin_rate = v3f_length(&omega);
    if (kp_yaw < AHRS_YAW_P_MIN) {
        kp_yaw = AHRS_YAW_P_MIN;
    }
    omega_yaw_P.z = error_z * P_gain(spin_rate) * kp_yaw * yaw_gain();
    if (use_fast_gains()) {
        omega_yaw_P.z *= 8;
    }
    if (yaw_deltat < 2.0f && spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        omega_I_sum.z += error_z * ki_yaw * yaw_deltat;
    }
    error_yaw = 0.8f * error_yaw + 0.2f * fabsf(yaw_error);
}

vector3f_t ra_delayed(const vector3f_t *ra)
{
    const vector3f_t ret = ra_delay_buffer;
    ra_delay_buffer = *ra;
    if (v3f_is_zero(&ret)) {
        return *ra;
    }
    return ret;
}

__attribute__((unused))static void drift_correction(float deltat)
{
    vector3f_t velocity;
    uint32_t last_correction_time;
    drift_correction_yaw();
    if (get_accel_health()) {
        vector3f_t delta_velocity = {};
        float delta_velocity_dt = 0;
        get_delta_velocity(&delta_velocity, &delta_velocity_dt);
        if (delta_velocity_dt > 0) {
            vector3f_t tmp = v3f_div(&delta_velocity, delta_velocity_dt);
            //MY_LOG("v: %f %f %f\n", delta_velocity.x, delta_velocity.y, delta_velocity.z);
            /* MY_LOG("dt: %f a: %f %f %f\n", delta_velocity_dt, */
            /*        tmp.x, tmp.y, tmp.z); */
            accel_ef = m3f_multi_v(&dcm_matrix, &tmp);
            //MY_LOG("accel_ef: %f %f %f\n", accel_ef.x, accel_ef.y, accel_ef.z);
            tmp  = v3f_uniform_scale(&accel_ef, deltat);
            //MY_LOG("mv: %f %f %f\n", tmp.x, tmp.y, tmp.z);
            ra_sum  = v3f_add(&ra_sum, &tmp);
            //MY_LOG("deltat: %f rasum: %f %f %f\n", deltat, ra_sum.x, ra_sum.y, ra_sum.z);
        }
    }
    accel_ef_blended = accel_ef;
    ra_deltat += deltat;
    //MY_LOG("ra_deltat: %f\n", ra_deltat);
    const bool fly_forward = get_fly_forward();
    if (!have_gps() ||
        gps_status() < GPS_OK_FIX_3D ||
        gps_num_sats() < gps_minsats) {
        if (ra_deltat < 0.199f) {
            //MY_LOG(".\n");
            return;
        }
        float airspeed;
        airspeed = last_airspeed;
        vector3f_t tmp = m3f_colx(&dcm_matrix);
        velocity = v3f_uniform_scale(&tmp, airspeed);
        velocity = v3f_add(&velocity, &wind);
        //MY_LOG("vel: %f %f %f\n", velocity.x, velocity.y, velocity.z);
        last_correction_time = xtimer_now().ticks32 / 1000;
        have_gps_lock = false;
    } else {
        if (gps_last_fix_time_ms() == ra_sum_start) {
            return;
        }
        velocity = gps_velocity();
        last_correction_time = gps_last_fix_time_ms();
        if (have_gps_lock == false) {
            last_velocity = velocity;
        }
        have_gps_lock = true;
        vector3f_t airspeed = v3f_sub(&velocity, &wind);
        airspeed = m3f_mul_transpose(&body_dcm_matrix, &airspeed);
        last_airspeed = MAX(airspeed.x, 0);
    }
    if (have_gps()) {
        last_lat = (*gps_location()).lat;
        last_lng = (*gps_location()).lng;
        last_pos_ms = xtimer_now().ticks32 / 1000;
        position_offset_north = 0;
        position_offset_east = 0;
        have_position = true;
    } else {
        position_offset_north += velocity.x * ra_deltat;
        position_offset_east += velocity.y * ra_deltat;
    }
    if (ra_sum_start == 0) {
        ra_sum_start = last_correction_time;
        last_velocity = velocity;
        return;
    }
    vector3f_t GA_e = {0, 0, -1};
    if (ra_deltat <= 0) {
        return;
    }
    bool using_gps_corrections = false;
    float ra_scale = 1.0f / (ra_deltat * GRAVITY_MSS);
    if (have_gps_lock || fly_forward) {
        //MY_LOG("have gps lock\n");
        const float v_scale = gps_gain * ra_scale;
        vector3f_t tmp = v3f_sub(&velocity, &last_velocity);
        const vector3f_t vdelta = v3f_uniform_scale(&tmp, v_scale);
        GA_e = v3f_add(&GA_e, &vdelta);
        GA_e = v3f_normalized(&GA_e);
        //MY_LOG("GA_e: %f %f %f\n", GA_e.x, GA_e.y, GA_e.z);
        if (isinf(GA_e.x) || isinf(GA_e.y) || isinf(GA_e.z)) {
            last_failure_ms = xtimer_now().ticks32 / 1000;
            return;
        }
        using_gps_corrections = true;
    }
    vector3f_t error;
    float error_dirn;
    vector3f_t GA_b;
    int8_t besti = -1;
    float best_error = 0;
    for (uint8_t i = 0; i < 1; i++) {
        if (!get_accel_health()) {
            continue;
        }
        ra_sum = v3f_uniform_scale(&ra_sum, ra_scale);
        if (using_gps_corrections) {
            GA_b = ra_delayed(&ra_sum);
        } else {
            GA_b = ra_sum;
        }
        //MY_LOG("GA_b: %f %f %f\n", GA_b.x, GA_b.y, GA_b.z);
        if (v3f_is_zero(&GA_b)) {
            MY_LOG("GA_b: 0\n");
            continue;
        }
        GA_b = v3f_normalized(&GA_b);
        if (v3f_isinf(&GA_b)) {
            continue;
        }
        error = v3f_cross_product(&GA_b, &GA_e);
        error_dirn = v3f_dot_product(&GA_b, &GA_e);
        const float error_length = v3f_length(&error);
        if (besti == -1 || error_length < best_error) {
            besti = i;
            best_error = error_length;
        }
        if (error_dirn < 0.0f) {
            best_error = 1.0f;
        }
    }
    if (besti == -1) {
        last_failure_ms = xtimer_now().ticks32 / 1000;
        return;
    }
    active_accel_instance = besti;
    if (dcm_use_compass()) {
        if (have_gps() && float_is_equal(gps_gain, 1.0f)) {
            error.z *= sinf(fabsf(roll));
        } else {
            error.z = 0;
        }
    }
    if (!ins_healthy()) {
        //MY_LOG("!ins_healthy\n");
        v3f_zero(&error);
    } else {
        error = m3f_mul_transpose(&dcm_matrix, &error);
    }
    if (v3f_isnan(&error) || v3f_isinf(&error)) {
        MY_LOG("error isinf\n");
        check_matrix();
        last_failure_ms = xtimer_now().ticks32 / 1000;
        return;
    }
    error_rp = 0.8 * error_rp + 0.2 * best_error;
    const float spin_rate = v3f_length(&omega);
    if (kp < AHRS_RP_P_MIN) {
        kp = AHRS_RP_P_MIN;
    }
    omega_P = v3f_uniform_scale(&error, P_gain(spin_rate) * kp);
    if (use_fast_gains()) {
        omega_P = v3f_uniform_scale(&omega_P, 8);
    }
    /* if (fly_forward && gps_status() >= GPS_OK_FIX_2D && */
    /*     gps_ground_speed() < GPS_SPEED_MIN && */
    /*     get_accel()->x >= 7 && pitch > radians(-30) && pitch < radians(30)) { */
    /*     omega_P = v3f_uniform_scale(&omega_P, 0.5f); */
    /* } */
    if (spin_rate < ToRad(SPIN_RATE_LIMIT)) {
        vector3f_t tmp = v3f_uniform_scale(&error, ki * ra_deltat);
        omega_I_sum = v3f_add(&omega_I_sum, &tmp);
        omega_I_sum_time += ra_deltat;
    }
    if (omega_I_sum_time >= 5) {
        const float change_limit = ins_get_gyro_drift_rate() * omega_I_sum_time;
        omega_I_sum.x = constrain_float(omega_I_sum.x, -change_limit, change_limit);
        omega_I_sum.y = constrain_float(omega_I_sum.y, -change_limit, change_limit);
        omega_I_sum.z = constrain_float(omega_I_sum.z, -change_limit, change_limit);
        omega_I  = v3f_add(&omega_I, &omega_I_sum);
        //MY_LOG("omega_I: %f %f %f\n", omega_I.x, omega_I.y, omega_I.z);
        v3f_zero(&omega_I_sum);
        omega_I_sum_time = 0;
    }
    memset((void*)&ra_sum, 0, sizeof(ra_sum));
    ra_deltat = 0;
    ra_sum_start = last_correction_time;
    last_velocity = velocity;
}

void dcm_update(void)
{
    const float delta_t = ins_get_delta_time();
    //MY_LOG("delta %f\n", delta_t);
    if (delta_t > 0.2f) {
        MY_LOG("DCM update delta_t > 0.2\n");
        memset((void*)&ra_sum, 0, sizeof(ra_sum));
        ra_deltat = 0;
        return;
    }
    matrix_update(delta_t);
    normalize();
    drift_correction(delta_t);
    check_matrix();
    //body_dcm_matrix = m3f_multi_m(&dcm_matrix, get_rotation_vehicle_body_to_autopilot_body());
    body_dcm_matrix = dcm_matrix;
    m3f_to_euler(&body_dcm_matrix, &roll, &pitch, &yaw);
    cos_yaw = cosf(yaw);
    sin_yaw = sinf(yaw);
    //backup_attitude();
    ahrs_get_origin(&last_origin);
}

static uint32_t log_time_ms;
void dcm_get_results(ahrs_estimates_t *results)
{
    results->roll_rad = roll;
    results->pitch_rad = pitch;
    results->yaw_rad = yaw;
    results->dcm_matrix = body_dcm_matrix;
    results->gyro_estimate = omega;
    results->gyro_drift = omega_I;
    results->accel_ef = accel_ef;
    results->accel_ef_blended = accel_ef_blended;
    if (log_time_ms == 0) {
        log_time_ms = xtimer_now().ticks32 / 1000;
    } else {
        uint32_t now = xtimer_now().ticks32 / 1000;
        if (now - log_time_ms > 1000) {
            log_time_ms = now;
            /* MY_LOG("roll: %f pitch %f yaw %f\n", degrees(results->roll_rad), */
            /*        degrees(results->pitch_rad), degrees(results->yaw_rad)); */

            /* vector3f_t tmp = get_accel(); */
            /* MY_LOG("accel filtered: %f %f %f\n", tmp.x, tmp.y, tmp.z); */

            /* vector3f_t tmp; */
            /* float dt; */
            /* bool ret = get_delta_velocity(&tmp, &dt); */
            /* tmp = v3f_div(&tmp, dt); */
            /* MY_LOG("%d avg accel: %f %f %f\n", ret, tmp.x, tmp.y, tmp.z); */
            /* MY_LOG("accel length: %f\n", v3f_length(&tmp)); */

            //MY_LOG("accel: %f %f %f\n", results->accel_ef.x, results->accel_ef.y, results->accel_ef.z);
        }
    }
}
