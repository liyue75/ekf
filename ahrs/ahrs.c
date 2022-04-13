#include "ahrs.h"
#include "matrix3f.h"
#include "ekf3.h"
#include "mutex.h"
#include "inertial_sensor.h"
#include "dcm.h"
#include "ahrs_backend.h"
#include "xtimer.h"
#include "fusion_math.h"
#include "uart_device.h"
#include "vector2f.h"

#include "ekf3_core.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static mutex_t ahrs_rsem;
bool _ekf3_started = false;
static matrix3f_t dcm_matrix;
static ahrs_estimates_t dcm_estimates;
static const uint16_t startup_delay_ms = 1000;
location_t _ahrs_current_loc;
vector3f_t _ahrs_velocity;
static bool have_position = false;
static bool home_is_set = false;
static bool home_locked = false;
static float ground_speed;

__attribute__((unused))static float gps_gain = 1.0f;
static uint32_t start_time_ms = 0;
bool fly_forward;
vehicle_class_t vehicle_class;
static location_t home;
static bool takeoff_expected;
static matrix3f_t rotation_autopilot_body_to_vehicle_body;
static matrix3f_t rotation_vehicle_body_to_autopilot_body;

static vector3f_t last_trim = {0, 0, 0};
static matrix3f_t rotation_imu_to_vehicle;
static matrix3f_t rotation_vehicle_to_imu;
float _ahrs_roll;
float _ahrs_pitch;
float _ahrs_yaw;
int32_t _roll_sensor;
int32_t _pitch_sensor;
int32_t _yaw_sensor;


void set_fly_forward(bool b)
{
    fly_forward = b;
}

bool get_fly_forward(void)
{
    return fly_forward;
}

void ahrs_init(void)
{
    mutex_init(&ahrs_rsem);
    ekf3_init();
    set_ekf_enable(true);
    dcm_init();
    m3f_from_euler(&rotation_imu_to_vehicle,last_trim.x, last_trim.y, last_trim.z);
    rotation_vehicle_to_imu = m3f_transposed(&rotation_imu_to_vehicle);
    //set_fly_forward(true);
    vehicle_class = GROUND;
}

void ahrs_reset(void)
{
    mutex_lock(&ahrs_rsem);
    dcm_reset();
    _ekf3_started = ekf3_initialise_filter();
    mutex_unlock(&ahrs_rsem);
}

bool ahrs_get_position(location_t *loc)
{
    if (ekf3_get_llh(loc)) {
        return true;
    }
    return false;
}

location_t ahrs_get_home(void)
{
    return home;
}

bool get_takeoff_expected(void)
{
    return takeoff_expected;
}

matrix3f_t *get_rotation_autopilot_body_to_vehicle_body(void)
{
    return &rotation_autopilot_body_to_vehicle_body;
}

matrix3f_t *get_rotation_vehicle_body_to_autopilot_body(void)
{
    return &rotation_vehicle_body_to_autopilot_body;
}

matrix3f_t *ahrs_get_rotation_body_to_ned(void)
{
    return &dcm_matrix;
}

void ahrs_reset_gyro_drift(void)
{
    mutex_lock(&ahrs_rsem);
    ekf3_reset_gyro_bias();
    mutex_unlock(&ahrs_rsem);
}

void update_dcm(void)
{
    dcm_update();
    dcm_get_results(&dcm_estimates);
    //copy_estimates_from_backend_estimates(&dcm_estimates);
}

__attribute__((unused))static void update_cd_values(void)
{
    _roll_sensor = degrees(_ahrs_roll) * 100;
    _pitch_sensor = degrees(_ahrs_pitch) * 100;
    _yaw_sensor = degrees(_ahrs_yaw) * 100;
    if (_yaw_sensor < 0) {
        _yaw_sensor += 36000;
    }
}

extern state_var_t _s;
static void update_ekf3(void)
{
    if (!_ekf3_started) {
        if (start_time_ms == 0) {
            start_time_ms = xtimer_now().ticks32 / 1000;
            return;
        }
        if (xtimer_now().ticks32 / 1000 - start_time_ms > startup_delay_ms) {
            _ekf3_started = ekf3_initialise_filter();
        }
    }
    if (_ekf3_started) {
        //MY_LOG("ekf3 started\n");
        ekf3_update_filter();
        vector3f_t eulers;
        ekf3_get_rotation_body_to_ned(&dcm_matrix);
        ekf3_get_euler_angles(&eulers);
        _ahrs_roll = eulers.x;
        _ahrs_pitch = eulers.y;
        _ahrs_yaw = eulers.z;
        //MY_LOG("roll %f pitch %f yaw %f\n", roll, pitch, yaw);
        //update_cd_values();
        //ahrs_backend_update_trig();
    }
}

static void update(void)
{
    ins_update();
    mutex_lock(&ahrs_rsem);
    update_dcm();
    update_ekf3();
    //update_aoa_ssa();
    //nmea_out_update();
    mutex_unlock(&ahrs_rsem);
}

bool ahrs_home_is_set(void)
{
    return home_is_set;
}

bool ahrs_set_home(const location_t *loc)
{
    mutex_lock(&ahrs_rsem);
    if (loc->lat == 0 && loc->lng == 0 && loc->alt == 0) {
        mutex_unlock(&ahrs_rsem);
        return false;
    }
    if (!location_check_latlng(loc)) {
        mutex_unlock(&ahrs_rsem);
        return false;
    }
    location_t temp = *loc;
    if (!location_change_alt_frame(&temp, ALT_ABSOLUTE)) {
        mutex_unlock(&ahrs_rsem);
        return false;
    }
    home = temp;
    home_is_set = true;
    //MY_LOG("ahrs home set\n");
    mutex_unlock(&ahrs_rsem);
    return true;
}

void ahrs_lock_home(void)
{
    home_locked = true;
}

static bool set_home(const location_t *loc, bool lock)
{
    if (!ahrs_set_home(loc)) {
        return false;
    }
    if (lock) {
        ahrs_lock_home();
    }
    MY_LOG("ahrs Set HOME to %.6f %.6f at %.2fm", loc->lat * 1.0e-7f, loc->lng * 1.0e-7f, loc->alt * 0.01);
    return true;
}

bool set_home_to_current_location(bool lock)
{
    location_t temp_loc;
    if (ahrs_get_position(&temp_loc)) {
        if (!set_home(&temp_loc, lock)) {
            return false;
        }
        return true;
    }
    return false;
}

bool ahrs_get_velocity_ned(vector3f_t *vec)
{
    ekf3_get_vel_ned(vec);
    return true;
}

void ahrs_update(void)
{
    //update_ahrs_flyforward();
    update();
    have_position = ahrs_get_position(&_ahrs_current_loc);
    if (!ahrs_home_is_set()) {
        if (!set_home_to_current_location(false)) {
        }
    }
    if (ahrs_get_velocity_ned(&_ahrs_velocity)) {
        vector2f_t tmp = {_ahrs_velocity.x, _ahrs_velocity.y};
        ground_speed = v2f_length(&tmp);
    }
}

bool ahrs_get_origin(location_t *ret)
{
    if (!ekf3_get_origin_llh(ret)) {
        return false;
    }
    return true;
}

const matrix3f_t* ahrs_get_dcm_rotation_body_to_ned(void)
{
    return &dcm_estimates.dcm_matrix;
}
