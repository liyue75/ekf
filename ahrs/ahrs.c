#include "ahrs.h"
#include "matrix3f.h"
#include "ekf3.h"
#include "mutex.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static mutex_t ahrs_rsem;
static bool ekf3_started;

__attribute__((unused))static float gps_gain = 1.0f;
bool fly_forward;
vehicle_class_t vehicle_class;
static location_t home;
static bool takeoff_expected;
static matrix3f_t rotation_autopilot_body_to_vehicle_body;
static matrix3f_t rotation_vehicle_body_to_autopilot_body;

static vector3f_t last_trim = {0, 0, 0};
static matrix3f_t rotation_imu_to_vehicle;
static matrix3f_t rotation_vehicle_to_imu;

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
    m3f_from_euler(&rotation_imu_to_vehicle,last_trim.x, last_trim.y, last_trim.z);
    rotation_vehicle_to_imu = m3f_transpose(&rotation_imu_to_vehicle);
    set_fly_forward(true);
    vehicle_class = GROUND;
}

void ahrs_reset(void)
{
    mutex_lock(&ahrs_rsem);
    ekf3_started = ekf_initialise_filter();
    mutex_unlock(&ahrs_rsem);
}

bool ahrs_get_position(location_t *loc)
{
    if (ekf3_get_llh(loc)) {
        return true;
    }
    return false;
}

location_t *ahrs_get_home(void)
{
    return &home;
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

void ahrs_reset_gyro_drift(void)
{
    mutex_lock(&ahrs_rsem);
    ekf3_reset_gyro_bias();
    mutex_unlock(&ahrs_rsem);
}
