#include "dal.h"
#include "vehicle.h"
#include "matrix3f.h"
#include "location.h"
#include "inertial_sensor.h"
#include "xtimer.h"
#include "vehicle.h"
#include "ahrs.h"
#include "ins_dal.h"
#include "gps_dal.h"
#include "compass_dal.h"

static uint32_t last_imu_time_us;
__attribute__((unused))static log_rfrh_t rfrh;
__attribute__((unused))static log_rfrf_t rfrf;
__attribute__((unused))static log_rfrn_t rfrn;

__attribute__((unused))static uint32_t micros;
__attribute__((unused))static uint32_t millis;

__attribute__((unused))static matrix3f_t *rotation_vehicle_body_to_autopilot_body;
static location_t home;
__attribute__((unused))static uint32_t last_imu_time_us;
__attribute__((unused))static bool logging_started;
__attribute__((unused))static bool force_write;

__attribute__((unused))static bool ekf3_init_done;
__attribute__((unused))static bool init_done;

void dal_start_frame(void)
{
    const uint32_t imu_us = get_last_update_usec();
    if (last_imu_time_us == imu_us) {
        return;
    }
    last_imu_time_us = imu_us;
    rfrh.time_flying_ms = get_time_flying_ms();
    rfrh.time_us = xtimer_now64().ticks64;
    //rfrn.armed = get_soft_armed();
    home = *ahrs_get_home();
    rfrn.lat = home.lat;
    rfrn.lng = home.lng;
    rfrn.alt = home.alt;
    //rfrn.EAS2TAS = baro_get_EAS2TAS();
    rfrn.vehicle_class = GROUND;
    rfrn.fly_forward = get_fly_forward();
    //rfrn.takeoff_expected = get_takeoff_expected();
    //rotation_vehicle_body_to_autopilot_body = get_rotation_vehicle_body_to_autopilot_body();
    dal_ins_start_frame();
    //dal_baro_start_frame();
    dal_gps_start_frame();
    dal_compass_start_frame();
    micros = rfrh.time_us;
    millis = rfrh.time_us / 1000;
}
