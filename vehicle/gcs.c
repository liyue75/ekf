#include <stdbool.h>
#include "inertial_sensor.h"
#include "gcs.h"
#include "ahrs.h"
#include "board_led.h"
#include "uart_device.h"
#include "xtimer.h"
#include "accel_cal.h"
#include "compass_calibrator.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static bool mag_cal_test = false;

bool is_mag_cal_testing(void)
{
    return mag_cal_test;
}

static mav_result_t mag_cal(void)
{
    //compass_calibrator_init();
    /* if(!compass_start_calibration()) { */
    /*     return MAV_RESULT_FAILED; */
    /* } */
    return MAV_RESULT_ACCEPTED;
}

static bool calibrate_gyros(void)
{
    init_gyro();
    if (!gyro_calibrated_ok_all()) {
        return false;
    }
    ahrs_reset_gyro_drift();
    return true;
}

static mav_result_t accel_cal(void)
{
    if (!calibrate_gyros()) {
        return MAV_RESULT_FAILED;
    }
    acal_init();
    acal_start();
    return MAV_RESULT_ACCEPTED;
}

static mav_result_t simple_accel_cal(void)
{
    //ins_simple_accel_cal(); //do not use simple accel cal because it is not accuracy
    return MAV_RESULT_ACCEPTED;
}

void pre_calibration(void)
{
    if (!gpio_read(IMU_CAL_SWITCH)) {
        led_on(LED_2);
        accel_cal();
        mag_cal();
        led_off(LED_2);
    } else {
        led_on(LED_2);
        simple_accel_cal();
//        mag_cal_test = true;
        xtimer_sleep(1);
        led_off(LED_2);
        set_fly_forward(true);
    }
}
