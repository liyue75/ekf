#include <stdbool.h>
#include "inertial_sensor.h"
#include "gcs.h"
#include "ahrs.h"
#include "board_led.h"
#include "uart_device.h"
#include "xtimer.h"
#include "accel_cal.h"

#define ENABLE_DEBUG 1
#include "debug.h"

static mav_result_t mag_cal(void)
{
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
    return MAV_RESULT_ACCEPTED;
}

void pre_calibration(void)
{
    if (!gpio_read(IMU_CAL_SWITCH)) {
        led_on(LED_2);
        accel_cal();
        mag_cal();
        DEBUG("calibrate finished, need restart\n");
    } else {
        led_on(LED_2);
        simple_accel_cal();
        xtimer_sleep(1);
        led_off(LED_2);
    }
}
