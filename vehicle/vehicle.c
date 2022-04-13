#include "kernel_defines.h"
#include "vehicle.h"
#include "scheduler.h"
#include "inertial_sensor.h"
#include "check_reg.h"
#include "uart_device.h"
#include "compass.h"
#include "ahrs.h"
#include "gps.h"
#include "mpu9250_params.h"
#include "xtimer.h"
#include "gcs.h"
#include "accel_cal.h"
#include "board_led.h"
#include "ins_dal.h"
#include "inertial_sensor.h"
#include "microdds_user.h"

#define ENABLE_DEBUG 1
#include "debug.h"

// IMU variables
// Integration time; time last loop took to run

bool _initialized;
static uint32_t last_flying_ms;
static bool likely_flying;
static bool compass_cal_have_started = false;
static uint32_t last_mag_cal_ms = 0;

static void vehicle_cal_update(void)
{
    /* vector3f_t v; */
    /* float dt; */
    /* if (get_delta_velocity(&v, &dt)) { */
    /*     vector3f_t tmp = v3f_div(&v, dt); */
    /*     MY_LOG("%f %f %f\n", tmp.x, tmp.y, tmp.z); */
    /* } */
    ins_acal_update();
    if ((is_mag_cal_testing() || acal_get_last_status() == ACCEL_CAL_SUCCESS) &&
        !compass_cal_have_started) {
        //MY_LOG("mag cal\n");
        uint32_t now = xtimer_now().ticks32 / 1000;
        if (last_mag_cal_ms == 0) {
            last_mag_cal_ms = now;
            return;
        }
        if (now - last_mag_cal_ms < 1000) {
            return;
        }
        last_mag_cal_ms = now;
        MY_LOG("start mag cal\n");
        if (compass_start_calibration()) {
            led_off(LED_1); led_off(LED_2); led_off(LED_3);
            compass_cal_have_started = true;
        }
    }
}

__attribute__((unused))static void send_output_data(void)
{
    microdds_send_data();
}

static const task_t common_tasks[] = {
SCHED_TASK(vehicle_cal_update, 10, 200, 245),
};

static const task_t scheduler_tasks[] = {
SCHED_TASK(ahrs_update, 400, 2000, 6),
SCHED_TASK(gps_update, 50, 400, 18),
SCHED_TASK(update_compass, 10, 250, 39),
SCHED_TASK(compass_cal_update, 2, 200, 102),
//SCHED_TASK(send_output_data, 50, 1000, 108),
SCHED_TASK(scheduler_update_logging, 1, 200, 114)
};

static void init_ap(void)
{
    compass_init();
    gps_init();
    //wheel_encoder_init();
    MY_LOG("scheduler loop rate hz = %d\n", get_loop_rate_hz());
    ahrs_init();
    ins_init(get_loop_rate_hz());
    ahrs_reset();
    pre_calibration();
}

__attribute__((unused)) static float g_dt;

bool vehicle_init(void)
{
    setup_checked_registers();
    scheduler_init(scheduler_tasks, ARRAY_SIZE(scheduler_tasks));
    g_dt = get_loop_period_s();
    init_ap();
    _initialized = true;
    return true;
}

void vehicle_loop(void)
{
    scheduler_loop();
    //g_dt = get_loop_period_s();
}

uint32_t get_time_flying_ms(void)
{
    if (!likely_flying) {
        return 0;
    }
    return xtimer_now().ticks32 / 1000 - last_flying_ms;
}

void vehicle_get_common_scheduler_tasks(const task_t **tasks, uint8_t *num_tasks)
{
    *tasks = common_tasks;
    *num_tasks = ARRAY_SIZE(common_tasks);
}
