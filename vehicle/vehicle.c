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

#define ENABLE_DEBUG 1
#include "debug.h"

// IMU variables
// Integration time; time last loop took to run

bool _initialized;
static uint32_t last_flying_ms;
static bool likely_flying;

static const task_t scheduler_tasks[] = {
    //SCHED_TASK(),
};

static void init_ap(void)
{
    compass_init();
    gps_init();
    //wheel_encoder_init();
    ahrs_init();
    MY_LOG("scheduler loop rate hz = %d\n", get_loop_rate_hz());
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
{}

uint32_t get_time_flying_ms(void)
{
    if (!likely_flying) {
        return 0;
    }
    return xtimer_now().ticks32 / 1000 - last_flying_ms;
}
