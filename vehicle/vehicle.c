#include "kernel_defines.h"
#include "vehicle.h"
#include "scheduler.h"
#include "inertial_sensor.h"
#include "check_reg.h"
#include "uart_device.h"
#include "ahrs.h"
#include "gps.h"

// IMU variables
// Integration time; time last loop took to run

bool _initialized;

static const task_t scheduler_tasks[] = {
    //SCHED_TASK(),
};

static void init_ap(void)
{
    //compass_init();
    gps_init();
    //wheel_encoder_init();

    ahrs_init();
    MY_LOG("scheduler loop rate hz = %d\n", get_loop_rate_hz());
    ins_init(get_loop_rate_hz());
    ahrs_reset();
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
