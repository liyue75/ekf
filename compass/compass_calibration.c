#include "compass.h"
#include "gps.h"
#include "uart_device.h"
#include "compass_calibrator.h"
#include "board_led.h"
#include "xtimer.h"
#include "thread.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#ifndef COMPASS_OFFSETS_MAX_DEFAULT
#define COMPASS_OFFSETS_MAX_DEFAULT 1800
#endif

#ifndef COMPASS_AUTO_ROT_DEFAULT
#define COMPASS_AUTO_ROT_DEFAULT 2
#endif

#define COMPASS_CAL_THREAD_STACKSIZE 4096
#define COMPASS_CAL_THREAD_PRIORITY 10

static char compass_calibration_thread_stack[COMPASS_CAL_THREAD_STACKSIZE];

bool compass_calibrator_inited = false;
static bool cal_saved;
static bool cal_autosave = true;
static float calibration_threshold = COMPASS_CALIBRATION_FITNESS_DEFAULT;
static int16_t offset_max = COMPASS_OFFSETS_MAX_DEFAULT;
static int8_t rotate_auto = COMPASS_AUTO_ROT_DEFAULT;
static bool cal_thread_started = false;
static bool cal_requires_reboot = false;
static bool cal_has_run;

static uint16_t get_offsets_max(void)
{
    return (uint16_t)offset_max;
}

bool compass_is_calibrating(void)
{
    switch (compass_calibrator_get_state().status) {
        case COMPASS_CAL_NOT_STARTED:
        case COMPASS_CAL_SUCCESS:
        case COMPASS_CAL_FAILED:
        case COMPASS_CAL_BAD_ORIENTATION:
            break;
        default:
            return true;
    }
    return false;
}

static void *update_calibration_trampoline(__attribute__((unused))void *arg)
{
    while (true) {
        compass_calibrator_update();
        xtimer_usleep(5000);
    }
    return NULL;
}

bool compass_start_calibration(void)
{
    if (!compass_healthy()) {
        MY_LOG("Compass cal compass unhealthy\n");
        return false;
    }
    if (!compass_use_for_yaw()) {
        MY_LOG("Compass cal !useforyaw\n");
        return false;
    }
    if (!compass_calibrator_inited) {
        compass_calibrator_inited = true;
        compass_calibrator_init();
    }
    if (gps_status() < GPS_OK_FIX_2D) {
        MY_LOG("Compass cal requires GPS Lock\n");
        return false;
    }
    if (rotate_auto) {
        compass_calibrator_set_orientation(ROTATION_NONE, rotate_auto>=2, rotate_auto>=3);
    }
    cal_saved = false;
    compass_calibrator_start(true, 0, get_offsets_max(), calibration_threshold * 2);
    if (!cal_thread_started) {
        cal_requires_reboot = true;
        kernel_pid_t pid = thread_create(compass_calibration_thread_stack,
                                         COMPASS_CAL_THREAD_STACKSIZE,
                                         COMPASS_CAL_THREAD_PRIORITY,
                                         THREAD_CREATE_WOUT_YIELD,
                                         update_calibration_trampoline,
                                         NULL,
                                         "compass cal thread");
        if (pid < 0) {
            MY_LOG("Can not create compass cal thread\n");
            return false;
        }
        MY_LOG("Compass cal thread pid = %d, prio = %d\n", pid, COMPASS_CAL_THREAD_PRIORITY);
        cal_thread_started = true;
    }
    return true;
}

static bool accept_calibration(void)
{
    if (!compass_calibrator_inited) {
        return false;
    }
    compass_report_t cal_report = compass_calibrator_get_report();
    if (cal_saved || cal_report.status == COMPASS_CAL_NOT_STARTED) {
        return true;
    } else if (cal_report.status == COMPASS_CAL_SUCCESS) {
        cal_saved = true;
        vector3f_t ofs = cal_report.ofs;
        vector3f_t diag = cal_report.diag;
        vector3f_t offdiag = cal_report.offdiag;
        float scaler_factor = cal_report.scale_factor;
        //sd_save
        MY_LOG("Compass param:\nofs:%f %f %f\ndiag:%f %f %f\noffdiag:%f %f %f\nscale_factor %f\n",
               ofs.x, ofs.y, ofs.z, diag.x, diag.y, diag.z, offdiag.x, offdiag.y, offdiag.z,
               scaler_factor);
        return true;
    } else {
        return false;
    }
}

static bool reboot = false;
void compass_cal_update(void)
{
    //bool running = false;
    if (!compass_calibrator_inited) {
        return;
    }
    if (compass_calibrator_failed()) {
        MY_LOG("compass calibrator failed\n");
    }
    compass_cal_state_t state = compass_calibrator_get_state();
    if (compass_calibrator_running()) {
        MY_LOG("compass cal completion:");
        for (uint8_t i = 0; i < 10; i++) {
            MY_LOG(" %x", state.completion_mask[i]);
        }
        MY_LOG(", pct: %.1f \n", state.completion_pct);
        //    running = true;
    } else if (cal_autosave && !cal_saved &&
               state.status == COMPASS_CAL_SUCCESS) {
        accept_calibration();
        compass_calibrator_stop();
    }
    if (compass_is_calibrating()) {
        cal_has_run = true;
        return;
    } else if (cal_has_run && !reboot) {
        reboot = true;
        MY_LOG("compass calibration finish, need restart");
    }
}
