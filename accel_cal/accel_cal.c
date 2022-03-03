#include "accel_cal.h"
#include "inertial_sensor.h"

static accel_cal_status_t cal_status;
accel_cal_t _acal;
static bool start_collect_sample;
static uint8_t num_active_calibrators;
//static uint8_t num_clients = 0;
static uint32_t last_position_request_ms;
static uint8_t step = 0;
static accel_cal_status_t last_result;
extern bool _accel_calibrator_inited;

accel_cal_status_t acal_get_status(void)
{
    return cal_status;
}

bool acal_running(void)
{
    return cal_status == ACCEL_CAL_WAITING_FOR_ORIENTATION ||
        cal_status == ACCEL_CAL_COLLECTING_SAMPLE;
}


void acal_start(void)
{
    if (_acal.started) {
        return;
    }
    start_collect_sample = false;
    num_active_calibrators = 0;
    acal_cal_clear();
    acal_cal_start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 0.5);
    num_active_calibrators++;
    _acal.started = true;
    _acal.saving = false;
    _acal.use_gcs_snoop = true;
    last_position_request_ms = 0;
    step = 0;
    last_result = ACCEL_CAL_NOT_STARTED;
    acal_update_status();
}

static bool get_calibrator(void)
{
    return _accel_calibrator_inited;
}

void acal_update_status(void)
{
    if (!get_calibrator()) {
        cal_status = ACCEL_CAL_NOT_STARTED;
        return;
    }
    if (acal_cal_get_status() == ACCEL_CAL_FAILED) {
        cal_status = ACCEL_CAL_FAILED;
        return;
    }
    if (acal_cal_get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
        cal_status = ACCEL_CAL_COLLECTING_SAMPLE;
        return;
    }
    if (acal_cal_get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        cal_status = ACCEL_CAL_WAITING_FOR_ORIENTATION;
        return;
    }
    if (acal_cal_get_status() == ACCEL_CAL_NOT_STARTED) {
        cal_status = ACCEL_CAL_NOT_STARTED;
        return;
    }
    cal_status = ACCEL_CAL_SUCCESS;
    return;
}
