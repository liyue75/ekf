#include "accel_cal.h"
#include "inertial_sensor.h"
#include "xtimer.h"
#include "uart_device.h"
#include "board_led.h"

#define AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS 1000

static accel_cal_status_t cal_status;
accel_cal_t _acal;
static bool start_collect_sample;
static uint8_t num_active_calibrators;
//static uint8_t num_clients = 0;
static uint32_t last_position_request_ms;
static uint8_t step = 0;
static accel_cal_status_t last_result = ACCEL_CAL_NOT_STARTED;
extern bool _accel_calibrator_inited;
static bool waiting_for_ack = false;
static int last_pin_value;

accel_cal_status_t acal_get_status(void)
{
    return cal_status;
}

accel_cal_status_t acal_get_last_status(void)
{
    return last_result;
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
    last_pin_value = 0;
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
        //led_off(LED_1);
        return;
    }
    if (acal_cal_get_status() == ACCEL_CAL_NOT_STARTED) {
        cal_status = ACCEL_CAL_NOT_STARTED;
        return;
    }
    cal_status = ACCEL_CAL_SUCCESS;
    return;
}

static void collect_sample(void)
{
    if (cal_status != ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        return;
    }
    acal_cal_collect_sample();
    start_collect_sample = false;
    acal_update_status();
}

static void clear(void)
{
   if (!_acal.started) {
       return;
   }
   acal_cal_clear();
   step = 0;
   _acal.started = false;
   _acal.saving = false;
   acal_update_status();
}

static void fail(void)
{
    MY_LOG("Accel Calibration FAILED\n");
    ins_acal_event_failure();
    last_result = ACCEL_CAL_FAILED;
    clear();
}

static void success(void)
{
    MY_LOG("Accel calibration successful\n");
    last_result = ACCEL_CAL_SUCCESS;
    clear();
}

void acal_update(void)
{
    if (!get_calibrator()) {
        return;
    }
    if (_acal.started) {
        acal_update_status();
        //uint8_t num_active_calibrators = 1;
        if (start_collect_sample) {
            collect_sample();
        }
        uint8_t cal_step;
        switch (cal_status) {
            case ACCEL_CAL_NOT_STARTED:
                fail();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION:
                cal_step = acal_cal_get_num_samples_collected() + 1;
                if (cal_step != step) {
                    step = cal_step;
                    if (_acal.use_gcs_snoop) {
                        const char *msg;
                        switch (step) {
                            case ACCELCAL_VEHICLE_POS_LEVEL:
                                msg = "level";
                                led_on(LED_3);
                                break;
                            case ACCELCAL_VEHICLE_POS_LEFT:
                                msg = "on its LEFT side";
                                led_on(LED_2);
                                break;
                            case ACCELCAL_VEHICLE_POS_RIGHT:
                                msg = "on its RIGHT side";
                                led_on(LED_3);led_on(LED_2);
                                break;
                            case ACCELCAL_VEHICLE_POS_NOSEDOWN:
                                msg = "nose DOWN";
                                led_on(LED_1);
                                break;
                            case ACCELCAL_VEHICLE_POS_NOSEUP:
                                msg = "nose UP";
                                led_on(LED_1);led_on(LED_3);
                                break;
                            case ACCELCAL_VEHICLE_POS_BACK:
                                msg = "on its BACK";
                                led_on(LED_1);led_on(LED_2);
                                break;
                            default:
                                fail();
                                return;
                        }
                        MY_LOG("Place vehicle %s and swith botton.\n", msg);
                        waiting_for_ack = true;
                    }
                }
                uint32_t now = xtimer_now().ticks32 / 1000;
                if (now - last_position_request_ms > AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS) {
                    last_position_request_ms = now;
                }
                break;
            case ACCEL_CAL_COLLECTING_SAMPLE:
                acal_cal_check_for_timeout();
                acal_update_status();
                if (cal_status == ACCEL_CAL_FAILED) {
                    fail();
                }
                return;
            case ACCEL_CAL_SUCCESS:
                if (_acal.saving) {
                    success();
                    return;
                } else {
                    ins_acal_save_calibrations();
                    _acal.saving = true;
                }
                return;
            default:
            case ACCEL_CAL_FAILED:
                fail();
                return;
        }

    }
    /* else if (last_result != ACCEL_CAL_NOT_STARTED) { */
    /*     uint32_t now = xtimer_now().ticks32 / 1000; */
    /*     if (now - last_position_request_ms > AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS) { */
    /*         last_position_request_ms = now; */
    /*         switch (last_result) { */
    /*             case ACCEL_CAL_SUCCESS: */
    /*                 MY_LOG("VEHICLE_POS %d\n", ACCELCAL_VEHICLE_POS_SUCCESS); */
    /*                 break; */
    /*             case ACCEL_CAL_FAILED: */
    /*                 MY_LOG("VEHICLE_POS %d\n", ACCELCAL_VEHICLE_POS_FAILED); */
    /*                 break; */
    /*             default: */
    /*                 break; */
    /*         } */
    /*     } */
    /* } */
}

void acal_handle_message(void)
{
    if (!waiting_for_ack) {
        return;
    }
    int pin_value = gpio_read(IMU_CAL_SWITCH);
    if (pin_value != last_pin_value) {
        last_pin_value = pin_value;
        start_collect_sample = true;
        waiting_for_ack = false;
    }
}
