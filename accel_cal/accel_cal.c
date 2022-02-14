#include "accel_cal.h"

typedef enum accel_cal_status {
ACCEL_CAL_NOT_STARTED = 0,
ACCEL_CAL_WAITING_FOR_ORIENTATION = 1,
ACCEL_CAL_COLLECTING_SAMPLE = 2,
ACCEL_CAL_SUCCESS = 3,
ACCEL_CAL_FAILED = 4
} accel_cal_status_t;

static accel_cal_status_t status;

bool acal_running(void)
{
    return status == ACCEL_CAL_WAITING_FOR_ORIENTATION || status == ACCEL_CAL_COLLECTING_SAMPLE;
}
