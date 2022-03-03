#ifndef ACCEL_CAL_H_
#define ACCEL_CAL_H_

#include <stdbool.h>
#include "vector3f.h"
#include "accel_calibrator.h"

typedef struct {
    bool use_gcs_snoop;
    bool started;
    bool saving;
} accel_cal_t;

bool acal_running(void);
accel_cal_status_t acal_get_status(void);
void acal_start(void);
void acal_update_status(void);
#endif // ACCEL_CAL_H_
