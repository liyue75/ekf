#ifndef GCS_H_
#define GCS_H_

#include <stdbool.h>

typedef enum {
MAV_RESULT_ACCEPTED = 0,
MAV_RESULT_TEMPORARILY_REJECTED = 1,
MAV_RESULT_DENIED = 2,
MAV_RESULT_UNSUPPORTED = 3,
MAV_RESULT_FAILED = 4,
MAV_RESULT_IN_PROGRESS = 5,
MAV_RESULT_CANCELLED = 6,
MAV_RESULT_ENUM_END = 7,
} mav_result_t;

void pre_calibration(void);
bool is_mag_cal_testing(void);

#endif // GCS_H_
