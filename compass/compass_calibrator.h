#ifndef COMPASS_CALIBRATOR_H_
#define COMPASS_CALIBRATOR_H_

#include "vector3f.h"

#define COMPASS_MAX_SCALE_FACTOR 1.5
#define COMPASS_MIN_SCALE_FACTOR (1.0 / COMPASS_MAX_SCALE_FACTOR)

void compass_calibrate_new_sample(const vector3f_t *sample);

#endif // COMPASS_CALIBRATOR_H_
