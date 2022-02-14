#ifndef AHRS_H_
#define AHRS_H_

#include <stdbool.h>

typedef enum vehicle_class {
UNKNOWN,
GROUND
} vehicle_class_t;

void ahrs_init(void);

void set_fly_forward(bool b);
void ahrs_reset(void);
#endif // AHRS_H_
