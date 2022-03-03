#ifndef DEVICE_H_
#define DEVICE_H_
#include <stdint.h>

typedef void (*PeriodicCb)(void);


void* i2c_register_periodic_callback(uint32_t period_usec, PeriodicCb cb);

#endif // DEVICE_H_
