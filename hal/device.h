#ifndef DEVICE_H_
#define DEVICE_H_
#include <stdint.h>

typedef void (*PeriodicCb)(void);

typedef struct callback_info {
    struct callback_info *next;
    PeriodicCb cb;
    uint32_t period_usec;
    uint64_t next_usec;
} callback_info_t;

void* register_periodic_callback(uint32_t period_usec, PeriodicCb cb);

#endif // DEVICE_H_
