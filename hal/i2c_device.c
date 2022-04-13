#include <stdlib.h>
#include <stdbool.h>
#include "thread.h"
#include "i2c_device.h"
#include "xtimer.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define HAL_I2C_PERIODIC_STACK_SIZE (1024)
#define HAL_I2C_PERIODIC_THREAD_PRIORITY 8

typedef struct callback_info {
    struct callback_info *next;
    PeriodicCb cb;
    uint32_t period_usec;
    uint64_t next_usec;
} callback_info_t;

static char device_periodic_stack[HAL_I2C_PERIODIC_STACK_SIZE];
static bool thread_started = false;
static callback_info_t *callbacks = NULL;
static kernel_pid_t i2c_pid;

static void *bus_thread(__attribute__((unused))void *arg)
{
    while (true) {
        uint64_t now = xtimer_now64().ticks64;
        callback_info_t *callback;
        for (callback = callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                callback->cb();
            }
        }
        __attribute__((unused))uint64_t next_needed = 0;
        now = xtimer_now64().ticks64;
        for (callback = callbacks; callback; callback = callback->next) {
            if (next_needed == 0 ||
            callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }
        //delay for at most 50ms, to handle newly added callbacks
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        if (delay < 100) {
            delay = 100;
        }
        xtimer_usleep(delay);
    }
    return NULL;

}

void * i2c_register_periodic_callback(uint32_t period_usec, PeriodicCb cb)
{
    if (!thread_started) {
        thread_started = true;
        i2c_pid = thread_create(device_periodic_stack,
                                        sizeof(device_periodic_stack),
                                         HAL_I2C_PERIODIC_THREAD_PRIORITY,
                                        THREAD_CREATE_WOUT_YIELD /*| THREAD_CREATE_STACKTEST8*/,
                                        bus_thread,
                                        NULL,
                                        "i2c device periodic thread");
        if (i2c_pid < 0) {
            DEBUG("Could not create device periodic thread\n");
            return NULL;
        }
        DEBUG("i2c periodic thread pid = %d, priority = %d\n", i2c_pid,
              HAL_I2C_PERIODIC_THREAD_PRIORITY);
    }
    callback_info_t *callback = (callback_info_t *)malloc(sizeof(callback_info_t));
    if (callback == NULL) {
        DEBUG("Could not add callback, malloc falling\n");
        return NULL;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = xtimer_now64().ticks64 + period_usec;
    callback->next = callbacks;
    callbacks = callback;
    return callback;
}
