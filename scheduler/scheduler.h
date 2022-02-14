#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include <stdbool.h>
#include <stdint.h>

#define SCHED_TASK(func, task_name, interval_ticks, _max_time_micros, _priority) { \
    .function = func, \
    .name = task_name, \
    .rate_hz = interval_ticks, \
    .max_time_micros = _max_time_micros, \
    .priority = _priority \
}

typedef void (*task_fn_t)(void*);

typedef struct {
    task_fn_t function;
    const char *name;
    float rate_hz;
    uint16_t max_time;
    uint8_t priority;
} task_t;

uint16_t get_loop_rate_hz(void);
uint32_t get_loop_period_us(void);
float get_loop_period_s(void);

bool scheduler_init(const task_t *tasks, uint8_t num_tasks);

#endif // SCHEDULER_H_
