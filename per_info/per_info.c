#include <stdlib.h>
#include <string.h>
#include "per_info.h"
#include "uart_device.h"
#include "board_led.h"
#include "fusion_math.h"
#include "xtimer.h"
#include "scheduler.h"

#define ENABLE_DEBUG 1
#include "debug.h"


static uint16_t loop_rate_hz;
static uint16_t overtime_threshold_micros;
static uint16_t loop_count;
static uint32_t max_time;
static uint32_t min_time;
static uint16_t long_running;
static uint64_t sigma_time;
static uint64_t sigmasquared_time;
static float filtered_loop_time;
static uint8_t num_tasks = 0;
static task_info_t *task_info = NULL;
static uint32_t last_check_us;
bool _ignore_loop;

void per_info_allocate_task_info(uint8_t _num_tasks)
{
    task_info = (task_info_t *)calloc(_num_tasks + 1, sizeof(task_info_t));
    if (task_info == NULL) {
        DEBUG("task_info allocate failed\n");
        num_tasks = 0;
        return;
    }
    num_tasks = _num_tasks;
}

void per_info_task_slipped(uint8_t task_index)
{
    if (task_info && task_index <= num_tasks) {
        task_info[task_index].overrun_count++;
    }
}

void per_info_reset(void)
{
    loop_count = 0;
    max_time = 0;
    min_time = 0;
    long_running = 0;
    sigma_time = 0;
    sigmasquared_time = 0;
    if (task_info != NULL) {
        memset(task_info, 0, (num_tasks + 1) * sizeof(task_info_t));
    }
}

void per_info_set_loop_rate(uint16_t rate_hz)
{
    overtime_threshold_micros = 1000000 / rate_hz * 1.2f;
    if (loop_rate_hz != rate_hz) {
        loop_rate_hz = rate_hz;
        filtered_loop_time = 1.0f / rate_hz;
    }
}

void per_info_update_task_info(uint8_t task_index, uint16_t task_time_us, bool overrun)
{
    if (task_info == NULL) {
        return;
    }
    if (task_index > num_tasks) {
        return;
    }
    task_info_t *ti = &task_info[task_index];
    ti->max_time_us = MAX(ti->max_time_us, task_time_us);
    if (ti->min_time_us == 0) {
        ti->min_time_us = task_time_us;
    } else {
        ti->min_time_us = MIN(ti->min_time_us, task_time_us);
    }
    ti->elapsed_time_us += task_time_us;
    ti->tick_count++;
    if (overrun) {
        ti->overrun_count++;
    }
}

void per_info_check_loop_time(uint32_t time_in_micros)
{
    loop_count++;
    if (_ignore_loop) {
        _ignore_loop = false;
        return;
    }
    if (time_in_micros > max_time) {
        max_time = time_in_micros;
    }
    if (min_time == 0 || time_in_micros < min_time) {
        min_time = time_in_micros;
    }
    if (time_in_micros > overtime_threshold_micros) {
        long_running++;
    }
    sigma_time += time_in_micros;
    sigmasquared_time += time_in_micros * time_in_micros;
    const uint32_t now = xtimer_now().ticks32;
    const uint32_t loop_time_us = now - last_check_us;
    last_check_us = now;
    if (loop_time_us < overtime_threshold_micros + 10000UL) {
        filtered_loop_time = 0.99f * filtered_loop_time + 0.01f * loop_time_us * 1.0e-6f;
    }
}

uint32_t get_stddev_time(void)
{
    return sqrtf((sigmasquared_time - (sigma_time * sigma_time) / loop_count) / loop_count);
}

__attribute__((unused))static uint32_t get_avg_time(void)
{
    return (sigma_time / loop_count);
}

void per_info_update_logging(void)
{
    MY_LOG("PERF: %u/%u [%lu:%lu] F=%uHZ sd=%lu Ex=%lu\n",
           (unsigned)long_running, (unsigned)loop_count,
           (unsigned long)max_time, (unsigned long)min_time,
           (unsigned)(0.5 + (1.0f / filtered_loop_time)),
           (unsigned long)get_stddev_time(),
           (unsigned long)get_extra_loop_us());
}

bool per_info_has_task_info(void)
{
    return task_info != NULL;
}

void per_info_free_task_info(void)
{
    free(task_info);
    task_info = NULL;
    num_tasks = 0;
}
