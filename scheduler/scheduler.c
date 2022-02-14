#include <stdio.h>
#include <stdlib.h>
#include "scheduler.h"
#include "fusion_math.h"
#include "per_info.h"

#define SCHEDULER_DEFAULT_LOOP_RATE 50

static uint16_t loop_rate_hz = SCHEDULER_DEFAULT_LOOP_RATE;
static uint16_t active_loop_rate_hz;
__attribute__((unused))static uint32_t last_loop_time_us;
static float last_loop_time_s;
static uint16_t loop_period_us;
static float loop_period_s;
static const task_t *vehicle_tasks;
static uint8_t num_vehicle_tasks;
static uint16_t *last_run;
static uint16_t tick_counter;

uint16_t get_loop_rate_hz(void)
{
    if (active_loop_rate_hz == 0) {
        active_loop_rate_hz = loop_rate_hz;
    }
    return active_loop_rate_hz;
}

uint32_t get_loop_period_us(void)
{
    if (loop_period_us == 0) {
        loop_period_us = 100000UL / loop_rate_hz;
    }
    return loop_period_us;
}

float get_loop_period_s(void)
{
    if (float_is_zero(loop_period_s)) {
        loop_period_s = 1.0f / loop_rate_hz;
    }
    return loop_period_s;
}

bool scheduler_init(const task_t *tasks, uint8_t num_tasks)
{
    if (loop_rate_hz < 50) {
        loop_rate_hz = 50;
    } else if (loop_rate_hz > 1000) {
        loop_rate_hz = 1000;
    }
    last_loop_time_s = 1.0 / loop_rate_hz;
    vehicle_tasks = tasks;
    num_vehicle_tasks = num_tasks;
    last_run = (uint16_t *)malloc(sizeof(uint16_t) * num_vehicle_tasks);
    tick_counter = 0;
    per_info_set_loop_rate(get_loop_rate_hz());
    per_info_reset();
    // sanity check the task lists to ensure the priorities are
    // never decrease
    uint8_t old = 0;
    for (uint8_t i = 0; i < num_vehicle_tasks; i++) {
        if (vehicle_tasks[i].priority < old) {
            return false;
        }
        old = vehicle_tasks[i].priority;
    }
    return true;
}
