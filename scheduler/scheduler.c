#include <stdio.h>
#include <stdlib.h>
#include "scheduler.h"
#include "fusion_math.h"
#include "per_info.h"
#include "vehicle.h"
#include "uart_device.h"
#include "xtimer.h"
#include "board_led.h"
#include "inertial_sensor.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define SCHEDULER_DEFAULT_LOOP_RATE 50
#define ENABLE_SCHEDULER_DEBUG 0
#define ENABLE_RECORD_TASK_INFO 0
static bool scheduler_debug = ENABLE_SCHEDULER_DEBUG;
static bool options = ENABLE_RECORD_TASK_INFO;

static uint16_t loop_rate_hz = SCHEDULER_DEFAULT_LOOP_RATE;
static uint16_t active_loop_rate_hz;
__attribute__((unused))static uint32_t last_loop_time_us;
static float last_loop_time_s;
static uint16_t loop_period_us;
static float loop_period_s = 0;
static const task_t *vehicle_tasks;
static uint8_t num_vehicle_tasks;
static uint8_t num_tasks;
__attribute__((unused))static const task_t *common_tasks;
__attribute__((unused))static uint8_t num_common_tasks;
static uint16_t *last_run;
static uint32_t task_time_allowed;
static uint32_t task_time_started;
__attribute__((unused))static uint32_t spare_micros;
__attribute__((unused))static uint8_t spare_ticks;;
uint32_t loop_timer_start_us = 0;

static uint16_t tick_counter = 0;
static const uint8_t max_task_slowdown = 4;
static uint32_t task_not_achieved = 0;
__attribute__((unused))static uint32_t task_all_achieved = 0;
__attribute__((unused))static uint32_t extra_loop_us;


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
        loop_period_us = 1000000UL / loop_rate_hz;
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

bool scheduler_init(const task_t *tasks, uint8_t _num_tasks)
{
    if (loop_rate_hz < 50) {
        loop_rate_hz = 50;
    } else if (loop_rate_hz > 1000) {
        loop_rate_hz = 1000;
    }
    last_loop_time_s = 1.0 / loop_rate_hz;
    vehicle_tasks = tasks;
    num_vehicle_tasks = _num_tasks;
    vehicle_get_common_scheduler_tasks(&common_tasks, &num_common_tasks);
    num_tasks = num_vehicle_tasks + num_common_tasks;
    last_run = (uint16_t *)calloc(num_tasks, sizeof(uint16_t));
    tick_counter = 0;
    per_info_set_loop_rate(get_loop_rate_hz());
    per_info_reset();
    if (options) {
        per_info_allocate_task_info(num_tasks);
    }
    // sanity check the task lists to ensure the priorities are
    // never decrease
    uint8_t old = 0;
    for (uint8_t i = 0; i < num_common_tasks; i++) {
        if (common_tasks[i].priority < old) {
            DEBUG("common tasks priority order error\n");
            return false;
        }
        old = common_tasks[i].priority;
    }
    old = 0;
    for (uint8_t i = 0; i < num_vehicle_tasks; i++) {
        if (vehicle_tasks[i].priority < old) {
            DEBUG("vehicle tasks priority order error\n");
            return false;
        }
        old = vehicle_tasks[i].priority;
    }
    return true;
}

static void scheduler_run(uint32_t time_available)
{
    uint32_t now = xtimer_now().ticks32;
    uint8_t vehicle_tasks_offset = 0;
    uint8_t common_tasks_offset = 0;
    for (uint8_t i = 0; i < num_tasks; i++) {
        bool run_vehicle_task = false;
        if (vehicle_tasks_offset < num_vehicle_tasks &&
            common_tasks_offset < num_common_tasks) {
            const task_t *vehicle_task = &vehicle_tasks[vehicle_tasks_offset];
            const task_t *common_task = &common_tasks[common_tasks_offset];
            if (vehicle_task->priority <= common_task->priority) {
                run_vehicle_task = true;
            }
        } else if (vehicle_tasks_offset < num_vehicle_tasks) {
            run_vehicle_task = true;
        } else if (common_tasks_offset < num_common_tasks) {
            run_vehicle_task = false;
        } else {
            MY_LOG("scheduler run tasks offset error\n");
            break;
        }
        const task_t *task = run_vehicle_task ? &vehicle_tasks[vehicle_tasks_offset] :
            &common_tasks[common_tasks_offset];
        if (run_vehicle_task) {
            vehicle_tasks_offset++;
        } else {
            common_tasks_offset++;
        }
        const uint16_t dt = tick_counter - last_run[i];
        uint32_t interval_ticks = (float_is_zero(task->rate_hz) ? 1 :
                                   loop_rate_hz / task->rate_hz);
        if (interval_ticks < 1) {
            interval_ticks = 1;
        }
        if (dt < interval_ticks) {
            continue;
        }
        task_time_allowed = task->max_time_micros;
        if (dt >= interval_ticks * 2) {
            per_info_task_slipped(i);
        }
        if (dt >= interval_ticks * max_task_slowdown) {
            task_not_achieved++;
        }
        if (task_time_allowed > time_available) {
            continue;
        }
        task_time_started = now;
        task->function();
        last_run[i] = tick_counter;
        now = xtimer_now().ticks32;
        uint32_t time_taken = now - task_time_started;
        __attribute__((unused))bool overrun = false;
        if (time_taken > task_time_allowed) {
            overrun = true;
            //led_on(LED_1);
            MY_LOG("scheduler overrun task[%d] (%ld/%ld)\n", i, time_taken, task_time_allowed);
        }
        per_info_update_task_info(i, time_taken, overrun);
        if (time_taken >= time_available) {
            time_available = 0;
            break;
        }
        time_available -= time_taken;
    }
    spare_micros += time_available;
    spare_ticks++;
    if (spare_ticks == 32) {
        spare_ticks /= 2;
        spare_micros /= 2;
    }
}

static void tick(void)
{
    tick_counter++;
}

void scheduler_loop(void)
{
    ins_wait_for_sample();
    const uint32_t sample_time_us = xtimer_now().ticks32;
    if (loop_timer_start_us == 0) {
        loop_timer_start_us = sample_time_us;
        last_loop_time_s = get_loop_period_s();
    } else {
        last_loop_time_s = (sample_time_us - loop_timer_start_us) * 1.0e-6;
    }
    tick();
    const uint32_t loop_us = get_loop_period_us();
    //uint32_t now = xtimer_now().ticks32;
    uint32_t time_available = 0;
    //const uint32_t loop_tick_us = now - sample_time_us;
    //if (loop_tick_us < loop_us) {
    //    time_available = loop_us - loop_tick_us;
    //}
    time_available = loop_us;
    time_available += extra_loop_us;
    scheduler_run(time_available);
    if (task_not_achieved > 0) {
        extra_loop_us = MIN(extra_loop_us + 100U, 5000);
        task_not_achieved = 0;
        task_all_achieved = 0;
    } else if (extra_loop_us > 0) {
        task_all_achieved++;
        if (task_all_achieved > 50) {
            task_all_achieved = 0;
            if (extra_loop_us > 50) {
                extra_loop_us = extra_loop_us - 50;
            } else {
                extra_loop_us = 0;
            }
        }
    }
    per_info_check_loop_time(sample_time_us - loop_timer_start_us);
    loop_timer_start_us = sample_time_us;
}

uint32_t get_extra_loop_us(void)
{
    return extra_loop_us;
}

__attribute__((unused))static float load_average(void)
{
    if (spare_ticks == 0) {
        return 0;
    }
    const uint32_t loop_us = get_loop_period_us();
    const uint32_t used_time = loop_us - (spare_micros / spare_ticks);
    return used_time / (float)loop_us;
}

void scheduler_update_logging(void)
{
    if (scheduler_debug) {
        per_info_update_logging();
    }
    //MY_LOG("load : %f\n", load_average() * 1000);
    per_info_set_loop_rate(get_loop_rate_hz());
    per_info_reset();
    /* if (!options && per_info_has_task_info()) { */
    /*     per_info_free_task_info(); */
    /* } else if (options && !per_info_has_task_info()) { */
    /*     per_info_allocate_task_info(); */
    /* } */
}
