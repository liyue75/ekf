#include "per_info.h"

static uint16_t loop_rate_hz;
static uint16_t overtime_threshold_micros;
static uint16_t loop_count;
static uint32_t max_time;
static uint32_t min_time;
static uint16_t long_running;
static float filtered_loop_time;
//__attribute__((unsued))static bool ignore_loop;
__attribute__((unused))static uint8_t num_tasks;

void per_info_reset(void)
{
    loop_count = 0;
    max_time = 0;
    min_time = 0;
    long_running = 0;
}

void per_info_set_loop_rate(uint16_t rate_hz)
{
    overtime_threshold_micros = 1000000 / rate_hz * 1.2f;
    if (loop_rate_hz != rate_hz) {
        loop_rate_hz = rate_hz;
        filtered_loop_time = 1.0f / rate_hz;
    }
}
