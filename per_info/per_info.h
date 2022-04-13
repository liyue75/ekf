#ifndef PER_INFO_H_
#define PER_INFO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t min_time_us;
    uint16_t max_time_us;
    uint32_t elapsed_time_us;
    uint32_t tick_count;
    uint16_t slip_count;
    uint16_t overrun_count;
} task_info_t;

void per_info_reset(void);
void per_info_set_loop_rate(uint16_t rate_hz);
void per_info_allocate_task_info(uint8_t _num_tasks);
void per_info_task_slipped(uint8_t task_index);
void per_info_update_task_info(uint8_t task_index, uint16_t task_time_us, bool overrun);
void per_info_check_loop_time(uint32_t time_in_micros);
void per_info_update_logging(void);
bool per_info_has_task_info(void);
void per_info_free_task_info(void);

#endif // PER_INFO_H_
