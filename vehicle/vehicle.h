#ifndef VEHICLE_H_
#define VEHICLE_H_
#include <stdbool.h>
#include <stdint.h>
#include "scheduler.h"

bool vehicle_init(void);
void vehicle_loop(void);
uint32_t get_time_flying_ms(void);
void vehicle_get_common_scheduler_tasks(const task_t **tasks, uint8_t *num_tasks);
void update_compass(void);

#endif // VEHICLE_H_
