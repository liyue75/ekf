#ifndef VEHICLE_H_
#define VEHICLE_H_
#include <stdbool.h>
#include <stdint.h>

bool vehicle_init(void);
void vehicle_loop(void);
uint32_t get_time_flying_ms(void);

#endif // VEHICLE_H_
