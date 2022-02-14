#ifndef EKF3_H_
#define EKF3_H_

#include <stdbool.h>
#include <stdint.h>
#include "vector2f.h"

typedef struct {
    uint32_t last_function_call;
    bool core_changed;
    uint32_t last_primary_change;
    float core_delta;
} yaw_reset_data_t;

typedef struct {
    uint32_t last_function_call;
    bool core_changed;
    uint32_t last_primary_change;
    vector2f_t core_delta;         //NE position change
} pos_reset_data_t;

typedef struct {
    uint32_t last_function_call;
    bool core_changed;
    uint32_t last_primary_change;
    float core_delta;        //D position change
} pos_down_reset_data_t;

void set_ekf_enable(bool enable);

void ekf3_init(void);
bool ekf_initialise_filter(void);

#endif // EKF3_H_
