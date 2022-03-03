#ifndef COMPASS_BACKEND_H_
#define COMPASS_BACKEND_H_
#include <stdint.h>
#include "vector3f.h"

void compass_accumulate_sample(vector3f_t * field, uint32_t max_samples);

void compass_backend_init(void);
void compass_drain_accumlated_sample(void);

#endif // COMPASS_BACKEND_H_
