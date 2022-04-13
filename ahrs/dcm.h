#ifndef DCM_H_
#define DCM_H_

#include <stdint.h>
#include "ahrs_backend.h"

typedef enum {
GPSUse_Disable = 0,
GPSUse_Enable = 1,
GPSUse_EnableWithHeight = 2
} GPSUse_t;

void dcm_init(void);
void dcm_reset(void);
void dcm_update(void);
void dcm_get_results(ahrs_estimates_t *results);

#endif // DCM_H_
