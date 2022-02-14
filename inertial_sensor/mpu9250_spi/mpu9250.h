#ifndef MPU_9250_H
#define MPU_9250_H

#include "spi_device.h"

#ifdef __cplusplus
extern "C" {
#endif

__attribute__((unused)) bool sensor_init(void);

void sensor_start(void);

#ifdef __cplusplus
}
#endif

#endif
