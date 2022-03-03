#ifndef MPU_9250_H
#define MPU_9250_H

#include "spi_device.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        //uint8_t n_slaves;
        //uint8_t max_slaves;
        //uint32_t devid;
        uint8_t addr;
        uint8_t instance;
        uint8_t sample_reg_start;
        uint8_t sample_size;
        bool registered;
        uint8_t mpu_addr;
        uint8_t mpu_reg;
        uint8_t mpu_ctrl;
        uint8_t mpu_do;
    } mpu9250_slave_t;
bool mpu9250_init(void);

void imu_backend_start(void);

void configure_slaves(void);

    bool mpu9250_update(void);

#ifdef __cplusplus
}
#endif

#endif
