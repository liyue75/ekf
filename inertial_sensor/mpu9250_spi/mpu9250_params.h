#ifndef MPU9250_PARAMS_H_
#define MPU9250_PARAMS_H_

#ifdef __cplusplus__
extern "C" {
#endif

#include "spi_device.h"
#ifndef MPU9250_PARAM_SPI
#define MPU9250_PARAM_SPI SPI_DEV(0)
#endif

#ifndef MPU9250_PARAM_SPI_CLK
//#define MPU9250_PARAM_SPI_CLK SPI_CLK_1MHZ
//#define MPU9250_PARAM_SPI_CLK 10000000 //10MHZ
#define MPU9250_PARAM_SPI_CLK  20000000
#endif

#ifndef MPU9250_PARAM_SPI_CS
#define MPU9250_PARAM_SPI_CS GPIO_PIN(0, 4)
#endif

#ifndef MPU9250_PARAM_SPI_EVT
#define MPU9250_PARAM_SPI_EVT GPIO_PIN(2, 4)
#endif

#ifndef MPU9250_PARAM_SPI_MODE
#define MPU9250_PARAM_SPI_MODE SPI_MODE_0
#endif

#ifndef MPU9250_PARAM_SPI_READ_FLAG
#define MPU9250_PARAM_SPI_READ_FLAG 0x80
#endif

#ifndef MPU9250_PARAMS
#define MPU9250_PARAMS { .spi = MPU9250_PARAM_SPI,  \
        .clk = MPU9250_PARAM_SPI_CLK,               \
        .cs = MPU9250_PARAM_SPI_CS,                 \
        .evt = MPU9250_PARAM_SPI_EVT,               \
        .mode = MPU9250_PARAM_SPI_MODE,         \
        .read_flag = MPU9250_PARAM_SPI_READ_FLAG}
#endif

    static const spi_device_t _mpu9250_spi_params[] =
    {MPU9250_PARAMS};


#ifdef __cplusplus__
}
#endif

#endif // MPU9250_PARAMS_H_
