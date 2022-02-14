#ifndef HAL_DEVICE_H
#define HAL_DEVICE_H

#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct  {
        spi_t spi;
        spi_clk_t clk;
        gpio_t cs;
        gpio_t evt;
        spi_mode_t mode;
        uint8_t read_flag;
    } spi_device_t;

    bool spi_device_transfer(const spi_device_t *spidev,
                             const uint8_t *send,
                             uint32_t send_len,
                             uint8_t *recv,
                             uint32_t recv_len);

    bool read_registers(const spi_device_t *spidev,
                        uint8_t first_reg,
                        uint8_t *recv,
                        uint32_t recv_len);

    bool write_register(const spi_device_t *spidev,
                        uint8_t reg,
                        uint8_t val);

    __attribute__((unused)) bool set_check_register(const spi_device_t *spidev,
                                                    uint8_t reg,
                                                    uint8_t val);



















#ifdef __cplusplus
}
#endif

#endif
