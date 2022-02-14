#include <stdio.h>
#include <string.h>
#include "spi_device.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

bool spi_device_transfer(const spi_device_t *spidev,
                         const uint8_t *send,
                         uint32_t send_len,
                         uint8_t *recv,
                         uint32_t recv_len)
{
    if ((send_len == recv_len && send == recv) || !send || !recv) {
        //DEBUG("send_len = %lu, recv_len = %lu\n", send_len, recv_len);
        spi_acquire(spidev->spi, spidev->cs, spidev->mode, spidev->clk);
        spi_transfer_bytes(spidev->spi, spidev->cs, false, send, recv,
                           send_len == 0?recv_len:send_len);
        spi_release(spidev->spi);
        return true;
    }
    uint8_t buf[send_len + recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    /*
    {
        DEBUG("buf: ");
        for (uint32_t i = 0; i < send_len + recv_len; i++) {
            DEBUG("0x%x ", buf[i]);
            }
        DEBUG("\n");
    }
    */
    spi_acquire(spidev->spi, spidev->cs, spidev->mode, spidev->clk);
    spi_transfer_bytes(spidev->spi, spidev->cs, false, buf, buf, send_len + recv_len);
    spi_release(spidev->spi);
    //DEBUG("buf[1] = %x\n", buf[1]);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool read_registers(const spi_device_t *spidev,
                    uint8_t first_reg,
                    uint8_t *recv,
                    uint32_t recv_len)
{
    first_reg |= spidev->read_flag;
    //DEBUG("first_reg = 0x%x\n", first_reg);
    return spi_device_transfer(spidev, &first_reg, 1, recv, recv_len);


    /*
    spi_acquire(spidev->spi, spidev->cs, spidev->mode, spidev->clk);
    spi_transfer_byte(spidev->spi, spidev->cs, true, first_reg);
    spi_transfer_bytes(spidev->spi, spidev->cs, false, NULL, recv, recv_len);
    spi_release(spidev->spi);
    return true;
    */
}

bool write_register(const spi_device_t *spidev,
                    uint8_t reg,
                    uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return spi_device_transfer(spidev, buf, sizeof(buf), NULL, 0);
    /*
    spi_acquire(spidev->spi, spidev->cs, spidev->mode, spidev->clk);
    spi_transfer_byte(spidev->spi, spidev->cs, true, reg);
    spi_transfer_byte(spidev->spi, spidev->cs, false, val);
    spi_release(spidev->spi);
    return true;
*/
}
