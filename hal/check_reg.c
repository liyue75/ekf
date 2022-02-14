#include <stdlib.h>
#include "check_reg.h"
#include "spi_device.h"

#define USE_MPU9250 1
#if USE_MPU9250
#include "mpu9250_params.h"
static const spi_device_t *spidev = &_mpu9250_spi_params[0];
#endif

#define ENABLE_DEBUG 1
#include "debug.h"


static checkreg_t checked_reg_array[CHECK_REG_NUM];

static struct {
    uint8_t n_allocated;
    uint8_t n_set;
    uint8_t next;
    uint8_t frequency;
    uint8_t counter;
    checkreg_t last_reg_fail;
    checkreg_t *regs;
} checked;

bool setup_checked_registers(void)
{
    checked.regs = checked_reg_array;
    checked.n_allocated = CHECK_REG_NUM;
    checked.frequency = CHECK_REG_FREQ;
    checked.counter = 0;
    return true;
}

void set_checked_bank_reg(uint8_t bank, uint8_t reg, uint8_t val)
{
    checkreg_t *regs = checked.regs;
    for (uint8_t i = 0; i < checked.n_set; i++) {
        if (regs[i].regnum == reg && regs[i].bank == bank) {
            regs[i].value = val;
            return;
        }
    }
    if (checked.n_set == checked.n_allocated) {
        DEBUG("Not enough chedked registers for reg 0x%02x\n",
            (unsigned)reg);
        return;
    }
    regs[checked.n_set].bank = bank;
    regs[checked.n_set].regnum = reg;
    regs[checked.n_set].value = val;
    checked.n_set++;
}

void set_checked_register(uint8_t reg, uint8_t val)
{ 
    set_checked_bank_reg(0, reg, val);
}

static bool check_next(void)
{
    if (checked.n_set == 0) {
        return true;
    }
    if (++checked.counter < checked.frequency) {
        return true;
    }
    checked.counter = 0;
    checkreg_t reg = checked.regs[checked.next];
    uint8_t v, v2;
    if ((!read_registers(spidev, reg.regnum, &v, 1) || v != reg.value) &&
        (!read_registers(spidev, reg.regnum, &v2, 1) || v2 != reg.value)) {
        DEBUG("0x%02xreg  0x%02x -> 0x%2x", reg.regnum, v, reg.value);
        write_register(spidev, reg.regnum, reg.value);
        checked.last_reg_fail = reg;
        checked.last_reg_fail.value = v;
        return false;
    }
    checked.next = (checked.next + 1) % checked.n_set;
    return true;
}

bool check_next_register(checkreg_t *fail)
{
    if (check_next()) {
        return true;
    }
    *fail = checked.last_reg_fail;
    return false;
}



















