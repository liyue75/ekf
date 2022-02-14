#ifndef CHECK_REG_H_
#define CHECK_REG_H_

#include <stdbool.h>
#include <stdint.h>

#define CHECK_REG_NUM  20
#define CHECK_REG_FREQ 10

typedef struct checkreg{
    uint8_t bank;
    uint8_t regnum;
    uint8_t value;
} checkreg_t;


bool setup_checked_registers(void);
void set_checked_register(uint8_t reg, uint8_t val);
bool check_next_register(checkreg_t *fail);

#endif // CHECK_REG_H_
