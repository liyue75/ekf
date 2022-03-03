#include "ak8963.h"
#include "compass.h"
#include "mpu9250.h"
#include "uart_device.h"
#include "mpu9250_regs.h"
#include "periph/i2c.h"
#include "i2c_device.h"
#include "compass_backend.h"
#include "xtimer.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define AK8963_I2C_ADDR 0x0c
#define AK8963_WIA 0x00
#define AK8963_Device_ID 0x48
#define AK8963_HXL 0x03

#define AK8963_CNTL1 0x0A
# define AK8963_CONTINUOUS_MODE1 0x02
# define AK8963_CONTINUOUS_MODE2 0x06
# define AK8963_SELFTEST_MODE 0x08
# define AK8963_POWERDOWN_MODE 0x00
# define AK8963_FUSE_MODE 0x0f
# define AK8963_16BIT_ADC 0x10
# define AK8963_14BIT_ADC 0x00

#define AK8963_CNTL2 0x0B
# define AK8963_RESET 0x01

#define AK8963_ASAX 0x10

#define AK8963_MILLIGAUSS_SCALE 10.0f

typedef struct __attribute__((__packed__)) {
    int16_t val[3];
    uint8_t st2;
} sample_regs_t;

static const float ADC_16BIT_RESOLUTION = 0.15f;
static mpu9250_slave_t _aux_bus_slave;
static float magnetometer_ASA[3] = {0, 0, 0};

extern mag_state_t _compass_state[];
uint8_t _compass_instance;

static void init_9250_aux_bus_slave(void)
{
    _aux_bus_slave.addr = AK8963_I2C_ADDR;
    _aux_bus_slave.instance = 0;
    _aux_bus_slave.mpu_addr = MPU9X50_SLAVE0_ADDR_REG + _aux_bus_slave.instance * 3;
    _aux_bus_slave.mpu_reg = _aux_bus_slave.mpu_addr + 1;
    _aux_bus_slave.mpu_ctrl = _aux_bus_slave.mpu_addr + 2;
    _aux_bus_slave.mpu_do = MPU9X50_SLAVE0_DATA_OUT_REG + _aux_bus_slave.instance;
}

static bool check_id(void)
{
    uint8_t data;
    for (uint8_t i = 0; i < 5; i++) {
        i2c_acquire(I2C_DEV(0));
        i2c_read_reg(I2C_DEV(0), AK8963_I2C_ADDR, AK8963_WIA, &data, 0);
        i2c_release(I2C_DEV(0));
        if (data == AK8963_Device_ID) {
            DEBUG("check compass who am i = 0x%x\n", data);
            return true;
        }
    }
    return false;
}

static bool calibrate(void)
{
    i2c_acquire(I2C_DEV(0));
    i2c_write_reg(I2C_DEV(0), AK8963_I2C_ADDR,
                  AK8963_CNTL1, AK8963_FUSE_MODE | AK8963_16BIT_ADC, I2C_NOSTOP);
    uint8_t response[3];
    i2c_read_regs(I2C_DEV(0), AK8963_I2C_ADDR,
                  AK8963_ASAX, response, 3, 0);
    i2c_release(I2C_DEV(0));
    for (int i = 0; i < 3; i++) {
        float data = response[i];
        magnetometer_ASA[i] = ((data - 128) / 256 + 1);
    }
    return true;
}

static bool setup_mode(void)
{
    i2c_acquire(I2C_DEV(0));
    if (i2c_write_reg(I2C_DEV(0), AK8963_I2C_ADDR,
                      AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC, 0) == 0) {
        i2c_release(I2C_DEV(0));
        return true;
    }
    i2c_release(I2C_DEV(0));
    return false;
}

static bool register_compass(void)
{
    _compass_state[0].registered = true;
    _compass_state[0].priority = 0;
    _compass_instance = 0;
    //compass_count = 1;
    return true;
}

static void set_dev_id(void)
{
    _compass_state[0].dev_id = 0;
    _compass_state[0].detected_dev_id = 0;
}

static void set_rotation(Rotation_t rotation)
{
    _compass_state[0].rotation = rotation;
}

static void update(void )
{
    sample_regs_t regs;
    i2c_acquire(I2C_DEV(0));
    if (i2c_read_regs(I2C_DEV(0), AK8963_I2C_ADDR, AK8963_HXL, &regs,
                      sizeof(regs), 0) != 0) {
        i2c_release(I2C_DEV(0));
        return;
    }
    i2c_release(I2C_DEV(0));
    vector3f_t raw_field = {regs.val[0], regs.val[1], regs.val[2]};
    if (float_is_zero(raw_field.x) && float_is_zero(raw_field.y) &&
        float_is_zero(raw_field.z)) {
        return;
    }
    raw_field.x *= magnetometer_ASA[0];
    raw_field.y *= magnetometer_ASA[1];
    raw_field.z *= magnetometer_ASA[2];
    raw_field = v3f_uniform_scale(&raw_field, ADC_16BIT_RESOLUTION * AK8963_MILLIGAUSS_SCALE);
    compass_accumulate_sample(&raw_field, 10);
    //MY_LOG("%d %d %d\n", (int)raw_field.x, (int)raw_field.y, (int)raw_field.z);
}

static bool ak8963_init(void)
{
    if (!check_id()) {
        DEBUG("check ak8963 id failed\n");
        return false;
    }
    if (!calibrate()) {
        DEBUG("compass calibrate failed\n");
        return false;
    }
    if (!setup_mode()) {
        DEBUG("compass set_upmode failed\n");
        return false;
    }
    register_compass();
    set_dev_id();
    set_rotation(ROTATION_NONE);
    i2c_register_periodic_callback(10000, update);
    return true;
}

bool ak8963_probe(void)
{
    if (ak8963_init())
        return true;
    return false;
}

bool ak8963_probe_mpu9250(void)
{
    if (!mpu9250_init()) {
        DEBUG("init mpu9250 error\n");
        return false;
    }
    configure_slaves();
    init_9250_aux_bus_slave();
    ak8963_init();
    return true;
}
