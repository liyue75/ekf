#include <stdlib.h>
#include <string.h>
#include "mpu9250.h"
#include "mpu9250_regs.h"
#include "mpu9250_params.h"
#include "xtimer.h"
#include "thread.h"
#include "xtimer.h"
#include "fusion_math.h"
#include "definitions.h"
#include "spi_device.h"
#include "check_reg.h"
#include "backend.h"
#include "fusion_math.h"
#include "definitions.h"
#include "kernel_defines.h"
#include "rotation.h"
#include "device.h"
#include "uart_device.h"
#include "check_reg.h"
#include "vector3f.h"
#include "inertial_sensor.h"
//include "lowpass_filter2p.h"
#include "lowpass_filter_v3f.h"
#include "board_led.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_BUFFER_LEN 8

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2 * idx] << 8) | v[2 * idx + 1]))
#define uint16_val(v, idx) (((uint16_t)v[2 * idx] << 8) | v[2 * idx + 1])

static struct {
    vector3f_t accel;
    vector3f_t gyro;
    uint8_t accel_count;
    uint8_t gyro_count;
    lpf_v3f_t accel_lpf;
} accum;

bool _mpu9250_have_init = false;
static uint32_t last_reset_ms;
uint8_t _last_stat_user_ctrl;
static bool enable_offset_checking;
extern float _temp_zero;
extern float _temp_sensitivity;
static float gyro_raw_sample_rates;
extern float _accel_raw_sample_rates;
extern float _gyro_raw_sample_rates;
static uint8_t gyro_fifo_downsample_rate;
static uint8_t accel_fifo_downsample_rate;
static uint8_t gyro_to_accel_sample_ratio;
static bool accel_sensor_rate_sampling_enabled;
static bool gyro_sensor_rate_sampling_enabled;
static uint16_t gyro_backend_rate_hz;
static uint16_t accel_backend_rate_hz;
static Rotation_t gyro_orientation;
static Rotation_t accel_orientation;
static float fifo_accel_scale;
static float fifo_gyro_scale;
static float accel_scale;
static float gyro_scale;
static uint32_t accel_error_count;
static uint32_t gyro_error_count;
static int16_t raw_temp;
extern float _temperature;
extern uint32_t _accel_clip_count;
extern float _temp_filtered;

static bool enable_fast_sampling = true;
static const uint16_t multiplier_accel = INT16_MAX / (26 * GRAVITY_MSS);
float _clip_limit = 15.5f * GRAVITY_MSS;

static uint8_t fifo_buffer[MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE];

static bool block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return read_registers(&_mpu9250_spi_params[0], reg, buf, size);
}

static uint8_t register_read(uint8_t reg)
{
    uint8_t val = 0;
/*
    DEBUG("reg = 0x%x, &_mpu9250_spi_params[0]->spi = %d \
          , cs = %lu, clk = %lu, mode = %d, flag = 0x%x\n", reg, \
          (_mpu9250_spi_params[0]).spi, \
          (_mpu9250_spi_params[0]).cs, \
          (&_mpu9250_spi_params[0])->clk, \
          (&_mpu9250_spi_params[0])->mode, \
          (&_mpu9250_spi_params[0])->read_flag \
    );
*/
    read_registers(&_mpu9250_spi_params[0], reg, &val, 1);
    return val;
}

static void register_write(uint8_t reg, uint8_t val)
{
    write_register(&_mpu9250_spi_params[0], reg, val);
}

static void register_write_check(uint8_t reg, uint8_t val)
{
    set_checked_register(reg, val);
    write_register(&_mpu9250_spi_params[0], reg, val);
}

static bool check_whoami(void)
{
    uint8_t whoami = register_read(MPU9X50_WHO_AM_I_REG);
    DEBUG("whoami = 0x%x\n", whoami);
    if (whoami == MPU_WHOAMI_MPU9250) {
        return true;
    }
    return false;
}

__attribute__((unused)) static bool mpu9250_hardware_init(void)
{
    if (!check_whoami()) {
        return false;
    }
    //xtimer_usleep(10000);
    /*
    for(uint8_t i=0; i< 10; i++) {
        check_whoami();
        xtimer_usleep(10000);
    }
*/
    //chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        _last_stat_user_ctrl = register_read(MPU9X50_USER_CTRL_REG);
        DEBUG("_last_stat_user_ctrl = 0x%x\n", _last_stat_user_ctrl);
        if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            register_write(MPU9X50_USER_CTRL_REG, _last_stat_user_ctrl);
            xtimer_usleep(10000);
        }
        /* reset device */
        register_write(MPU9X50_PWR_MGMT_1_REG, BIT_PWR_MGMT_1_DEVICE_RESET);
        xtimer_usleep(100000);
        //mpu spi initialize
        _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
        register_write(MPU9X50_USER_CTRL_REG, _last_stat_user_ctrl);
        //wake up device and select GyroZ clock
        register_write(MPU9X50_PWR_MGMT_1_REG, BIT_PWR_MGMT_1_CLK_ZGYRO);
        xtimer_usleep(5000);
        if (register_read(MPU9X50_PWR_MGMT_1_REG) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            DEBUG(" pll clock selected\n");
            break;
        }
        xtimer_usleep(10000);
        if ((register_read(MPU9X50_INT_STATUS) & BIT_RAW_RDY_INT) != 0) {
            DEBUG("data ready\n");
            break;
        }
    }
    //register_write(MPU9X50_USER_CTRL_REG, 0x34);
    //xtimer_usleep(100000);
    //_ = register_read(MPU9X50_USER_CTRL_REG);
    //DEBUG("after tries,  = 0x%x\n", _last_stat_user_ctrl);
    if (tries == 5) {
        DEBUG("failed to boot mpu9250 5 times\n");
        return false;
    }

    //check_whoami();
    return true;
}

bool mpu9250_init(void)
{
    bool success = true;
    spi_init(_mpu9250_spi_params[0].spi);
    int ret  = spi_init_cs(_mpu9250_spi_params[0].spi, _mpu9250_spi_params[0].cs);
    if (ret != 0) {
        DEBUG("[mpu9250] spi cs init error, ret = %d", ret);
        return false;
    }
    success = mpu9250_hardware_init();
    DEBUG("[mpu9250] hardware init: success = %d\n", success);
    _mpu9250_have_init = true;
    return success;
}


static void set_accel_raw_sample_rate(uint16_t rate_hz)
{
    _accel_raw_sample_rates = rate_hz;
}

static void set_gyro_raw_sample_rate(uint16_t rate_hz)
{
    gyro_raw_sample_rates = rate_hz;
}

static void fifo_reset(void)
{
    uint32_t now = xtimer_now().ticks32 / 1000;
    last_reset_ms = now;
    uint8_t user_ctrl = _last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_RESET | BIT_USER_CTRL_FIFO_EN);
    __attribute__((unused))uint8_t val = register_read(MPU9X50_FIFO_EN_REG);
    //DEBUG("user_ctrl = 0x%x, fifo en reg = 0x%x\n", user_ctrl, val);
    register_write(MPU9X50_FIFO_EN_REG, 0);
    xtimer_usleep(1000);
    val = register_read(MPU9X50_FIFO_EN_REG);
    //DEBUG("after write: fifo en reg = 0x%x\n", val);
    register_write(MPU9X50_USER_CTRL_REG, user_ctrl);
    register_write(MPU9X50_USER_CTRL_REG, user_ctrl | BIT_USER_CTRL_FIFO_RESET);
    register_write(MPU9X50_USER_CTRL_REG, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    register_write(MPU9X50_FIFO_EN_REG, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                   BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN);
    xtimer_usleep(1000);
    val = register_read(MPU9X50_USER_CTRL_REG);
    //DEBUG("user ctrl reg = 0x%x\n", val);
    _last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;
    notify_accel_fifo_reset();
    notify_gyro_fifo_reset();
}

static void set_filter_register(void)
{
    uint8_t config = 0;
    gyro_fifo_downsample_rate = accel_fifo_downsample_rate = 1;
    gyro_to_accel_sample_ratio = 2;
    gyro_backend_rate_hz = accel_backend_rate_hz = 1000;
    if (enable_fast_sampling) {
        uint8_t loop_limit = 1;
        if (get_ins_loop_rate_hz() > 1000) {
            loop_limit = 2;
        }
        if (get_ins_loop_rate_hz() > 2000) {
            loop_limit = 4;
        }
        uint8_t fast_sampling_rate = constrain_value(get_fast_sampling_rate(), loop_limit, 8);
        gyro_fifo_downsample_rate = 8 / fast_sampling_rate;
        gyro_backend_rate_hz *= fast_sampling_rate;
        accel_fifo_downsample_rate = MAX(4 / fast_sampling_rate, 1);
        accel_backend_rate_hz *= MIN(fast_sampling_rate, 4);

        init_lpf_v3f(&accum.accel_lpf);
        set_cutoff_freq_v3f_sam(4000, 188, &accum.accel_lpf);
        DEBUG("accel down sample = %d, backend rate = %d, \
              gyro down sample = %d, backend = %d\n", accel_fifo_downsample_rate,
              accel_backend_rate_hz, gyro_fifo_downsample_rate, gyro_backend_rate_hz);

        DEBUG("accum.gyro_count = %d, gyro_to_accel_sample_ratio = %d, a\\g = %d\n",
              accum.gyro_count,  gyro_to_accel_sample_ratio,
              accum.gyro_count % gyro_to_accel_sample_ratio);
        set_accel_oversampling(accel_fifo_downsample_rate);
        set_gyro_oversampling(gyro_fifo_downsample_rate);
        accel_sensor_rate_sampling_enabled = true;
        gyro_sensor_rate_sampling_enabled = true;
        register_write(MPU9X50_SLAVE4_CTRL_REG, 0x1F);
        config |= BITS_DLPF_CFG_256HZ_NOLPF2;
    }
    config |= MPUREG_CONFIG_FIFO_MODE_STOP;
    register_write_check(MPU9X50_LPF_REG, config);
    register_write_check(MPU9X50_ACCEL_CFG_REG2, ICM_ACC_FCHOICE_B);
}


__attribute__((unused))static bool check_raw_temp(__attribute__((unused))int16_t t2)
{
    if (abs(t2 - raw_temp) < 400) {
        return true;
    }
    uint8_t trx[2];
    if (block_read(MPU9X50_TEMP_START_REG, trx, 2)) {
        raw_temp = int16_val(trx, 0);
    }
    MY_LOG("raw temp= %d, t2 = %d\n", raw_temp, t2);
    return (abs(t2 - raw_temp) < 800);
}

static bool accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    __attribute__((unused))const int32_t _unscaled_clip_limit = _clip_limit / accel_scale;
    __attribute__((unused))bool clipped = false;
    bool ret = true;
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;
        uint16_t t2 = int16_val(data, 3);
        if (!check_raw_temp(t2)) {
            MY_LOG("check_raw_temp error\n");
            fifo_reset();
            ret = false;
            break;
        }
        tsum += t2;
        if (accum.gyro_count % gyro_to_accel_sample_ratio == 0) {
            //accel data is at 4KHz or 1KHz
            vector3f_t a = {int16_val(data, 1), int16_val(data, 0), -int16_val(data, 2)};
            if (fabs(a.x) > _unscaled_clip_limit ||
                fabs(a.y) > _unscaled_clip_limit ||
                fabs(a.z) > _unscaled_clip_limit) {
                clipped = true;
           }
            vector3f_t filtered_acc = lpf_v3f_apply(&a, &accum.accel_lpf);
            accum.accel.x += filtered_acc.x;
            accum.accel.y += filtered_acc.y;
            accum.accel.z += filtered_acc.z;
            accum.accel_count++;
            if (accum.accel_count % accel_fifo_downsample_rate == 0) {
                accum.accel.x *= fifo_accel_scale;
                accum.accel.y *= fifo_accel_scale;
                accum.accel.z *= fifo_accel_scale;
                rotate_and_correct_accel(&accum.accel, accel_orientation);
                notify_new_accel_raw_sample(&accum.accel, 0, false);
                accum.accel.x = accum.accel.y = accum.accel.z = 0;
                accum.accel_count = 0;
                accum.gyro_count = 0;
            }
        }
        accum.gyro_count++;
        vector3f_t g = {int16_val(data, 5), int16_val(data, 4), -int16_val(data, 6)};
        accum.gyro.x += g.x;
        accum.gyro.y += g.y;
        accum.gyro.z += g.z;
        if (accum.gyro_count % gyro_fifo_downsample_rate == 0) {
            accum.gyro.x *= fifo_gyro_scale;
            accum.gyro.y *= fifo_gyro_scale;
            accum.gyro.z *= fifo_gyro_scale;
            rotate_and_correct_gyro(&accum.gyro, gyro_orientation);
            notify_new_gyro_raw_sample(&accum.gyro, 0);
            v3f_zero(&accum.gyro);
        }
        //if (accum.gyro_count % 10 == 0) {
            //DEBUG("n_samples :%d, 0 1 2: %d, %d, %d\n", n_samples, int16_val(data, 0),
             //     int16_val(data, 1), int16_val(data, 2));
        //}
    }
    if (clipped) {
        _accel_clip_count++;
    }
    if (ret) {
        float temp = ((float)tsum / n_samples) * _temp_sensitivity + _temp_zero;
        //_temp_filtered = lpf2p_apply();
        _temp_filtered = temp;
    }
    return ret;
}

static void poll_data(void)
{
    uint8_t n_samples;
    __attribute((unused))uint16_t bytes_read;
    uint8_t *rx = fifo_buffer;
    __attribute__((unused))bool need_reset = false;
    if (!block_read(MPU9X50_FIFO_COUNT_START_REG, rx, 2)) {
        goto check_registers;
    }
    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / MPU_SAMPLE_SIZE;
    if (n_samples == 0) {
        goto check_registers;
    }
    if (n_samples > 32) {
        need_reset = true;
        n_samples = 24;
    }
    //MY_LOG("%d\n", n_samples);
    while (n_samples > 0) {
        uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);
        memset(rx, 0, n * MPU_SAMPLE_SIZE);
        uint8_t reg = MPU9X50_FIFO_RW_REG | 0X80;
        spi_device_transfer(&_mpu9250_spi_params[0], &reg, 1,
                            rx, n * MPU_SAMPLE_SIZE);
        if (!accumulate_sensor_rate_sampling(rx, n)) {
            break;
        }
        n_samples -= n;
        //MY_LOG("%d \n", ((uint16_t)rx[0] << 8) + rx[1]);
    }
    if (need_reset) {
        //MY_LOG("fifo reset");
        fifo_reset();
    }
    checkreg_t reg;
check_registers:
    if (!check_next_register(&reg)) {
        gyro_error_count++;
        accel_error_count++;
        //led_on(LED_3);
    }
    return;
}

void imu_backend_start(void)
{
    register_write(MPU9X50_PWR_MGMT_2_REG, 0x00);
    xtimer_usleep(1000);
    uint8_t val = register_read(MPU9X50_PWR_MGMT_2_REG);
    DEBUG("mgmt 2 pwr val = 0x%x\n", val);
    fifo_reset();
    enable_offset_checking = true;
    if (!register_gyro(1000) || !register_accel(1000)) {
        return;
    }
    set_filter_register();
    set_accel_raw_sample_rate(accel_backend_rate_hz);
    set_gyro_raw_sample_rate(gyro_backend_rate_hz);
    set_raw_sample_accel_multiplier(multiplier_accel);
    register_write_check(MPU9X50_RATE_DIV_REG, 0);
    xtimer_usleep(1000);
    // Gyro scale 2000 ./s
    register_write_check(MPU9X50_GYRO_CFG_REG, BITS_GYRO_FS_2000DPS);
    xtimer_usleep(1000);
    // Accel scale 16g (2048 LSB/g)
    register_write_check(MPU9X50_ACCEL_CFG_REG, 3 << 3);
    accel_scale = GRAVITY_MSS / 2048.f;
    gyro_scale = (radians(1) / 16.4f);
    xtimer_usleep(1000);
    // configure interrupt to fire when new data arrives
    uint8_t int_reg = register_read(MPU9X50_INT_ENABLE_REG);
    DEBUG("int_reg = 0x%x\n", int_reg);
    register_write(MPU9X50_INT_ENABLE_REG, BIT_RAW_RDY_EN);
    xtimer_usleep(1000);
    uint8_t v = register_read(MPU9X50_INT_PIN_CFG_REG);
    DEBUG("int pin cfg reg = 0x%x\n", v);
    v |=  BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN;
    v &= BIT_BYPASS_EN;
    register_write(MPU9X50_INT_PIN_CFG_REG, v);
    if (enable_offset_checking) {
        uint8_t regs[] = {MPUREG_ACC_OFF_X_H, MPUREG_ACC_OFF_X_L,
        MPUREG_ACC_OFF_Y_H, MPUREG_ACC_OFF_Y_L,
        MPUREG_ACC_OFF_Z_H, MPUREG_ACC_OFF_Z_L};
        for (uint8_t i = 0; i < ARRAY_SIZE(regs); i++) {
            register_write_check(regs[i], register_read(regs[i]));
        }

    }
    set_gyro_accel_orientation(&accel_orientation,
                               &gyro_orientation,
                               ROTATION_NONE);
    //setup scale factors for fifo data after downsampling
    fifo_accel_scale = accel_scale / accel_fifo_downsample_rate;
    fifo_gyro_scale = gyro_scale / gyro_fifo_downsample_rate;
    register_periodic_callback(1000000 / gyro_backend_rate_hz, poll_data);
}

void configure_slaves(void)
{
    if (!(_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN)) {
        _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_MST_EN;
        register_write(MPU9X50_USER_CTRL_REG, _last_stat_user_ctrl);
    }
    register_write(MPU9X50_I2C_MST_REG, BIT_I2C_MST_P_NSR | BIT_I2C_MST_CLK_400KHZ);
    register_write(MPU9X50_SLAVE4_CTRL_REG, 9);
    register_write(MPU9X50_I2C_DELAY_CTRL_REG, BIT_I2C_SLV0_DLY_EN | BIT_I2C_SLV1_DLY_EN |
                   BIT_I2C_SLV2_DLY_EN | BIT_I2C_SLV3_DLY_EN);
}

bool mpu9250_update(void)
{
    backend_update_accel();
    backend_update_gyro();
    backend_publish_temperature(_temp_filtered);
    return true;
}
