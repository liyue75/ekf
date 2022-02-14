/*
 * Copyright (C) 2015 Freie Universität Berlin
 *               2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mpu9x50
 * @{
 *
 * @file
 * @brief       Register and bit definitions for the MPU-9X50 (MPU9150 and MPU9250) 9-Axis Motion Sensor
 *
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Jannes Volkens <jannes.volkens@haw-hamburg.de>
 */

#ifndef MPU9X50_REGS_H
#define MPU9X50_REGS_H


#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @name    MPU-9X50 register definitions
 * @{
 */
#define MPU9X50_YG_OFFS_TC_REG          (0x01)
#define MPU9X50_RATE_DIV_REG            (0x19)
     #  define MPUREG_SMPLRT_1000HZ      0X00
     #  define MPUREG_SMPLRT_500HZ       0X01
     #  define MPUREG_SMPLRT_250HZ       0X03
     #  define MPUREG_SMPLRT_200HZ       0X04
     #  define MPUREG_SMPLRT_100HZ      0X09
     #  define MPUREG_SMPLRT_50HZ       0X13
#define MPU9X50_LPF_REG                 (0x1A)
#  define MPUREG_CONFIG_EXT_SYNC_SHIFT       3
#  define MPUREG_CONFIG_EXT_SYNC_GX     0X02
     #define MPUREG_CONFIG_EXT_SYNC_GY   0X03
     #define MPUREG_CONFIG_EXT_SYNC_GZ   0X04
     #define MPUREG_CONFIG_EXT_SYNC_AX   0X05
     #define MPUREG_CONFIG_EXT_SYNC_AY   0X06
#define MPUREG_CONFIG_EXT_SYNC_AZ   0X07
     #define MPUREG_CONFIG_FIFO_MODE_STOP 0X40
#define MPU9X50_GYRO_CFG_REG            (0x1B)
#       define BITS_GYRO_FS_250DPS       0X00
#       define BITS_GYRO_FS_500DPS     0X08
#       define BITS_GYRO_FS_1000DPS    0X10
#       define BITS_GYRO_FS_2000DPS    0X18
#       define BITS_GYRO_FS_MASK    0X18
#       define BITS_GYRO_ZGYRO_SELFTEST    0X20
#       define BITS_GYRO_YGYRO_SELFTEST    0X40
#       define BITS_GYRO_XGYRO_SELFTEST    0X80
#define MPU9X50_ACCEL_CFG_REG           (0x1C)
#define MPU9X50_ACCEL_CFG_REG2          0x1D
#       define ICM_ACC_FCHOICE_B        0x08
#       define ICM_ACC_DLPF_CFG_1046HZ_NOLPF 0x00
#       define ICM_ACC_DLPF_CFG_218HZ  0x01
#       define ICM_ACC_DLPF_CFG_99HZ   0x02
#       define ICM_ACC_DLPF_CFG_44HZ    0x03
#define MPU9X50_FIFO_EN_REG             (0x23)
#       define BIT_TEMP_FIFO_EN    0X80
#       define BIT_XG_FIFO_EN    0X40
#       define BIT_YG_FIFO_EN    0X20
#       define BIT_ZG_FIFO_EN    0X10
#       define BIT_ACCEL_FIFO_EN    0X08
#       define BIT_SLV2_FIFO_EN    0X04
#       define BIT_SLV1_FIFO_EN    0X02
#       define BIT_SLV0_FIFI_EN0    0X01
#define MPU9X50_I2C_MST_REG             (0x24)
#       define BIT_I2C_MST_P_NSR    0X10
#       define BIT_I2C_MST_CLK_400KHZ    0X0D
#define MPU9X50_SLAVE0_ADDR_REG         (0x25)
#define MPU9X50_SLAVE0_REG_REG          (0x26)
#define MPU9X50_SLAVE0_CTRL_REG         (0x27)
#define MPU9X50_SLAVE1_ADDR_REG         (0x28)
#define MPU9X50_SLAVE1_REG_REG          (0x29)
#define MPU9X50_SLAVE1_CTRL_REG         (0x2A)
#define MPU9X50_SLAVE4_CTRL_REG         (0x34)
#define MPU9X50_INT_PIN_CFG_REG         (0x37)
#       define BIT_BYPASS_EN    0X02
#       define BIT_INT_RD_CLEAR    0x10
#       define BIT_LATCH_INT_EN    0X20
#define MPU9X50_INT_ENABLE_REG          (0x38)
#       define BIT_RAW_RDY_EN    0X01
#       define BIT_DMP_INT_EN    0X02
#       define BIT_UNKNOWN_INT_EN    0X04
#       define BIT_I2C_MST_INT_EN    0X08
#       define BIT_FIFO_OFLOW_EN    0x10
#      define BIT_ZMOT_EN     0X20
#       define BIT_MOT_EN    0X40
#       define BIT_FF_EN    0X80
#define MPU9X50_DMP_INT_STATUS          (0x39)
#       define BIT_RAW_RDY_INT    0X01
#       define BIT_DMP_INT    0X02
#       define BIT_UNKNOWN_INT    0X04
#       define BIT_I2C_MST_INT    0X08
#       define BIT_FIFO_OFLOW_INT    0X10
#       define BIT_ZMOT_INT    0X20
#       define BIT_MOT_INT    0X40
     #       define BIT_FF_INT    0X80
#define MPU9X50_INT_STATUS              (0x3A)
#define MPU9X50_ACCEL_START_REG         (0x3B)
#define MPU9X50_TEMP_START_REG          (0x41)
#define MPU9X50_GYRO_START_REG          (0x43)
#define MPU9X50_EXT_SENS_DATA_START_REG (0x49)
#define MPU9X50_COMPASS_DATA_START_REG  (0x4A)
#define MPU9X50_SLAVE0_DATA_OUT_REG     (0x63)
#define MPU9X50_SLAVE1_DATA_OUT_REG     (0x64)
#define MPU9X50_SLAVE2_DATA_OUT_REG     (0x65)
#define MPU9X50_SLAVE3_DATA_OUT_REG     (0x66)
#define MPU9X50_I2C_DELAY_CTRL_REG      (0x67)
#define MPU9X50_USER_CTRL_REG           (0x6A)
     #  define BIT_USER_CTRL_SIG_COND_RESET    (0x01) //resets signal paths and results registers for all sensors
     #  define BIT_USER_CTRL_I2C_MST_RESET     (0x02) //reset I2C Master (only applicable if I2C_MST_EN bit is set)
     #  define BIT_USER_CTRL_FIFO_RESET        (0x04) //Reset FIFO buffer
     #  define BIT_USER_CTRL_DMP_RESET         (0x08) //Reset DMP
     #  define BIT_USER_CTRL_I2C_IF_DIS        (0x10) //Disable primary I2C interface and enable hal.spi->interface
     #  define BIT_USER_CTRL_I2C_MST_EN        (0x20) //Enable MPU to act as the I2C Master to external slave sensors
     #  define BIT_USER_CTRL_FIFO_EN           (0x40) //Enable FIFO operations
     #  define BIT_USER_CTRL_DMP_EN            (0x80) //Enable DMP operations
#define MPU9X50_PWR_MGMT_1_REG          (0x6B)
     #  define BIT_PWR_MGMT_1_CLK_INTERNAL     (0x00) // clock set to internal 8Mhz oscillator
     #  define BIT_PWR_MGMT_1_CLK_XGYRO        (0x01) // PLL with X axis gyroscope reference
     #  define BIT_PWR_MGMT_1_CLK_YGYRO        0x02 //PLL with Y axis gyroscope reference
     #  define BIT_PWR_MGMT_1_CLK_ZGYRO        0x03
     #  define BIT_PWR_MGMT_1_CLK_EXT32KHZ     0x04 // PLL with external 32.768kHz reference
     #  define BIT_PWR_MGMT_1_CLK_EXT19HMZ     0x05
     #  define BIT_PWR_MGMT_1_CLK_STOP         0x07
     #  define BIT_PWR_MGMT_1_TEMP_DIS         0x08
     #  define BIT_PWR_MGMT_1_CYCLE            0x20
     #  define BIT_PWR_MGMT_1_SLEEP            0x40
     #  define BIT_PWR_MGMT_1_DEVICE_RESET     0x80
#define MPU9X50_PWR_MGMT_2_REG          (0x6C)
#define MPU9X50_FIFO_COUNT_START_REG    (0x72)
#define MPU9X50_FIFO_RW_REG             (0x74)
#define MPU9X50_WHO_AM_I_REG            (0x75)
/** @} */

#define MPUREG_ACC_OFF_X_H        0x77
#define MPUREG_ACC_OFF_X_L        0x78
#define MPUREG_ACC_OFF_Y_H        0x7a
#define MPUREG_ACC_OFF_Y_L        0x7b
#define MPUREG_ACC_OFF_Z_H        0x7d
#define MPUREG_ACC_OFF_Z_L        0x7e

 /**
  * @name    Compass register definitions
  * @{
  */
#define COMPASS_WHOAMI_REG              (0x00)
#define COMPASS_ST1_REG                 (0x02)
#define COMPASS_DATA_START_REG          (0x03)
#define COMPASS_ST2_REG                 (0x09)
#define COMPASS_CNTL_REG                (0x0A)
#define COMPASS_ASTC_REG                (0x0C)
#define COMPASS_ASAX_REG                (0x10)
#define COMPASS_ASAY_REG                (0x11)
#define COMPASS_ASAZ_REG                (0x12)
/** @} */

/**
 * @name    MPU9X50 bitfield definitions
 * @{
 */
#define BIT_SLV0_DELAY_EN               (0x01)
#define BIT_SLV1_DELAY_EN               (0x02)
#define BIT_I2C_BYPASS_EN               (0x02)
#define BIT_I2C_MST_EN                  (0x20)
#define BIT_PWR_MGMT1_SLEEP             (0x40)
#define BIT_WAIT_FOR_ES                 (0x40)
#define BIT_I2C_MST_VDDIO               (0x80)
#define BIT_SLAVE_RW                    (0x80)
#define BIT_SLAVE_EN                    (0x80)
#define BIT_DMP_EN                      (0x80)
/** @} */

// WHOAMI values
#define MPU_WHOAMI_MPU9250              (0x71)

#define BITS_DLPF_CFG_256HZ_NOLPF2 0x00
#define BITS_DLPF_CFG_188HZ 0x01
#define BITS_DLPF_CFG_98HZ 0x02
#define BITS_DLPF_CFG_42HZ 0x03
#define BITS_DLPF_CFG_20HZ 0x04
#define BITS_DLPF_CFG_10HZ 0x05
#define BITS_DLPF_CFG_5HZ 0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF 0x07
#define BITS_DLPF_CFG_MASK 0x07

#ifdef __cplusplus
}
#endif

#endif /* MPU9X50_REGS_H */
/** @} */
