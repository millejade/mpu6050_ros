// Copyright here

#ifndef MPU6050_DECODER_H
#define MPU6050_DECODER_H

#include "i2c_wrapper.h"
#include <map>

enum class MPU6050_REGISTER_ADDR
{
  FSYNC_DLPF_CONFIGURATION = 0x1A
  GYRO_CONFIGURATION = 0x1B,
  ACCEL_CONFIGURATION = 0x1C,
  POWER_MANAGEMENT_1 = 0x6B,
  POWER_MANAGEMENT_2 = 0x6C
};

enum class GYRO_CONFIG // Refer to datasheet
{
  FS_SEL_250 = 0b00000000,
  FS_SEL_500 = 0b00001000,
  FS_SEL_1000 = 0b00010000,
  FS_SEL_2000 = 0b00011000
  // TODO: Gyro Axes Test
};

enum GYRO_CONFIG_CHOICE
{
  250_DEG_PER_S = 250,
  500_DEG_PER_S = 500,
  1000_DEG_PER_S = 1000,
  2000_DEG_PER_S = 2000
};

enum class ACCEL_CONFIG // Refer to datasheet
{
  AFS_SEL_2G = 0b00000000,
  AFS_SEL_4G = 0b00001000,
  AFS_SEL_8G = 0b00010000,
  AFS_SEL_16G = 0b00011000
  // TODO: Accel Axes Test
};

enum ACCEL_CONFIG_CHOICE
{
  2_ACCEL_GRAV = 2,
  4_ACCEL_GRAV = 4,
  8_ACCEL_GRAV = 8,
  16_ACCEL_GRAV = 16
};

enum class PWR_MGMT_1 // Refer to datasheet
{
  INTERNAL_OSCI_8MHZ = 0;
  PLL_X_GYRO_REF,
  PLL_Y_GYRO_REF,
  PLL_Z_GYRO_REF,
  PLL_EXT_32768HZ_REF,
  PLL_EXT_19_2MHZ_REF,
  STOP_CLK_TIMING_GEN_RST = 7,
  DISABLE_TEMPERATURE,
  CYCLE = 32,
  SLEEP = 64,
  DEVICE_RESET = 128
};

enum class PWR_MGMT_1 // Register 0x6B (107)
{
  INTERNAL_OSCI_8MHZ = 0;
  PLL_X_GYRO_REF,
  PLL_Y_GYRO_REF,
  PLL_Z_GYRO_REF,
  PLL_EXT_32768HZ_REF,
  PLL_EXT_19_2MHZ_REF,
  STOP_CLK_TIMING_GEN_RST = 7,
  DISABLE_TEMPERATURE,
  CYCLE = 32,
  SLEEP = 64,
  DEVICE_RESET = 128
};

enum class PWR_MGMT_2 // Register 0x6C (108)
{
  STBY_ZG = 1, // Z-axis gyro in standby mode
  STBY_YG = 2, // Y-axis gyro in standby mode
  STBY_XG = 4, // X-axis gyro in standby mode
  STBY_ZA = 8, // Z-axis accelerometer in standby mode
  STBY_YA = 16, // Y-axis accelerometer in standby mode
  STBY_XA = 64, // X-axis accelerometer in standby mode
  // TODO: LOW-POWER WAKEUP CONTROL
}

#define 

class Mpu6050Decoder
{
  public:
    Mpu6050Decoder();

    ~Mpu6050Decoder();

    bool setI2Cinterface();

    bool closeI2CInterface();

    bool readAccelData();

    bool readGyroData();

    bool readTempData(;

  private:
    bool configI2C();

    bool convertAccelData();

    bool convertGyroData();

    bool convertTempData();

    I2CWrapper* i2c_wraapper;
    bool en_i2c_interface;
    /* 
      Create a struct file for:
        - raw data storage of accel, gyro, and temp
        - converted data storage of above mentioned data
        - union for the raw data structs mentioned
    */

}

#endif
