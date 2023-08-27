// Copyright here

#ifndef MPU6050_DECODER_H
#define MPU6050_DECODER_H

#include "i2c_wrapper.h"

class Mpu6050Decoder
{
  public:
    Mpu6050Decoder();

    ~Mpu6050Decoder();

    bool setI2Cinterface();

    bool closeI2CInterface();

    bool readAccelData();

    bool readGyroData();
-
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
