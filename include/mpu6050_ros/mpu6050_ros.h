// Copyright here

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include "mpu6050_decoder.h"

class MPU6050ROS
{
  public:
    MPU6050ROS();

    ~MPU6050ROS();

    bool initMPU6050();

    bool initROSParams();

  private:
    // Sensor Configuration
    uint8_t pwr_mgmt_config;
    uint8_t gyro_config;
    uint8_t accel_config;

    // Enabled Topics
    
}

#endif
