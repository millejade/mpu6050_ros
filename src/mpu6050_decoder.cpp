

#include <ros/console.h>
#include "mpu6050_decoder.h"

Mpu6050Decoder::Mpu6050Decoder()
{

}

Mpu6050Decoder::~Mpu6050Decoder()
{

}

bool Mpu6050Decoder::setI2Cinterface()
{

}

bool Mpu6050Decoder::configI2C(std::map<std::string, uint16_t>* config_map)
{
  bool b_continue = false;
  
  GYRO_CONFIG gyro_config;
  ACCEL_CONFIG accel_config;
  PWR_MGMT_1 clk_src_config;

  // Configure Power Management- Clock Source Initial Setup
  clk_src_config = static_cast<PWR_MGMT_1>(config_map["CLK_SRC_CONFIG"]);

  // Configure Gyroscope Full Scale Range
  if (config_map["GYRO_CONFIG"] == GYRO_CONFIG_CHOICE::250_DEG_PER_S)
  {
    gyro_config = GYRO_CONFIG::FS_SEL_250;
  }
  else if (config_map["GYRO_CONFIG"] == GYRO_CONFIG_CHOICE::500_DEG_PER_S)
  {
    gyro_config = GYRO_CONFIG::FS_SEL_500;
  }
  else if (config_map["GYRO_CONFIG"] == GYRO_CONFIG_CHOICE::1000_DEG_PER_S)
  {
    gyro_config = GYRO_CONFIG::FS_SEL_1000;
  }
  else if (config_map["GYRO_CONFIG"] == GYRO_CONFIG_CHOICE::2000_DEG_PER_S)
  {
    gyro_config = GYRO_CONFIG::FS_SEL_2000;
  }
  else // Should not be executed, handled on ROS params already - Default value
  {
    gyro_config = GYRO_CONFIG::FS_SEL_250;
  }

  // Configure Accelerometer Full Scale Range
  if (config_map["ACCEL_CONFIG"] == 2_ACCEL_GRAV)
  {
    accel_config = ACCEL_CONFIG::AFS_SEL_2G;
  }
  else if (config_map["ACCEL_CONFIG"] == 4_ACCEL_GRAV)
  {
    accel_config = ACCEL_CONFIG::AFS_SEL_4G;
  }
  else if (config_map["ACCEL_CONFIG"] == 8_ACCEL_GRAV)
  {
    accel_config = ACCEL_CONFIG::AFS_SEL_8G;
  }
  else if (config_map["ACCEL_CONFIG"] == 16_ACCEL_GRAV)
  {
    accel_config = ACCEL_CONFIG::AFS_SEL_16G;
  }
  else // Should not be executed, handled on ROS params already - Default value
  {
    accel_config = ACCEL_CONFIG::AFS_SEL_2G;
  }

  // Set Power Management - TODO: ROS Parameters for other value
  b_continue = i2cWriteByte(static_cast<uint8_t>(MPU6050_REGISTER_ADDR::POWER_MANAGEMENT_1), \
                            static_cast<uint16_t>(clk_src_config));
  
  // Set Gyroscope Configuration - TODO: ROS Parameters for other value
  if (b_continue)
  {
    ROS_INFO_STREAM("@ " << __func__ << ": Successfully configure gyroscope.");
    b_continue = i2cWriteByte(static_cast<uint8_t>(MPU6050_REGISTER_ADDR::GYRO_CONFIGURATION), \
                              static_cast<uint16_t>(gyro_config));
  }
  else
  {
    ROS_ERROR_STREAM("@ " << __func__ << ": Failed to configure power management.");
  }

  // Set Accelerator Configuration - TODO: ROS Parameters for other value
  if (b_continue)
  {
    ROS_INFO_STREAM("@ " << __func__ << ": Successfully configure accelerometer.");
    b_continue = i2cWriteByte(static_cast<uint8_t>(MPU6050_REGISTER_ADDR::ACCEL_CONFIGURATION), \
                              static_cast<uint16_t>(accel_config));
  }
  else
  {
    ROS_ERROR_STREAM("@ " << __func__ << ": Failed to configure gyroscope.");
  }

  // No configuration needed for temperature

}

bool Mpu6050Decoder::readAccelData()
{

}

bool Mpu6050Decoder::readGyroData()
{

}

bool Mpu6050Decoder::readTempData()
{

}

bool Mpu6050Decoder::convertAccelData()
{

}

bool Mpu6050Decoder::convertGyroData()
{

}

bool Mpu6050Decoder::convertTempData()
{

}