

#include <ros/console.h>
#include "i2c_wrapper.h"

I2CWrapper::I2CWrapper()
{
  ROS_INFO_STREAM("@ " << __func__ << ": Created an object.");
  i2c_fd_ = -1;
}

I2CWrapper::~I2CWrapper()
{
  ROS_INFO_STREAM("@ " << __func__ << ": Object has been destroyed.");
}

bool I2CWrapper::initI2CSlave(std::string i2c_dev_path, uint8_t slave_addr)
{
  ROS_INFO("@ %s: Initializing I2C Slave [ %p ]", __func__, slave_addr);

  bool b_continue = false;
  int8_t ioctl_ret = -1;

  if (i2c_fd_ == -1)
  {
    i2c_fd_ = open(i2c_dev_path, 0_RDWR);
    b_continue = true;
  }
  else
  {
    ROS_ERROR_STREAM("@ " << __func__ << ": I2C interface is already enable.");
  }

  if (b_continue)
  {
    if (i2c_fd_ != -1)
    {
      ROS_INFO_STREAM("@ " << __func__ << ": Successfully set the file descriptor.");
      b_continue = true;
    }
    else
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": " << (std::string)strerror(errno));
    }
  }

  if (b_continue)
  {
    ioctl_ret = ioctl(i2c_fd_, I2C_SLAVE, slave_addr);

    if (ioctl_ret != -1)
    {
      ROS_INFO_STREAM("@ " << __func__ << ": Successfully set the I2C Slave address.");
    }
    else
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": " << (std::string)strerror(errno));
      b_continue = false;
    }
  }

  return b_continue;
}

bool I2CWrapper::deinitI2CSlave()
{
  i2c_fd_ = -1;
}

bool I2CWrapper::i2cReadByte(uint8_t register_addr, int16_t* buf)
{
  bool b_continue = false;
  int8_t write_ret = 0;
  int8_t read_ret = 0;
  
  if (i2c_fd_ == -1)
  {
    write_ret = write(i2c_fd_, &register_addr, 1);
    b_continue = true;
  }
  else
  {
    ROS_ERROR_STREAM("@ " << __func__ << ": I2C interface is still disabled.");
  }

  if (b_continue)
  {
    if (write_ret == 1)
    {
      ROS_DEBUG("@ %s: Successfully locked to register address [ %p ]", __func__, register_addr);
    }
    else if (write_ret == -1)
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": " << (std::string)strerror(errno));
      b_continue = false;
    }
    else
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": Invalid size." << );
      b_continue = false;
    }
  }
  
  if (b_continue)
  {
    read_ret = read(i2c_fd_, buf, 1);

    if (read_ret == 1)
    {
      ROS_DEBUG("@ %s: Successfully read register address [ %p ]", __func__, register_addr);
    }
    else if (read_ret == -1)
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": " << (std::string)strerror(errno));
      b_continue = false;
    }
    else
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": Invalid size." << );
      b_continue = false;
    }
  }

  return b_continue;
}

bool I2CWrapper::i2cWriteByte(uint8_t register_addr, uint8_t val)
{
  bool b_continue = false;
  uint8_t buf[2] = {register_addr, val};

  int8_t write_ret = 0;
  
  if (i2c_fd_ != -1)
  {
    write_ret = write(i2c_fd_, buf, 2);
    b_continue = true;
  }
  else
  {
    ROS_ERROR_STREAM("@ " << __func__ << ": I2C interface is still disabled.");
  }

  if (b_continue)
  {
    if (write_ret == 2)
    {
      ROS_DEBUG("@ %s: Successfully wrote to register address [ %p ]", __func__, register_addr);
      b_continue = true;
    }
    else if (write_ret == -1)
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": " << (std::string)strerror(errno));
    }
    else
    {
      ROS_ERROR_STREAM("@ " << __func__ << ": Invalid size." << );
    }
  }
  
  return b_continue;
}
