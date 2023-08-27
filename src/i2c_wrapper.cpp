

#include <ros/console.h>
#include "i2c_wrapper.h"

I2CWrapper::I2CWrapper()
{
  ROS_INFO_STREAM("@ " << __func__ << ": Created an object.");
}

I2CWrapper::~I2CWrapper()
{
  ROS_INFO_STREAM("@ " << __func__ << ": Object has been destroyed.");
}

bool I2CWrapper::initI2CSlave(std::string i2c_dev_path, uint8_t slave_addr)
{
  bool b_continue = false;
  int8_t ioctl_ret = -1;

  i2c_fd = open(i2c_dev_path, 0_RDWR);

  if (i2c_fd != -1)
  {
    ROS_INFO_STREAM("@ " << __func__ << ": Successfully set the file descriptor.");
    b_continue = true;
  }
  else
  {
    ROS_ERROR_STREAM("@ " << __func__ << ": " << (std::string)strerror(errno));
  }

  if (b_continue)
  {
    ioctl_ret = ioctl(i2c_fd, I2C_SLAVE, slave_addr);

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
  i2c_fd = -1;
}

bool I2CWrapper::i2cReadByte(uint8_t register_addr, uint8_t* buf)
{
  bool b_continue = false;
  int8_t write_ret = 0;
  int8_t read_ret = 0;
  
  write_ret = write(i2c_fd, &register_addr, 1);

  if (write_ret == 1)
  {
    ROS_DEBUG("@ %s: Successfully locked to register address [ %p ]", __func__, static_cast<int>(register_addr));
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

  if (b_continue)
  {
    read_ret = read(i2c_fd, buf, 1);

    if (read_ret == 1)
    {
      ROS_DEBUG("@ %s: Successfully read register address [ %p ]", __func__, static_cast<int>(register_addr));
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
  
  write_ret = write(i2c_fd, buf, 2);

  if (write_ret == 1)
  {
    ROS_DEBUG("@ %s: Successfully locked to register address [ %p ]", __func__, static_cast<int>(register_addr));
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

  return b_continue;
}
