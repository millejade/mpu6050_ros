// Copyright here

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>

class I2CWrapper
{
  public:
    I2CWrapper();

    ~I2CWrapper();

    bool initI2CSlave(std::string i2c_dev_path, uint8_t slave_addr);

    bool deinitI2CSlave();

    bool i2cReadByte(uint8_t register_addr, int16_t* buf);

    bool i2cWriteByte(uint8_t register_addr, uint16_t val);

  private:
    in8_t i2c_fd_;
}

#endif
