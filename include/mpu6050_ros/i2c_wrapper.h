// Copyright here

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class I2CWrapper
{
  public:
    I2CWrapper();

    ~I2CWrapper();

    uint8_t initI2C();

    uint8_t deinitI2C();

    uint8_t isDataAvailable();

    uint8_t readData();

    uint8_t writeData();

  private:

}

#endif
