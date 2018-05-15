/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */
#ifndef __MAX30205_H_
#define __MAX30205_H_

#include "mbed.h"

/**
 * @brief Driver for the MAX30205 on the HSP Platform
 */

class MAX30205 {
public:
  ///< MAX30205 Register Addresses
  typedef enum Registers {
    MAX30205_Temperature   = 0x00,
    MAX30205_Configuration = 0x01,
    MAX30205_THYST         = 0x02,
    MAX30205_TOS           = 0x03
  } Registers_t;

  /**
  * @brief  Constructor using I2C PinNames
  * @param sda Pinname for sda
  * @param scl Pinname for scl
  */
  MAX30205(PinName sda, PinName scl, int slaveAddress);
  /**
  * @brief  Constructor using pointer to I2C object
  * @param *i2c Pointer to I2C object
  */
  MAX30205(I2C *i2c, int slaveAddress);

  /** @brief Destructor */
  ~MAX30205(void);

  /** @brief Write a register into device at slave address
  * @param reg register address
  * @param value value to write
  */
  int reg_write(char reg, char value);

  /**
  * @brief  Detect the second instance of the MAX30205
  * @param reg register address
  * @param value 8-bit value to writes
  */
  int reg_read(char reg, char *value);

  /**
  * @brief Write a 16-bit value into device at slave address
  * @param reg register address
  * @param value 16-bit value to write
  */
  int reg_write16(char reg, uint16_t value);

  /**
  * @brief Read a 16-bit value from a device at a slave address
  * @param reg register address
  * @param value pointer to store read value
  */
  int reg_read16(char reg, uint16_t *value);

  /**
  * @brief Read the temperature from the device into a 16 bit value
  * @param value pointer to a 16 bit short
  */
  int readTemperature(uint16_t *value);

  /**
  * @brief Read the THYST value from a specified device instance
  * @param value 16-bit pointer of value to read into
  */
  int reg_THYST_Read(uint16_t *value);

  /**
  * @brief Write the THYST to a device instance
  * @param value 16-bit value to write
  */
  int reg_THYST_Write(uint16_t value);

  /**
  * @brief Convert a raw temperature value into a float
  * @param rawTemp raw temperature value to convert
  * @return the convereted value in degrees C
  */
  float toCelsius(unsigned int rawTemp);

  /**
  * @brief Convert the passed in temperature in C to Fahrenheit
  * @param temperatureC Temperature in C to convert
  * @returns Returns the converted Fahrenheit value
  */
  float toFahrenheit(float temperatureC);

private:
  /**
   * @brief I2C pointer
   */
  I2C *i2c;
  /**
   * @brief Is this object the owner of the I2C object
   */
  bool isOwner;
  /**
   * @brief Device slave address
   */
  int slaveAddress;
};

#endif /* __MAX30205_H_ */
