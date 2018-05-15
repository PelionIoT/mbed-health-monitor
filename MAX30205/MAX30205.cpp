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
#include "MAX30205.h"

//******************************************************************************
MAX30205::MAX30205(PinName sda, PinName scl, int slaveAddress)
    : slaveAddress(slaveAddress)
{
    i2c = new I2C(sda, scl);
    isOwner = true;
    i2c->frequency(100000);
}

//******************************************************************************
MAX30205::MAX30205(I2C *i2c, int slaveAddress) : slaveAddress(slaveAddress)
{
    this->i2c = i2c;
    i2c->frequency(100000);
    isOwner = false;
}

//******************************************************************************
MAX30205::~MAX30205(void)
{
    if (isOwner == true) {
        delete i2c;
    }
}

//******************************************************************************
int MAX30205::reg_write(char reg, char value)
{
    int result;
    char cmdData[2] = {(char)reg, value};
    result = i2c->write(slaveAddress, cmdData, 2);
    if (result != 0) {
        return -1;
    }
    return 0;
}

//******************************************************************************
int MAX30205::reg_write16(char reg, uint16_t value)
{
    int result;
    char hi = (value >> 8) & 0xFF;
    char lo = value & 0xFF;
    char cmdData[3] = {reg, hi, lo};
    result = i2c->write(slaveAddress, cmdData, 3);
    if (result != 0) {
        return -1;
    }
    return 0;
}

//******************************************************************************
int MAX30205::reg_read(char reg, char *value)
{
    int result;
    char cmdData[1] = {reg};

    result = i2c->write(slaveAddress, cmdData, 1);
    if (result != 0) {
        return -1;
    }
    result = i2c->read(slaveAddress, value, 1);
    if (result != 0) {
        return -1;
    }
    return 0;
}

//******************************************************************************
int MAX30205::reg_read16(char reg, uint16_t *value)
{
    int result;
    char data[2];
    char cmdData[1] = {reg};
    result = i2c->write(slaveAddress, cmdData, 1);
    if (result != 0) {
        return -1;
    }
    result = i2c->read(slaveAddress, data, 2);
    if (result != 0) {
        return -1;
    }
    *value = (data[0] << 8) + data[1];
    return 0;
}

//******************************************************************************
int MAX30205::readTemperature(uint16_t *value)
{
    uint8_t data[2];
    int status;
    status = reg_read16(MAX30205_Temperature, (uint16_t *)&data);
    *value = (data[0] << 8) + data[1];
    return status;
}

//******************************************************************************
float MAX30205::toCelsius(unsigned int rawTemp)
{
    float val;
    float val1, val2;
    val1 = (float)(rawTemp >> 8);
    val2 = (float)(rawTemp & 0xFF);
    val = val2 + (val1 / 256.0f);
    return val;
}

//******************************************************************************
float MAX30205::toFahrenheit(float temperatureC)
{
    return temperatureC * 9.0f / 5.0f + 32.0f;
}

//******************************************************************************
int MAX30205::reg_THYST_Read(uint16_t *value)
{
    return reg_read16(MAX30205_THYST, value);
}

//******************************************************************************
int MAX30205::reg_THYST_Write(uint16_t value)
{
    return reg_write16(MAX30205_THYST, value);
}
