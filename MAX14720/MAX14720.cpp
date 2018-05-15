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

#include "MAX14720.h"

//******************************************************************************
MAX14720::MAX14720(PinName sda, PinName scl, int slaveAddress) : 
          slaveAddress(slaveAddress) {
  i2c = new I2C(sda, scl);
  isOwner = true;
  clkDivEn = false;
  clkDivSet = 0;
  boostISet = BOOST_ISET_100mA;
  boostMillivolts = 3300;
  boostEn = BOOST_DISABLED;
  boostEMI = false;
  boostInd = false;
  boostHysOff = false;
  boostPasDsc = false;
  boostActDsc = false;
  buckMd = BUCK_BURST;
  buckFst = false;
  buckISet = BUCK_ISET_300mA;
  buckCfg = false;
  buckInd = false;
  buckHysOff = true;
  buckMinOT = true;
  buckInteg = true;
  buckPasDsc = false;
  buckActDsc = false;
  buckFScl = false;
}
//******************************************************************************
MAX14720::MAX14720(I2C *i2c, int slaveAddress) : slaveAddress(slaveAddress) {
  this->i2c = i2c;
  isOwner = false;
  clkDivEn = false;
  clkDivSet = 0;
  boostISet = BOOST_ISET_100mA;
  boostMillivolts = 3300;
  boostEn = BOOST_DISABLED;
  boostEMI = false;
  boostInd = false;
  boostHysOff = false;
  boostPasDsc = false;
  boostActDsc = false;
  buckMd = BUCK_BURST;
  buckFst = false;
  buckISet = BUCK_ISET_300mA;
  buckCfg = false;
  buckInd = false;
  buckHysOff = true;
  buckMinOT = true;
  buckInteg = true;
  buckPasDsc = false;
  buckActDsc = false;
  buckFScl = false;
}
//******************************************************************************
MAX14720::~MAX14720() {
  if (isOwner == true) {
    delete i2c;
  }
}

//******************************************************************************
int MAX14720::boostSetMode(boostEn_t mode) {
  int result;
  char data;
  boostEn = mode;
  data = (boostEn << 3) | (boostEMI << 1) | (boostInd);
  result = writeReg(REG_BOOST_CFG, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  return 0;
}

//******************************************************************************
int MAX14720::boostSetVoltage(int mV) {
  int result;
  char data;
  result = MAX14720_NO_ERROR;
  if ((MAX14720_BOOST_MIN_MV <= mV) && (mV <= MAX14720_BOOST_MAX_MV)) {
    boostMillivolts = mV;
    data = (mV - MAX14720_BOOST_MIN_MV) / MAX14720_BOOST_STEP_MV;
  } else {
    return MAX14720_ERROR;
  }
  if (boostEn == BOOST_ENABLED) {
    result = writeReg(REG_BOOST_CFG, 0x00);
  }
  if (result == MAX14720_ERROR) {
    return result;
  }
  result = writeReg(REG_BOOST_VSET, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  if (boostEn == BOOST_ENABLED) {
    data = (boostEn << 3) | (boostEMI << 1) | (boostInd);
    result = writeReg(REG_BOOST_CFG, data);
  }
  if (result == MAX14720_ERROR) {
    return result;
  }
  return 0;
}

//******************************************************************************
int MAX14720::init() {
  int result;
  char data;
  data = (clkDivEn << 7) | (clkDivSet);
  result = writeReg(REG_BOOST_CDIV, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  data = (boostISet);
  result = writeReg(REG_BOOST_ISET, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  if ((MAX14720_BOOST_MIN_MV <= boostMillivolts) &&
      (boostMillivolts <= MAX14720_BOOST_MAX_MV)) {
    data = (boostMillivolts - MAX14720_BOOST_MIN_MV) / MAX14720_BOOST_STEP_MV;
  } else {
    return MAX14720_ERROR;
  }
  result = writeReg(REG_BOOST_VSET, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  data = (buckMd << 1) | (buckFst);
  result = writeReg(REG_BUCK_CFG, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  data = (boostHysOff << 7) | (boostPasDsc << 6) | (boostActDsc << 5) |
         (buckPasDsc << 2) | (buckActDsc << 1) | (buckFScl);
  result = writeReg(REG_BBB_EXTRA, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  // Write Boost Enable Register Last
  data = (boostEn << 3) | (boostEMI << 1) | (boostInd);
  result = writeReg(REG_BOOST_CFG, data);
  if (result == MAX14720_ERROR) {
    return result;
  }
  return 0;
}

//******************************************************************************
int MAX14720::monSet(monCfg_t monCfg) {
  int result;
  result = writeReg(REG_MON_CFG, monCfg);
  if (result == MAX14720_ERROR) {
    return result;
  }
  return 0;
}

//******************************************************************************
int MAX14720::shutdown() {
  int result;
  result = writeReg(REG_PWR_OFF, 0xB2);
  if (result == MAX14720_ERROR) {
    return result;
  }
  return 0;
}

//******************************************************************************
int MAX14720::writeReg(registers_t reg, char value) {
  int result;
  char cmdData[2] = {(char)reg, value};
  result = i2c->write(slaveAddress, cmdData, 2);
  if (result != 0) {
    return MAX14720_ERROR;
  }
  return MAX14720_NO_ERROR;
}

//******************************************************************************
int MAX14720::readReg(registers_t reg, char *value) {
  int result;
  char cmdData[1] = {(char)reg};

  result = i2c->write(slaveAddress, cmdData, 1);
  if (result != 0) {
    return MAX14720_ERROR;
  }
  result = i2c->read(slaveAddress, value, 1);
  if (result != 0) {
    return MAX14720_ERROR;
  }
  return MAX14720_NO_ERROR;
}
