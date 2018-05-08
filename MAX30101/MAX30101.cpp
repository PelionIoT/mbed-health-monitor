/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 
 
#include "MAX30101.h"


//*****************************************************************************
MAX30101::MAX30101(I2C *i2c)
{
    m_i2cBus = i2c;
}


//*****************************************************************************
MAX30101::~MAX30101()
{
    //empty block
}


//*****************************************************************************
int32_t MAX30101::enableInterrupts(const InterruptBitField_u data)
{
    char cmdData[3];
    cmdData[0] = InterruptEnable1;
    cmdData[1] = (data.all & 0xF0);
    cmdData[2] = (data.all & 0x02);

    return m_i2cBus->write(I2C_W_ADRS, cmdData, 3);
}


//*****************************************************************************
int32_t MAX30101::getInterruptStatus(InterruptBitField_u &data)
{
    char local_data[3];
    local_data[0] = InterruptStatus1;
    
    int32_t result = m_i2cBus->write(I2C_W_ADRS, local_data, 1);
    if(result == 0)
    {
        result = m_i2cBus->read(I2C_R_ADRS, (local_data + 1), 2);
        if(result == 0)
        {
            data.all = local_data[1] + local_data[2];
        }
    }
    
    return result;
}


//*****************************************************************************
int32_t MAX30101::setFIFOConfiguration(const FIFO_Configuration_u config)
{
    return writeRegister(FIFO_Configuration, config.all);
}
    

//*****************************************************************************
int32_t MAX30101::getFIFOConfiguration(FIFO_Configuration_u &config)
{
    return readRegister(FIFO_Configuration, config.all);
}
    
    
//*****************************************************************************
int32_t MAX30101::setModeConfiguration(const ModeConfiguration_u config)
{
    return writeRegister(ModeConfiguration, (config.all & 0xC7));
}
    

//*****************************************************************************    
int32_t MAX30101::getModeConfiguration(ModeConfiguration_u &config)
{
    return readRegister(ModeConfiguration, config.all);
}   
    

//*****************************************************************************    
int32_t MAX30101::setSpO2Configuration(const SpO2Configuration_u config)
{
    // Clear Read, Write, and Overflow registers
    writeRegister(FIFO_WritePointer, 0x00);
    writeRegister(FIFO_ReadPointer, 0x00);
    writeRegister(OverflowCounter, 0x00);
    return writeRegister(SpO2Configuration, (config.all & 0x7F));
}   


//*****************************************************************************    
int32_t MAX30101::getSpO2Configuration(SpO2Configuration_u &config)
{
    return readRegister(SpO2Configuration, config.all);
}


//*****************************************************************************  
int32_t MAX30101::setLEDPulseAmplitude(Registers_e reg, const uint8_t amp)
{
    return writeRegister(reg, amp);
}


//*****************************************************************************  
int32_t MAX30101::getLEDPulseAmplitude(Registers_e reg, uint8_t &amp)
{
    return readRegister(reg, amp);
}


//*****************************************************************************  
int32_t MAX30101::setMultiLEDModeControl(Registers_e reg, const ModeControlReg_u data)
{
    return writeRegister(reg, data.all);
}
 
    
//*****************************************************************************  
int32_t MAX30101::getMultiLEDModeControl(Registers_e reg, ModeControlReg_u &data)
{
    return readRegister(reg, data.all);
}

    
//*****************************************************************************  
int32_t MAX30101::getDieTemperature(uint16_t &data)
{
    int32_t result = -1;
    
    //Die temp conversion time is 30ms
    //ATTEMPTS > than 30ms at 100KHz SCL for getInterruptStatus call
    const uint32_t ATTEMPTS = 100; 
    uint32_t num_reads = 0;
    
    //local i2c transaction buffer
    char local_data[2];
    local_data[0] = DieTempConfig;
    local_data[1] = 1;
    
    //initiate die temp conversion
    result = m_i2cBus->write(I2C_W_ADRS, local_data, 2, true);
    if(result == 0)
    {
        //object for holding status registers data
        InterruptBitField_u status;
        status.all = 0;
        
        //poll status registers until temp ready, or read fails
        do
        {
            result = getInterruptStatus(status);
            num_reads++;
        }
        while(!status.bits.die_temp && (result == 0) && (num_reads < ATTEMPTS));
        
        if(status.bits.die_temp)
        {
            //set pointer to temperature integer register
            local_data[0] = DieTempInt;
            result = m_i2cBus->write(I2C_W_ADRS, local_data, 1, true);
            if(result == 0)
            {
                //read two bytes
                result = m_i2cBus->read(I2C_R_ADRS, local_data, 2);
                if(result == 0)
                {
                    //stuff data
                    data = ( (local_data[0] << 8) | (local_data[1] << 4) );
                    data = (data >> 4);
                }
            }
        }
        else
        {
            //if result equals 0 because num_reads exceeded ATTEMPTS,
            //change result to -1.  Otherwise keep result's error code.
            result = (result == 0) ? -1 : result;
        }
    }
    return result;
}

    
//*****************************************************************************  
int32_t MAX30101::getDieTemperatureC(float &data)
{
    uint16_t raw_temp;
    
    int32_t result = getDieTemperature(raw_temp);
    if(result == 0)
    {
        if(raw_temp & 0x0800)
        {
            data = ((0xFFFFF000 | raw_temp)/16.0F);
        }
        else
        {
            data = (raw_temp/16.0F);
        }
    }
    
    return result;
}


//*****************************************************************************  
float MAX30101::celsius2fahrenheit(float c) 
{
    return ((1.8F * c) + 32.0F);
}
    

//*****************************************************************************     
int32_t MAX30101::setProxIntThreshold(const uint8_t data)
{
    return writeRegister(ProxIntThreshold, data);
}
    

//*****************************************************************************     
int32_t MAX30101::getProxIntThreshold(uint8_t &data)
{
    return readRegister(ProxIntThreshold, data);
}

bool MAX30101::read_spo2_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30101 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30101 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
    uint32_t un_temp;
    unsigned char uch_temp;
    *pun_red_led=0;
    *pun_ir_led=0;
    char ach_i2c_data[6];

    //read and clear status register
    readRegister(InterruptStatus1, uch_temp);
    readRegister(InterruptStatus2, uch_temp);

    ach_i2c_data[0] = FIFO_DataRegister;
    if(m_i2cBus->write(I2C_W_ADRS, ach_i2c_data, 1, true)!=0)
        return false;
    if(m_i2cBus->read(I2C_R_ADRS, ach_i2c_data, 6, false)!=0)
    {
        return false;
    }
    un_temp=(unsigned char) ach_i2c_data[0];
    un_temp<<=16;
    *pun_red_led+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[1];
    un_temp<<=8;
    *pun_red_led+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[2];
    *pun_red_led+=un_temp;

    un_temp=(unsigned char) ach_i2c_data[3];
    un_temp<<=16;
    *pun_ir_led+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[4];
    un_temp<<=8;
    *pun_ir_led+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[5];
    *pun_ir_led+=un_temp;
    *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
    *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]


    return true;
}

//*****************************************************************************     
int32_t MAX30101::readFIFO(LedChannels_e numLeds, uint8_t *data, uint16_t &readBytes)
{
    int32_t result = -1;

    readBytes = 0;

    //Get write pointer
    result = readRegister(FIFO_WritePointer, m_fifoWritePtr);
    if(result == 0)
    {
        //Get read pointer
        result = readRegister(FIFO_ReadPointer, m_fifoReadPtr);
        if(result == 0)
        {
            //Calculate num bytes to read
            if(m_fifoWritePtr > m_fifoReadPtr)
            {
                m_fifoNumBytes = ((m_fifoWritePtr - m_fifoReadPtr) * 
                                  (BYTES_PER_CH * numLeds));
            }
            else
            {
                m_fifoNumBytes = (((32 - m_fifoReadPtr) + m_fifoWritePtr) * 
                                   (BYTES_PER_CH * numLeds));
            }
            
            //temporary buffer for data
            char local_data[MAX30101::MAX_FIFO_BYTES];
            local_data[0] = FIFO_DataRegister;
            
            //Set fifo data ptr
            result = m_i2cBus->write(I2C_W_ADRS, local_data, 1, true);
            if(result == 0)
            {
                //read fifo
                /**
                for(int i = 0; i <= m_fifoNumBytes; i = i + 8)
                    result = m_i2cBus->read(I2C_R_ADRS, local_data+i, 8);
                **/
                result = m_i2cBus->read(I2C_R_ADRS, local_data,  m_fifoNumBytes);
                if(result == 0)
                {
                    //move data to user buffer
                    memcpy(data, local_data, m_fifoNumBytes);
                    readBytes = m_fifoNumBytes;
                }
            }
        }
    }
    
    return result;
}


//*****************************************************************************
int32_t MAX30101::writeRegister(Registers_e reg, uint8_t value)
{
    char local_data[2] = {reg, value};

    return m_i2cBus->write(I2C_W_ADRS, local_data, 2);
}


//*****************************************************************************
int32_t MAX30101::readRegister(Registers_e reg, uint8_t &value)
{
    int32_t result;
    
    char local_data[2];
    local_data[0] = reg;

    result = m_i2cBus->write(I2C_W_ADRS, local_data, 1);
    if(result == 0) 
    {
        result = m_i2cBus->read(I2C_R_ADRS, (local_data + 1), 1);
        if (result == 0) 
        {
            value = local_data[1];
        }
    }

    return result;
}

