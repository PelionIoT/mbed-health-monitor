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
#ifndef __MAX30101_H_
#define __MAX30101_H_

#include "mbed.h"

/**
 * @brief Library for the MAX30101\n
 * The MAX30101 is an integrated pulse oximetry and heart-rate monitor module.
 * It includes internal LEDs, photodetectors, optical elements, and low-noise
 * electronics with ambient light rejection. The MAX30101 provides a complete
 * system solution to ease the design-in process for mobile and wearable
 * devices.
 *
 * @code
 * #include "mbed.h"
 * #include "max32630fthr.h"
 * #include "MAX30101.h"
 *
 * MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);
 *
 * //Get I2C instance
 * I2C i2cBus(P3_4, P3_5);
 *
 * //Get temp sensor instance
 * MAX30101 op_sensor(i2cBus); //Constructor takes 7-bit slave adrs
 *
 * int main(void)
 * {
 *     //use sensor
 * }
 * @endcode
 */

class MAX30101 {
public:
    /// 7-bit slave address
    static const uint8_t I2C_ADRS = 0x57;
    /// 8-bit write address
    static const uint8_t I2C_W_ADRS = 0xAE;
    /// 8-bit read address
    static const uint8_t I2C_R_ADRS = 0xAF;
    /// Max # Bytes in FIFO
    static const uint16_t MAX_FIFO_BYTES = 288;
    ///# of bytes per LED channel
    static const uint8_t BYTES_PER_CH = 3;

    /// MAX30101 Register Map
    enum Registers_e {
        InterruptStatus1 = 0x00,
        InterruptStatus2 = 0x01,
        InterruptEnable1 = 0x02,
        InterruptEnable2 = 0x03,
        FIFO_WritePointer = 0x04,
        OverflowCounter = 0x05,
        FIFO_ReadPointer = 0x06,
        FIFO_DataRegister = 0x07,
        FIFO_Configuration = 0x08,
        ModeConfiguration = 0x09,
        SpO2Configuration = 0x0A,
        LED1_PA = 0x0C,
        LED2_PA = 0x0D,
        LED3_PA = 0x0E,
        ProxModeLED_PA = 0x10,
        ModeControlReg1 = 0x11,
        ModeControlReg2 = 0x12,
        DieTempInt = 0x1F,
        DieTempFrac = 0x20,
        DieTempConfig = 0x21,
        ProxIntThreshold = 0x30,
        RevID = 0xFE,
        PartID = 0xFF
    };

    /// MAX30101 Operational Modes
    enum OpModes_e { HeartRateMode = 2, SpO2Mode = 3, MultiLedMode = 7 };

    /// Number of LED channels used
    enum LedChannels_e {
        OneLedChannel = 1,
        TwoLedChannels = 2,
        ThreeLedChannels = 3
    };

    /// Number of samples averaged per FIFO sample, set in FIFO config
    enum NumSamplesAveraged_e {
        AveragedSamples_0 = 0,
        AveragedSamples_2 = 1,
        AveragedSamples_4 = 2,
        AveragedSamples_8 = 3,
        AveragedSamples_16 = 4,
        AveragedSamples_32 = 5
    };

    /// ADC Range, set in SpO2 config
    enum ADCRange_e {
        ADC_Range_0 = 0,
        ADC_Range_1 = 1,
        ADC_Range_2 = 2,
        ADC_Range_3 = 3
    };

    // LED PulseWidth, set in SpO2 config
    enum LEDPulseWidth { PW_0 = 0, PW_1 = 1, PW_2 = 2, PW_3 = 3 };

    /// Sample rate, set in SpO2 config
    enum SampleRate_e {
        SR_50_Hz = 0,
        SR_100_Hz = 1,
        SR_200_Hz = 2,
        SR_400_Hz = 3,
        SR_800_Hz = 4,
        SR_1000_Hz = 5,
        SR_1600_Hz = 6,
        SR_3200_Hz = 7
    };

    /// Interrupt Status/Enable BitField
    union InterruptBitField_u {
        uint8_t all;

        struct BitField_s {
            uint8_t pwr_rdy : 1;  ///< Bit0
            uint8_t die_temp : 1; ///< Bit1
            uint8_t reserved : 2; ///< Bit3:2
            uint8_t prox_int : 1; ///< Bit4
            uint8_t alc_ovf : 1;  ///< Bit5
            uint8_t ppg_rdy : 1;  ///< Bit6
            uint8_t a_full : 1;   ///< Bit7
        } bits;
    };

    /// FIFO Configuration BitField
    union FIFO_Configuration_u {
        uint8_t all;
        struct BitField_s {
            uint8_t fifo_a_full : 4;
            uint8_t fifo_roll_over_en : 1;
            uint8_t sample_average : 3;
        } bits;
    };

    /// Mode Configuration BitField
    union ModeConfiguration_u {
        uint8_t all;
        struct BitField_s {
            uint8_t mode : 3;
            uint8_t reserved : 3;
            uint8_t reset : 1;
            uint8_t shdn : 1;
        } bits;
    };

    /// SpO2 Configuration BitField
    union SpO2Configuration_u {
        uint8_t all;
        struct BitField_s {
            uint8_t led_pw : 2;
            uint8_t spo2_sr : 3;
            uint8_t spo2_adc_range : 2;
            uint8_t reserved : 1;
        } bits;
    };

    /// Multi-LED Mode Control Register BitField
    union ModeControlReg_u {
        uint8_t all;
        struct BitField_s {
            uint8_t lo_slot : 3;
            uint8_t reserved1 : 1;
            uint8_t hi_slot : 3;
            uint8_t reserved2 : 1;
        } bits;
    };

    /**
     * @brief  Constructor using reference to I2C object
     * @param i2c - Pointer to I2C object
     */
    MAX30101(I2C *i2c);

    /** @brief Destructor */
    ~MAX30101();

    /**
     * @brief Writes appropriate bits to Interrupt Enable 1 and 2.
     *
     * @param data - Interrupts to enable
     * @return 0 on success, non 0 otherwise
     */
    int32_t enableInterrupts(const InterruptBitField_u data);

    /**
     * @brief Reads interrupt status flags from Interrupt Status 1 and 2.
     *
     * @param[out] data - Contains interrupts status flags on success.
     * @return 0 on success, non 0 otherwise
     */
    int32_t getInterruptStatus(InterruptBitField_u &data);

    /**
     * @brief Writes FIFO configuration register with given data
     *
     * @param config - FIFO Configuration
     * @return 0 on success, non 0 otherwise
     */
    int32_t setFIFOConfiguration(const FIFO_Configuration_u config);

    /**
     * @brief Reads FIFO configuration register
     *
     * @param[out] config - FIFO Configuration on success
     * @return 0 on success, non 0 otherwise
     */
    int32_t getFIFOConfiguration(FIFO_Configuration_u &config);

    /**
     * @brief Writes Mode configuration register with given data
     *
     * @param config - Mode Configuration
     * @return 0 on success, non 0 otherwise
     */
    int32_t setModeConfiguration(const ModeConfiguration_u config);

    /**
     * @brief Reads Mode configuration register
     *
     * @param[out] config - Mode Configuration on success
     * @return 0 on success, non 0 otherwise
     */
    int32_t getModeConfiguration(ModeConfiguration_u &config);

    /**
     * @brief Writes SpO2 configuration register with given data
     *
     * @param config - SpO2 Configuration
     * @return 0 on success, non 0 otherwise
     */
    int32_t setSpO2Configuration(const SpO2Configuration_u config);

    /**
     * @brief Reads SpO2 configuration register
     *
     * @param[out] config - SpO2 Configuration on success
     * @return 0 on success, non 0 otherwise
     */
    int32_t getSpO2Configuration(SpO2Configuration_u &config);

    /**
     * @brief Writes LEDx/Prox Pulse Amplitude register with given data
     *
     * @param reg - LEDx/Prox Pulse Amplitude register to write
     * @param amp - LED pulse amplitude
     * @return 0 on success, non 0 otherwise
     */
    int32_t setLEDPulseAmplitude(Registers_e reg, const uint8_t amp);

    /**
     * @brief Reads LEDx/Prox Pulse Amplitude register
     *
     * @param reg - LEDx/Prox Pulse Amplitude register to read
     * @param[out] amp - LED pulse amplitude on success
     * @return 0 on success, non 0 otherwise
     */
    int32_t getLEDPulseAmplitude(Registers_e reg, uint8_t &amp);

    /**
     * @brief Writes Multi-LED Mode Control Register
     *
     * @param reg - Multi-LED Mode Control register 1 or 2
     * @param data - Data to write to register
     * @return 0 on success, non 0 otherwise
     */
    int32_t setMultiLEDModeControl(Registers_e reg,
                                   const ModeControlReg_u data);

    /**
     * @brief Reads Multi-LED Mode Control Register
     *
     * @param reg - Multi-LED Mode Control register 1 or 2
     * @param[out] data - Data read from register on success
     * @return 0 on success, non 0 otherwise
     */
    int32_t getMultiLEDModeControl(Registers_e reg, ModeControlReg_u &data);

    /**
     * @brief Gets raw die temperature, interrupt must be enabled
     *
     * @param[out] data - Raw die temperature on success
     *
     * @return 0 on success, non 0 otherwise
     */
    int32_t getDieTemperature(uint16_t &data);

    /**
     * @brief Gets die temperature in celsius, interrupt must be enabled
     *
     * @param[out] data - Die temperature in celsius on success
     *
     * @return 0 on success, non 0 otherwise
     */
    int32_t getDieTemperatureC(float &data);

    /**
     * @brief Converts celsius to Fahrenheit
     *
     * @param c - Temperature in celsius
     *
     * @return Temperature in Fahrenheit
     */
    float celsius2fahrenheit(float c);

    /**
     * @brief Writes Proximity Interrupt Threshold Register
     *
     * @param data - Data to write to register
     * @return 0 on success, non 0 otherwise
     */
    int32_t setProxIntThreshold(const uint8_t data);

    /**
     * @brief Reads Proximity Interrupt Threshold Register
     *
     * @param data - Data read on success
     * @return 0 on success, non 0 otherwise
     */
    int32_t getProxIntThreshold(uint8_t &data);

    /**
     * @brief Attempts to read FIFO
     *
     * @param numLeds - Number of LED channels used; 0 < numLeds < 4
     * @param data - pointer to buffer for holding read data
     * @param[out] readBytes - number of bytes read from fifo
     *
     * @return 0 on success, non 0 otherwise
     */
    int32_t readFIFO(LedChannels_e numLeds, uint8_t *data, uint16_t &readBytes);

    bool read_spo2_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);

protected:
    /**
     * @brief Write register of device
     * @param reg - Register address
     * @param value - Value to write
     * @return 0 on success, non-zero on failure
     */
    int32_t writeRegister(Registers_e reg, uint8_t value);

    /**
     * @brief  Read register of device
     * @param reg - Register address
     * @param[out] value - Read data on success
     * @return 0 on success, non-zero on failure
     */
    int32_t readRegister(Registers_e reg, uint8_t &value);

private:
    I2C *m_i2cBus;
    uint8_t m_fifoReadPtr, m_fifoWritePtr, m_fifoNumBytes;
};

#endif /* __MAX30101_H_ */
