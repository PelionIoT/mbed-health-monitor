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

#ifndef _MAX14720_H_
#define _MAX14720_H_

#include "mbed.h"

#define MAX14720_NO_ERROR 0
#define MAX14720_ERROR -1

#define MAX14720_BOOST_MIN_MV 2500
#define MAX14720_BOOST_MAX_MV 5000
#define MAX14720_BOOST_STEP_MV 100

/**
 * MAX14720 PMIC Driver
 *
 * @code
 * #include "mbed.h"
 * #include "MAX14720.h"
 *
 * // I2C Master 2
 * I2C i2c2(I2C2_SDA, I2C2_SCL);
 *
 * #define  I2C_ADDR_PMIC   (0x54)
 * MAX14720 max14720(&i2c2,I2C_ADDR_PMIC);
 *
 * DigitalOut led(LED1);
 * InterruptIn button(SW1);
 *
 * void turnOff()
 * {
 *     max14720.shutdown();
 * }
 *
 * int main()
 * {
 *     int result;
 *     button.fall(&turnOff);
 *     led = 0;
 *     max14720.boostEn = MAX14720::BOOST_ENABLED;
 *     result = max14720.init();
 *     if (result == MAX14720_ERROR) printf("Error initializing MAX14720");
 *     wait(1);
 *     while(1) {
 *         max14720.boostSetMode(MAX14720::BOOST_DISABLED);
 *         max14720.boostEn = MAX14720::BOOST_ENABLED;
 *         wait(0.5);
 *         max14720.boostSetVoltage(2500);
 *         wait(0.5);
 *         max14720.boostSetVoltage(5000);
 *         wait(0.5);
 *     }
 * }
 * @endcode
 */
class MAX14720 {

public:
    /**
     *@brief   Register Addresses
     *@details Enumerated MAX14720 register addresses
     */
    typedef enum {
        REG_CHIP_ID = 0x00,    ///< Chip ID
        REG_CHIP_REV = 0x01,   ///< Chip Revision
        REG_BOOST_CDIV = 0x03, ///< Boost Clock Divider
        REG_BOOST_ISET = 0x04, ///< Boost Peak Current
        REG_BOOST_VSET = 0x05, ///< Boost Voltage Setting
        REG_BOOST_CFG = 0x06,  ///< Boost Configuration
        REG_BUCK_VSET = 0x07,  ///< Buck Voltage Setting
        REG_BUCK_CFG = 0x08,   ///< Buck Configuration
        REG_BUCK_ISET = 0x09,  ///< Buck Peak Current and Settings
        REG_LDO_VSET = 0x0A,   ///< LDO Voltage Setting
        REG_LDO_CFG = 0x0B,    ///< LDO Configuration
        REG_SWITCH_CFG = 0x0C, ///< Switch Configuration
        REG_BAT_TIME = 0x0D,   ///< Battery Impedance Timing
        REG_BAT_CFG = 0x0E,    ///< Battery Impedance Configuration
        REG_BAT_BCV = 0x0F,    ///< Battery Cell Voltage
        REG_BAT_OCV = 0x10,    ///< Open Cell Voltage
        REG_BAT_LCV = 0x11,    ///< Loaded Cell Voltage
        REG_MON_CFG = 0x19,    ///< Monitor Multiplexer Configuration
        REG_BOOT_CFG = 0x1A,   ///< Boot Configuration
        REG_PIN_STAT = 0x1B,   ///< Pin Status
        REG_BBB_EXTRA = 0x1C,  ///< Buck/Buck-Boost Extra
        REG_HANDSHK = 0x1D,    ///< Power-On Handshake
        REG_UVLO_CFG = 0x1E,   ///< Under-Voltage Lock Out
        REG_PWR_OFF = 0x1F,    ///< Power Off Command
    } registers_t;

    /**
     *@brief   Boost Peak Current Settings
     *@details Enumerated peak current settings for boost regulator
     */
    typedef enum {
        BOOST_ISET_MIN,   ///< Minimum On-Time
        BOOST_ISET_50mA,  ///< 50mA Peak Current
        BOOST_ISET_100mA, ///< 100mA Peak Current
        BOOST_ISET_150mA, ///< 150mA Peak Current
        BOOST_ISET_200mA, ///< 200mA Peak Current
        BOOST_ISET_250mA, ///< 250mA Peak Current
        BOOST_ISET_300mA, ///< 300mA Peak Current
        BOOST_ISET_350mA, ///< 350mA Peak Current
    } boostISet_t;

    /**
     *@brief   Boost Enable Mode
     *@details Enumerated enable modes for boost regulator
     */
    typedef enum {
        BOOST_DISABLED, ///< Boost Disabled
        BOOST_ENABLED,  ///< Boost Enabled
        BOOST_EN_MPC,   ///< Boost Enabled by MPC pin
    } boostEn_t;

    /**
     *@brief   Buck Operating Modes
     *@details Enumerated operating modes for buck regulator
     */
    typedef enum {
        BUCK_BURST,    ///< Burst Mode Operation
        BUCK_FPWM,     ///< Forced PWM Operation
        BUCK_MPC_FPWM, ///< MPC activated Forced PWM
    } buckMd_t;

    /**
     *@brief   Buck Peak Current Settings
     *@details Enumerated peak current settings for buck regulator
     */
    typedef enum {
        BUCK_ISET_50mA,  ///< 50mA Peak Current
        BUCK_ISET_100mA, ///< 100mA Peak Current
        BUCK_ISET_150mA, ///< 150mA Peak Current
        BUCK_ISET_200mA, ///< 200mA Peak Current
        BUCK_ISET_250mA, ///< 250mA Peak Current
        BUCK_ISET_300mA, ///< 300mA Peak Current
        BUCK_ISET_350mA, ///< 350mA Peak Current
        BUCK_ISET_400mA, ///< 400mA Peak Current
    } buckISet_t;

    /**
     *@brief   Monitor Configurations
     *@details Enumerated configuration modes for monitor multiplexer
     */
    typedef enum {
        MON_PULLDOWN = 0x00, ///< Pulled down by 100k Ohm
        MON_HI_Z = 0x08,     ///< High Impedance
        MON_SWIN = 0x80,     ///< SWIN Selected
        MON_SWOUT = 0x81,    ///< SWOUT Selected
        MON_BIN = 0x82,      ///< BIN Selected
        MON_BOUT = 0x83,     ///< BOUT Selected
        MON_HVIN = 0x84,     ///< HVIN Selected
        MON_HVOUT = 0x85,    ///< HVOUT Selected
        MON_LIN = 0x86,      ///< LIN Selected
        MON_LOUT = 0x87,     ///< LOUT Selected
    } monCfg_t;

    /**
     *@brief   Under-Voltage Lock Out Input
     *@details Enumerated input selection options for UVLO
     */
    typedef enum {
        LIN_UVLO, ///< LIN used to determine UVLO condition
        BIN_UVLO, ///< BIN used to determine UVLO condition
    } uvloIn_t;

    /**
     * @brief MAX14720 constructor.
     *
     * @param sda mbed pin to use for SDA line of I2C interface.
     * @param scl mbed pin to use for SCL line of I2C interface.
     * @param slaveAddress Slave Address of the device.
     */
    MAX14720(PinName sda, PinName scl, int slaveAddress);

    /**
     * @brief MAX14720 constructor.
     *
     * @param i2c I2C object to use.
     * @param slaveAddress Slave Address of the device.
     */
    MAX14720(I2C *i2c, int slaveAddress);

    /**
     * @brief MAX14720 destructor.
     */
    ~MAX14720();

    /**
     * @brief   Initialize MAX14720
     * @details Applies settings to MAX14720.
     *  Settings are stored in public variables.
     *  The variables are pre-loaded with the most common configuation.
     *  Assign new values to the public variables before calling init.
     * @returns 0 if no errors, -1 if error.
     */
    int init();

    /**
     * @brief   Set the Boost Voltage
     * @details Sets the voltage for the boost regulator.
     *  The voltage is specified in millivoltst.
     *  The MAX14720 cannot update the voltage when enabled.
     *  This function checks the local boostEn variable and if the
     *  regualtor is enabled it will send the disable command before
     *  sending the new voltage and re-enable the boost regulator after
     *  the new voltage is written.
     * @param   mV voltage for boost regualtor in millivolts
     * @returns 0 if no errors, -1 if error.
     */
    int boostSetVoltage(int mV);

    /**
     * @brief   Set Boost Enable Mode
     * @details Sets the enable mode for the boost regulator
     * @param   mode The enable mode for the boost regulator
     * @returns 0 if no errors, -1 if error.
     */
    int boostSetMode(boostEn_t mode);

    /**
     * @brief   Configure Mon Pin
     * @details Configures the operating mode of the monitor multiplexer
     * @param   monCfg The configuration mode for the monitor pin
     * @returns 0 if no errors, -1 if error.
     */
    int monSet(monCfg_t monCfg);

    /**
     * @brief   Shutdown
     * @details Sends the command to turn off all supplies and put the part
     *  in battery saving shelf mode.
     * @returns 0 if no errors, -1 if error.
     */
    int shutdown();

    /**
     * @brief   Write Register
     * @details Writes the given value to the specified register
     * @param   reg The register to be written
     * @param   value The data to be written
     * @returns 0 if no errors, -1 if error.
     */
    int writeReg(registers_t reg, char value);

    /**
     * @brief   Read Register
     * @details Reads from the specified register
     * @param   reg The register to be read
     * @param   value Pointer for where to store the data
     * @returns 0 if no errors, -1 if error.
     */
    int readReg(registers_t reg, char *value);

    bool clkDivEn;         /// Boost Clock Divider Enable
    int clkDivSet;         /// Boost Clock Divider Setting
    boostISet_t boostISet; /// Boost Peak Current Setting
    int boostMillivolts;   /// Boost Voltage in millivolts
    boostEn_t boostEn;     /// Boost Enable Mode
    bool boostEMI,         /// Boost EMI Setting
        boostInd,          /// Boost Inductor Setting
        boostHysOff,       /// Boost Hysteresis Off
        boostPasDsc,       /// Boost Passive Discharge
        boostActDsc;       /// Boost Active Discharge
    buckMd_t buckMd;       /// Buck Operating Mode
    bool buckFst;          /// Buck Fast Start
    buckISet_t buckISet;   /// Buck Peak Current Setting
    bool buckCfg,   /// Buck Configuration (Set to 1 when using FPWM mode)
        buckInd,    /// Buck Inductor Setting
        buckHysOff, /// Buck Hysteresis Off
        buckMinOT,  /// Buck Minimum On Time
        buckInteg,  /// Buck Integrate
        buckPasDsc, /// Buck Passive Discharge
        buckActDsc, /// Buck Active Discharge
        buckFScl;   /// Buck Fet Scaling

private:
    /// @brief I2C pointer
    I2C *i2c;
    /// @brief Is this object the owner of the I2C object
    bool isOwner;
    /// @brief Device slave address
    int slaveAddress;
};

#endif /* _MAX14720_H_ */
