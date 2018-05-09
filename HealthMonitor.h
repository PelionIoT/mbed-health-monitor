/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _HEALTHMONITOR_H_
#define _HEALTHMONITOR_H_

#include "mbed.h"
#include "MAX30101.h"
#include "MAX14720.h"
#include "MAX30001.h"
#include "algorithm.h"

#define HVOUT_VOLTAGE 4500 // set to 4500 mV
#define MAX30101_I2C_SLAVE_ADDR (0xAE)
#define MAX14720_I2C_SLAVE_ADDR (0x54)

class HealthMonitor
{
public:
    HealthMonitor();
    ~HealthMonitor();
    int init();
    bool read_spo2(uint32_t *spo2);
    uint8_t read_hr();

private:
    I2C i2c2;
    SPI spi; // used by MAX30001
    void read_ecg(uint8_t* data); 
    int init_pulse_ox();
    int init_ecg();
    int init_pmic();
    void spo2_range();
    MAX30101 max30101;
    InterruptIn max30101_Interrupt;
    MAX30001 max30001;
    InterruptIn max30001_InterruptB;
    InterruptIn max30001_Interrupt2B;
    PwmOut pwmout; /// PWM used as fclk for the MAX30001
    MAX14720 max14720;
    
    // Variables for spo2 algo
    uint32_t aun_ir_buffer[500]; //IR LED sensor data
    int32_t n_ir_buffer_length;    //data length
    uint32_t aun_red_buffer[500];    //Red LED sensor data
    int32_t n_sp02; //SPO2 value
    int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
    int32_t n_heart_rate;   //heart rate value
    int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
    int i;
    bool spo2_init;

};

#endif // HEALTHMONITOR_H
