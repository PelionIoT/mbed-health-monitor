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

/// Initialization values for ECG_InitStart()
#define EN_ECG     0b1
#define OPENP      0b1
#define OPENN      0b1
#define POL        0b0
#define CALP_SEL   0b10
#define CALN_SEL   0b11
#define E_FIT      31
#define RATE       0b00
#define GAIN       0b00
#define DHPF       0b0
#define DLPF       0b01
/// Initialization values for CAL_InitStart() 
#define EN_VCAL  0b1
#define VMODE    0b1
#define VMAG     0b1
#define FCAL     0b011
#define THIGH    0x7FF
#define FIFTY    0b0
/// Initializaton values for Rbias_FMSTR_Init()
#define EN_RBIAS 0b01 
#define RBIASV   0b10
#define RBIASP   0b1
#define RBIASN   0b1
#define FMSTR    0b00

#include "HealthMonitor.h"
#include "MAX30101.h"

HealthMonitor::HealthMonitor():
    i2c2(I2C2_SDA, I2C2_SCL),
    spi(SPI0_MOSI, SPI0_MISO, SPI0_SCK, SPI0_SS),
    max14720(&i2c2, MAX14720_I2C_SLAVE_ADDR),
    max30001(&spi),
    max30001_InterruptB(P3_6),
    max30001_Interrupt2B(P4_5),
    max30101_Interrupt(P4_0),
    pwmout(P1_7),
    max30101(&i2c2)
{
    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    spo2_init = false;
}

HealthMonitor::~HealthMonitor(void){}

int HealthMonitor::init() 
{
    int result = init_pmic();
    // set NVIC priorities for GPIO to prevent priority inversion
    NVIC_SetPriority(GPIO_P0_IRQn, 5);
    NVIC_SetPriority(GPIO_P1_IRQn, 5);
    NVIC_SetPriority(GPIO_P2_IRQn, 5);
    NVIC_SetPriority(GPIO_P3_IRQn, 5);
    NVIC_SetPriority(GPIO_P4_IRQn, 5);
    NVIC_SetPriority(GPIO_P5_IRQn, 5);
    NVIC_SetPriority(GPIO_P6_IRQn, 5);
    // used by the MAX30001
    NVIC_SetPriority(SPI1_IRQn, 0);
    result += init_pulse_ox();
    result += init_ecg();
    return result;
}

int HealthMonitor::init_pulse_ox() 
{
    MAX30101::ModeConfiguration_u mode_config;
    mode_config.all = 0;
    mode_config.bits.reset = 1;
    int res = max30101.setModeConfiguration(mode_config);
    
    wait_ms(100);
    
    MAX30101::FIFO_Configuration_u fifo_config;
    fifo_config.bits.sample_average = 1;
    fifo_config.bits.fifo_a_full = 17; // fifo almost full = 17
    fifo_config.bits.fifo_roll_over_en = 0; // no roll over
    res = res + max30101.setFIFOConfiguration(fifo_config);

    MAX30101::SpO2Configuration_u spo2_config;
    spo2_config.bits.led_pw = 3; // 411 us
    spo2_config.bits.spo2_sr = 1; // 100 samples per second
    spo2_config.bits.spo2_adc_range = 1; // 4096 nA
    res = res + max30101.setSpO2Configuration(spo2_config);

    MAX30101::InterruptBitField_u interrupt_config;
    interrupt_config.bits.a_full = 1; // Almost full flag
    interrupt_config.bits.ppg_rdy = 1; // New FIFO Data Ready
    max30101.enableInterrupts(interrupt_config);
         

    // ~7 ma for both LED
    res += max30101.setLEDPulseAmplitude(MAX30101::LED1_PA, 0x24);
    res += max30101.setLEDPulseAmplitude(MAX30101::LED2_PA, 0x24);

    mode_config.all = 0;
    mode_config.bits.mode = 0x03;
    res += max30101.setModeConfiguration(mode_config);
    
    return res;
}

void HealthMonitor::spo2_range()
{
    //read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(max30101_Interrupt.read()==1);   //wait until the interrupt pin asserts
        
        max30101.read_spo2_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
            
    }
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    spo2_init = true;    
}

bool HealthMonitor::read_spo2(uint32_t *spo2)
{
    if(!spo2_init)
        spo2_range();
    
    i=0;

    //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
    for(i=100;i<500;i++)
    {
        aun_red_buffer[i-100]=aun_red_buffer[i];
        aun_ir_buffer[i-100]=aun_ir_buffer[i];
    }

    //take 100 sets of samples before calculating the heart rate.
    for(i=400;i<500;i++)
    {
        while(max30101_Interrupt.read()==1);
        max30101.read_spo2_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    //spo2_range();
    /**
    printf("red=");
    printf("%i", aun_red_buffer[i]);
    printf(", ir=");
    printf("%i", aun_ir_buffer[i]);
    printf(", HR=%i, ", n_heart_rate); 
    printf("HRvalid=%i, ", ch_hr_valid);
    printf("SpO2=%i, ", n_sp02);
    printf("SPO2Valid=%i\n\r", ch_spo2_valid);
    **/
    *spo2 = n_sp02;
    return (ch_spo2_valid == 1);
}

int HealthMonitor::init_pmic() {
    // initialize HVOUT on the MAX14720 PMIC
    int result = max14720.init();
    if (result == MAX14720_ERROR){
        return -1;
    }
    max14720.boostEn = MAX14720::BOOST_ENABLED;
    max14720.boostSetVoltage(HVOUT_VOLTAGE); 
    return result;
}

int HealthMonitor::init_ecg() {
    uint32_t all;
    max30001_InterruptB.disable_irq();
    max30001_Interrupt2B.disable_irq();
    max30001_InterruptB.mode(PullUp);
    max30001_InterruptB.fall(&MAX30001::Mid_IntB_Handler);
    max30001_Interrupt2B.mode(PullUp);
    max30001_Interrupt2B.fall(&MAX30001::Mid_Int2B_Handler);
    max30001_InterruptB.enable_irq();
    max30001_Interrupt2B.enable_irq();
    max30001.AllowInterrupts(1);
    // Configuring the FCLK for the ECG, set to 32.768KHZ
    max30001.FCLK_MaximOnly();    // mbed does not provide the resolution necessary, so for now we have a specific solution...
    max30001.sw_rst(); // Do a software reset of the MAX30001
    max30001.INT_assignment(MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,   MAX30001::MAX30001_NO_INT,  //  en_enint_loc,      en_eovf_loc,   en_fstint_loc,
                                     MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_NO_INT,  //  en_dcloffint_loc,  en_bint_loc,   en_bovf_loc,
                                     MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_NO_INT,  //  en_bover_loc,      en_bundr_loc,  en_bcgmon_loc,
                                     MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,   MAX30001::MAX30001_NO_INT,  //  en_pint_loc,       en_povf_loc,   en_pedge_loc,
                                     MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,  //  en_lonint_loc,     en_rrint_loc,  en_samp_loc,
                                     MAX30001::MAX30001_INT_ODNR, MAX30001::MAX30001_INT_ODNR);                            //  intb_Type,         int2b_Type)

    max30001.CAL_InitStart(EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);
    max30001.ECG_InitStart(EN_ECG, OPENP, OPENN, POL, CALP_SEL, CALN_SEL, E_FIT, RATE, GAIN, DHPF, DLPF);
    max30001.Rbias_FMSTR_Init(EN_RBIAS, RBIASV, RBIASP, RBIASN,FMSTR);
    max30001.synch();
    max30001.reg_read(MAX30001::STATUS, &all);
    return 0;  
} 

void HealthMonitor::read_ecg(uint8_t* data) {
    unsigned int i;
    uint8_t *bytePtr;
    MAX30001::max30001_bledata_t heartrateData;
    max30001.ReadHeartrateData(&heartrateData);
    bytePtr = reinterpret_cast<uint8_t *>(&heartrateData);
    for (i = 0; i < sizeof(MAX30001::max30001_bledata_t); i++)
      data[i] = bytePtr[i];
} 

uint8_t HealthMonitor::read_hr() {
    uint8_t data[4];    
    read_ecg(data);
    float t = 8.0f;
    float value = 0;
    float RtoR = (float) ((int) data[1] << 8) + (float) data[0];
    float fmStr = (float) ((int) data[3] << 8) + (float) data[2];
    if (fmStr == 0.0f) t = 7.813f;
    if (RtoR > 0.0f) {
        value = 60000.0f / (RtoR * t);
    }
    // Round up
    return (uint8_t)(value+0.5);
}



