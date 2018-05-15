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

#include "mbed.h"
#include "MAX30001.h"
#include "pwrseq_regs.h"

MAX30001 *MAX30001::instance = NULL;

//******************************************************************************
MAX30001::MAX30001(PinName mosi, PinName miso, PinName sclk, PinName cs) {
  spi = new SPI(mosi, miso, sclk, cs);
  spi->frequency(3000000);
  spi_owner = true;
  functionpointer.attach(&spiHandler);
  onDataAvailableCallback = NULL;
  xferFlag = 0;
  instance = this;
}

//******************************************************************************
MAX30001::MAX30001(SPI *_spi) {
  spi = _spi;
  spi->frequency(3000000);
  spi_owner = false;
  functionpointer.attach(&spiHandler);
  onDataAvailableCallback = NULL;
  xferFlag = 0;
  instance = this;
}

//******************************************************************************
MAX30001::~MAX30001(void) {
  if (spi_owner) {
    delete spi;
  }
}

//******************************************************************************
void MAX30001::FCLK_MaximOnly(void){
 
  // Use RTC crystal clock for MAX30001 FCLK
/*
  mxc_pwrseq_reg0_t pwr_reg0;
  mxc_pwrseq_reg4_t pwr_reg4;
 
  // Set the port pin connected to the MAX30001 FCLK pin as an output
  GPIO_SetOutMode(MAX30001_INT_PORT_FCLK, MAX30001_INT_PIN_FCLK, MXC_E_GPIO_OUT_MODE_NORMAL);
 
  // Enable Real Time Clock in Run and Sleep modes
  pwr_reg0 = MXC_PWRSEQ->reg0_f;
  pwr_reg0.pwr_rtcen_run = 1;
  pwr_reg0.pwr_rtcen_slp = 1;
  MXC_PWRSEQ->reg0_f = pwr_reg0;
 
  // Enable the RTC clock output path on P1.7
  pwr_reg4 = MXC_PWRSEQ->reg4_f;
  pwr_reg4.pwr_pseq_32k_en = 1;
  MXC_PWRSEQ->reg4_f = pwr_reg4;
*/
 
  #define PORT_FCLK 1
  #define PIN_FCLK  7
 
  // Set the Port pin connected to the MAX30001 FCLK pin as an output
  uint32_t temp = MXC_GPIO->out_mode[PORT_FCLK];  // Port 1
  
  //    temp = (temp & ~(0xF << (pin * 4))) | (val << (pin * 4));
                               /* pin 7 */      /* NORMAL MODE */
  temp = (temp & ~(0xF << (PIN_FCLK * 4))) | (MXC_V_GPIO_OUT_MODE_NORMAL << (PIN_FCLK * 4));
  
 
//    temp = (temp & ~(0xF << (7 * 4))) | (5 << (7 * 4));
  
  MXC_GPIO->out_mode[PORT_FCLK] = temp;
  
  
  // Enable Real Time Clock in Run and Sleep Modes
  MXC_PWRSEQ->reg0 = MXC_PWRSEQ->reg0 | MXC_F_PWRSEQ_REG0_PWR_RTCEN_RUN |  MXC_F_PWRSEQ_REG0_PWR_RTCEN_SLP;
  
  // Enable the RTC clock output path on P1.7
  MXC_PWRSEQ->reg4 = MXC_PWRSEQ->reg4 | MXC_F_PWRSEQ_REG4_PWR_PSEQ_32K_EN;
    
}
 

//******************************************************************************
int MAX30001::Rbias_FMSTR_Init(uint8_t En_rbias, uint8_t Rbiasv,
                               uint8_t Rbiasp, uint8_t Rbiasn,
                               uint8_t Fmstr) {
                                        	
  max30001_cnfg_gen_t cnfg_gen;                                        	
                                        	
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_rbias = En_rbias;
  cnfg_gen.bit.rbiasv   = Rbiasv;
  cnfg_gen.bit.rbiasp   = Rbiasp;
  cnfg_gen.bit.rbiasn   = Rbiasn;
  cnfg_gen.bit.fmstr    = Fmstr;

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }
  return 0;
}

//******************************************************************************
int MAX30001::CAL_InitStart(uint8_t En_Vcal, uint8_t Vmode,
                            uint8_t Vmag, uint8_t Fcal, uint16_t Thigh,
                            uint8_t Fifty) {
             
  max30001_cnfg_cal_t cnfg_cal;
                  
  ///< CNFG_CAL
  if (reg_read(CNFG_CAL, &cnfg_cal.all) == -1) {
    return -1;
  }

  cnfg_cal.bit.vmode = Vmode;
  cnfg_cal.bit.vmag  = Vmag;
  cnfg_cal.bit.fcal  = Fcal;
  cnfg_cal.bit.thigh = Thigh;
  cnfg_cal.bit.fifty = Fifty;

  if (reg_write(CNFG_CAL, cnfg_cal.all) == -1) {
    return -1;
  }

  /// @brief RTOS uses a 32768HZ clock.  32768ticks represents 1secs.  1sec/10 =
  ///        100msecs.
  wait(1.0 / 10.0);

  if (reg_read(CNFG_CAL, &cnfg_cal.all) == -1) {
    return -1;
  }

  cnfg_cal.bit.en_vcal = En_Vcal;

  if (reg_write(CNFG_CAL, cnfg_cal.all) == -1) {
    return -1;
  }

  /// @brief RTOS uses a 32768HZ clock.  32768ticks represents 1secs.  1sec/10 =
  ///        100msecs.
  wait(1.0 / 10.0);

  return 0;
}

//******************************************************************************
int MAX30001::CAL_Stop(void) {

  max30001_cnfg_cal_t cnfg_cal;

  if (reg_read(CNFG_CAL, &cnfg_cal.all) == -1) {
    return -1;
  }

  cnfg_cal.bit.en_vcal = 0; // Disable VCAL, all other settings are left unaffected

  if (reg_write(CNFG_CAL, cnfg_cal.all) == -1) {
    return -1;
  }

  return 0;
}
//******************************************************************************
int MAX30001::INT_assignment(max30001_intrpt_Location_t en_enint_loc,     max30001_intrpt_Location_t en_eovf_loc,  max30001_intrpt_Location_t en_fstint_loc,
		                     max30001_intrpt_Location_t en_dcloffint_loc, max30001_intrpt_Location_t en_bint_loc,  max30001_intrpt_Location_t en_bovf_loc,
		                     max30001_intrpt_Location_t en_bover_loc,     max30001_intrpt_Location_t en_bundr_loc, max30001_intrpt_Location_t en_bcgmon_loc,
		                     max30001_intrpt_Location_t en_pint_loc,      max30001_intrpt_Location_t en_povf_loc,  max30001_intrpt_Location_t en_pedge_loc,
		                     max30001_intrpt_Location_t en_lonint_loc,    max30001_intrpt_Location_t en_rrint_loc, max30001_intrpt_Location_t en_samp_loc,
		                     max30001_intrpt_type_t  intb_Type,           max30001_intrpt_type_t int2b_Type)


{
  
  max30001_en_int_t en_int;
  max30001_en_int2_t en_int2;
  
  ///< INT1

  if (reg_read(EN_INT, &en_int.all) == -1) {
    return -1;
  }

  // max30001_en_int2.bit.en_pint       = 0b1;  // Keep this off...

  en_int.bit.en_eint   = 0b1 & en_enint_loc;
  en_int.bit.en_eovf   = 0b1 & en_eovf_loc;
  en_int.bit.en_fstint = 0b1 & en_fstint_loc;

  en_int.bit.en_dcloffint = 0b1 & en_dcloffint_loc;
  en_int.bit.en_bint      = 0b1 & en_bint_loc;
  en_int.bit.en_bovf      = 0b1 & en_bovf_loc;

  en_int.bit.en_bover  = 0b1 & en_bover_loc;
  en_int.bit.en_bundr  = 0b1 & en_bundr_loc;
  en_int.bit.en_bcgmon = 0b1 & en_bcgmon_loc;

  en_int.bit.en_pint   = 0b1 & en_pint_loc;
  en_int.bit.en_povf   = 0b1 & en_povf_loc;
  en_int.bit.en_pedge  = 0b1 & en_pedge_loc;

  en_int.bit.en_lonint = 0b1 & en_lonint_loc;
  en_int.bit.en_rrint  = 0b1 & en_rrint_loc;
  en_int.bit.en_samp   = 0b1 & en_samp_loc;

  en_int.bit.intb_type = int2b_Type;

  if (reg_write(EN_INT, en_int.all) == -1) {
    return -1;
  }

  ///< INT2

  if (reg_read(EN_INT2, &en_int2.all) == -1) {
    return -1;
  }

  en_int2.bit.en_eint   = 0b1 & (en_enint_loc >> 1);
  en_int2.bit.en_eovf   = 0b1 & (en_eovf_loc >> 1);
  en_int2.bit.en_fstint = 0b1 & (en_fstint_loc >> 1);

  en_int2.bit.en_dcloffint = 0b1 & (en_dcloffint_loc >> 1);
  en_int2.bit.en_bint      = 0b1 & (en_bint_loc >> 1);
  en_int2.bit.en_bovf      = 0b1 & (en_bovf_loc >> 1);

  en_int2.bit.en_bover  = 0b1 & (en_bover_loc >> 1);
  en_int2.bit.en_bundr  = 0b1 & (en_bundr_loc >> 1);
  en_int2.bit.en_bcgmon = 0b1 & (en_bcgmon_loc >> 1);

  en_int2.bit.en_pint  = 0b1 & (en_pint_loc >> 1);
  en_int2.bit.en_povf  = 0b1 & (en_povf_loc >> 1);
  en_int2.bit.en_pedge = 0b1 & (en_pedge_loc >> 1);

  en_int2.bit.en_lonint = 0b1 & (en_lonint_loc >> 1);
  en_int2.bit.en_rrint  = 0b1 & (en_rrint_loc >> 1);
  en_int2.bit.en_samp   = 0b1 & (en_samp_loc >> 1);

  en_int2.bit.intb_type = intb_Type;

  if (reg_write(EN_INT2, en_int2.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::ECG_InitStart(uint8_t En_ecg, uint8_t Openp,
                            uint8_t Openn, uint8_t Pol,
                            uint8_t Calp_sel, uint8_t Caln_sel,
                            uint8_t E_fit, uint8_t Rate, uint8_t Gain,
                            uint8_t Dhpf, uint8_t Dlpf) {

  max30001_cnfg_emux_t cnfg_emux;
  max30001_cnfg_gen_t  cnfg_gen;
  max30001_status_t    status;
  max30001_mngr_int_t  mngr_int;
  max30001_cnfg_ecg_t  cnfg_ecg;
  
  ///< CNFG_EMUX

  if (reg_read(CNFG_EMUX, &cnfg_emux.all) == -1) {
    return -1;
  }

  cnfg_emux.bit.openp    = Openp;
  cnfg_emux.bit.openn    = Openn;
  cnfg_emux.bit.pol      = Pol;
  cnfg_emux.bit.calp_sel = Calp_sel;
  cnfg_emux.bit.caln_sel = Caln_sel;

  if (reg_write(CNFG_EMUX, cnfg_emux.all) == -1) {
    return -1;
  }

  /**** ENABLE CHANNELS ****/
  ///< CNFG_GEN

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_ecg = En_ecg; // 0b1

  ///< fmstr is default

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  ///< Wait for PLL Lock & References to settle down 

  max30001_timeout = 0;

  do {
    if (reg_read(STATUS, &status.all) == -1) {// Wait and spin for PLL to lock...
    
      return -1;
    }
  } while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

  ///< MNGR_INT

  if (reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.e_fit = E_fit; // 31

  if (reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  ///< CNFG_ECG

  if (reg_read(CNFG_ECG, &cnfg_ecg.all) == -1) {
    return -1;
  }

  cnfg_ecg.bit.rate = Rate; 
  cnfg_ecg.bit.gain = Gain;
  cnfg_ecg.bit.dhpf = Dhpf;
  cnfg_ecg.bit.dlpf = Dlpf;

  if (reg_write(CNFG_ECG, cnfg_ecg.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::ECGFast_Init(uint8_t Clr_Fast, uint8_t Fast, uint8_t Fast_Th) {
  
  max30001_mngr_int_t mngr_int;
  max30001_mngr_dyn_t mngr_dyn;
  
  if (reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.clr_fast = Clr_Fast;

  if (reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  if (reg_read(MNGR_DYN, &mngr_dyn.all) == -1) {
    return -1;
  }

  mngr_dyn.bit.fast = Fast;
  mngr_dyn.bit.fast_th = Fast_Th;

  if (reg_write(MNGR_INT, mngr_dyn.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::Stop_ECG(void) {

  max30001_cnfg_gen_t cnfg_gen;

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_ecg = 0; ///< Stop ECG

  ///< fmstr is default

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::PACE_InitStart(uint8_t En_pace, uint8_t Clr_pedge,
                             uint8_t Pol, uint8_t Gn_diff_off,
                             uint8_t Gain, uint8_t Aout_lbw,
                             uint8_t Aout, uint8_t Dacp,
                             uint8_t Dacn) {

  /**** SET MASTER FREQUENCY, ENABLE CHANNELS ****/

   max30001_cnfg_gen_t  cnfg_gen;
   max30001_status_t    status;
   max30001_mngr_int_t  mngr_int;
   max30001_cnfg_pace_t cnfg_pace;

  ///< CNFG_GEN

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_pace = En_pace; // 0b1;

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  /**** Wait for PLL Lock & References to settle down ****/
  max30001_timeout = 0;

  do {
    if (reg_read(STATUS, &status.all) ==
        -1) // Wait and spin for PLL to lock...
    {
      return -1;
    }

  } while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

  ///< MNGR_INT

  if (reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.clr_pedge = Clr_pedge; // 0b0;

  if (reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  ///< CNFG_PACE

  reg_read(CNFG_PACE, &cnfg_pace.all);

  cnfg_pace.bit.pol         = Pol;         
  cnfg_pace.bit.gn_diff_off = Gn_diff_off;
  cnfg_pace.bit.gain        = Gain;
  cnfg_pace.bit.aout_lbw    = Aout_lbw;
  cnfg_pace.bit.aout        = Aout;
  cnfg_pace.bit.dacp        = Dacp;
  cnfg_pace.bit.dacn        = Dacn;

  reg_write(CNFG_PACE, cnfg_pace.all);

  return 0;
}

//******************************************************************************
int MAX30001::Stop_PACE(void) {

  max30001_cnfg_gen_t cnfg_gen;

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_pace = 0; ///< Stop PACE

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::BIOZ_InitStart(
    uint8_t En_bioz, uint8_t Openp, uint8_t Openn, uint8_t Calp_sel,
    uint8_t Caln_sel, uint8_t CG_mode, uint8_t B_fit, uint8_t Rate,
    uint8_t Ahpf, uint8_t Ext_rbias, uint8_t Gain, uint8_t Dhpf, uint8_t Dlpf,
    uint8_t Fcgen, uint8_t Cgmon, uint8_t Cgmag, uint8_t Phoff) {

  max30001_cnfg_bmux_t cnfg_bmux;
  max30001_cnfg_gen_t  cnfg_gen;
  max30001_status_t    status;
  max30001_mngr_int_t  mngr_int;
  max30001_cnfg_bioz_t cnfg_bioz;
  
  
  // CNFG_BMUX

  if (reg_read(CNFG_BMUX, &cnfg_bmux.all) == -1) {
    return -1;
  }

  cnfg_bmux.bit.openp    = Openp;
  cnfg_bmux.bit.openn    = Openn;
  cnfg_bmux.bit.calp_sel = Calp_sel;
  cnfg_bmux.bit.caln_sel = Caln_sel;
  cnfg_bmux.bit.cg_mode  = CG_mode;

  if (reg_write(CNFG_BMUX, cnfg_bmux.all) == -1) {
    return -1;
  }

  /**** SET MASTER FREQUENCY, ENABLE CHANNELS ****/

  ///< CNFG_GEN

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_bioz = En_bioz;

  ///< fmstr is default

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  /**** Wait for PLL Lock & References to settle down ****/

  max30001_timeout = 0;

  do {
    if (reg_read(STATUS, &status.all) == -1) { // Wait and spin for PLL to lock...
      return -1;
    }

  } while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

  /**** Start of CNFG_BIOZ ****/

  ///< MNGR_INT

  if (reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.b_fit = B_fit; //;

  if (reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  ///< CNFG_BIOZ

  if (reg_read(CNFG_BIOZ, &cnfg_bioz.all) == -1) {
    return -1;
  }

  cnfg_bioz.bit.rate      = Rate;
  cnfg_bioz.bit.ahpf      = Ahpf;
  cnfg_bioz.bit.ext_rbias = Ext_rbias;
  cnfg_bioz.bit.gain      = Gain;
  cnfg_bioz.bit.dhpf      = Dhpf;
  cnfg_bioz.bit.dlpf      = Dlpf;
  cnfg_bioz.bit.fcgen     = Fcgen;
  cnfg_bioz.bit.cgmon     = Cgmon;
  cnfg_bioz.bit.cgmag     = Cgmag;
  cnfg_bioz.bit.phoff     = Phoff;

  if (reg_write(CNFG_BIOZ, cnfg_bioz.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::Stop_BIOZ(void) {

  max30001_cnfg_gen_t cnfg_gen;
  
  ///< CNFG_GEN

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_bioz = 0; // Stop BIOZ

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::BIOZ_InitBist(uint8_t En_bist, uint8_t Rnom,
                            uint8_t Rmod, uint8_t Fbist) {

  max30001_cnfg_bmux_t cnfg_bmux;

  ///< CNFG_BMUX

  if (reg_read(CNFG_BMUX, &cnfg_bmux.all) == -1) {
    return -1;
  }

  cnfg_bmux.bit.en_bist = En_bist;
  cnfg_bmux.bit.rnom = Rnom;
  cnfg_bmux.bit.rmod = Rmod;
  cnfg_bmux.bit.fbist = Fbist;

  if (reg_write(CNFG_BMUX, cnfg_bmux.all) == -1) {
    return -1;
  }

  return 0;
}
//******************************************************************************
int MAX30001::RtoR_InitStart(uint8_t En_rtor, uint8_t Wndw,
                             uint8_t Gain, uint8_t Pavg, uint8_t Ptsf,
                             uint8_t Hoff, uint8_t Ravg, uint8_t Rhsf,
                             uint8_t Clr_rrint) {

  max30001_mngr_int_t mngr_int;
  max30001_cnfg_rtor1_t cnfg_rtor1;
  max30001_cnfg_rtor2_t cnfg_rtor2;

  ///< MNGR_INT
  if (reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }

  mngr_int.bit.clr_rrint = Clr_rrint; 
  ///< 0b01 & 0b00 are for interrupt mode...
  ///< 0b10 is for monitoring mode... it just overwrites the data...

  if (reg_write(MNGR_INT, mngr_int.all) == -1) {
    return -1;
  }

  ///< RTOR1
  if (reg_read(CNFG_RTOR1, &cnfg_rtor1.all) == -1) {
    return -1;
  }

  cnfg_rtor1.bit.wndw = Wndw;
  cnfg_rtor1.bit.gain = Gain;
  cnfg_rtor1.bit.en_rtor = En_rtor;
  cnfg_rtor1.bit.pavg = Pavg;
  cnfg_rtor1.bit.ptsf = Ptsf;

  if (reg_write(CNFG_RTOR1, cnfg_rtor1.all) == -1) {
    return -1;
  }
  
  ///< RTOR2
  if (reg_read(CNFG_RTOR2, &cnfg_rtor2.all) == -1) {
    return -1;
  }
  cnfg_rtor2.bit.hoff = Hoff;
  cnfg_rtor2.bit.ravg = Ravg;
  cnfg_rtor2.bit.rhsf = Rhsf;

  if (reg_write(CNFG_RTOR2, cnfg_rtor2.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::Stop_RtoR(void) {

  max30001_cnfg_rtor1_t cnfg_rtor1;

  if (reg_read(CNFG_RTOR1, &cnfg_rtor1.all) == -1) {
    return -1;
  }

  cnfg_rtor1.bit.en_rtor = 0; ///< Stop RtoR

  if (reg_write(CNFG_RTOR1, cnfg_rtor1.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::PLL_lock(void) {
  ///< Spin to see PLLint become zero to indicate a lock.

  max30001_status_t status;

  max30001_timeout = 0;

  do {
    if (reg_read(STATUS, &status.all) == -1) { ///< Wait and spin for PLL to lock...

      return -1;
    }

  } while (status.bit.pllint == 1 && max30001_timeout++ <= 1000);

  return 0;
}

//******************************************************************************
int MAX30001::sw_rst(void) {
  ///< SW reset for the MAX30001 chip

  if (reg_write(SW_RST, 0x000000) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::synch(void) { ///< For synchronization
  if (reg_write(SYNCH, 0x000000) == -1) {
    return -1;
  }
  return 0;
}

//******************************************************************************
int MAX30001::fifo_rst(void) { ///< Resets the FIFO
  if (reg_write(FIFO_RST, 0x000000) == -1) {
    return -1;
  }
  return 0;
}

//******************************************************************************
int MAX30001::reg_write(MAX30001_REG_map_t addr, uint32_t data) {

  uint8_t result[4];
  uint8_t data_array[4];
  int32_t success = 0;

  data_array[0] = (addr << 1) & 0xff;

  data_array[3] = data & 0xff;
  data_array[2] = (data >> 8) & 0xff;
  data_array[1] = (data >> 16) & 0xff;

  success = SPI_Transmit(&data_array[0], 4, &result[0], 4);

  if (success != 0) {
    return -1;
  } else {
    return 0;
  }
}

//******************************************************************************
int MAX30001::reg_read(MAX30001_REG_map_t addr,
                       uint32_t *return_data) {
  uint8_t result[4];
  uint8_t data_array[1];
  int32_t success = 0;

  data_array[0] = ((addr << 1) & 0xff) | 1; // For Read, Or with 1
  success = SPI_Transmit(&data_array[0], 1, &result[0], 4);
  *return_data = /*result[0] + */ (uint32_t)(result[1] << 16) +
                 (result[2] << 8) + result[3];
  if (success != 0) {
    return -1;
  } else {
    return 0;
  }
}

//******************************************************************************
int MAX30001::Enable_DcLeadOFF_Init(int8_t En_dcloff, int8_t Ipol,
                                    int8_t Imag, int8_t Vth) {
  ///<  the leads are not touching the body

  max30001_cnfg_gen_t cnfg_gen;

  ///< CNFG_EMUX, Set ECGP and ECGN for external hook up...

  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_dcloff = En_dcloff;
  cnfg_gen.bit.ipol = Ipol;
  cnfg_gen.bit.imag = Imag;
  cnfg_gen.bit.vth = Vth;

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::Disable_DcLeadOFF(void) {
  
  max30001_cnfg_gen_t cnfg_gen;
  
  ///< CNFG_GEN
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_dcloff = 0; // Turned off the dc lead off.

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::BIOZ_Enable_ACLeadOFF_Init(uint8_t En_bloff, uint8_t Bloff_hi_it,
                                         uint8_t Bloff_lo_it) {

  max30001_cnfg_gen_t cnfg_gen;
  max30001_mngr_dyn_t mngr_dyn;

  ///< CNFG_GEN
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_bloff = En_bloff;

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  ///< MNGR_DYN
  if (reg_read(MNGR_DYN, &mngr_dyn.all) == -1) {
    return -1;
  }

  mngr_dyn.bit.bloff_hi_it = Bloff_hi_it;
  mngr_dyn.bit.bloff_lo_it = Bloff_lo_it;

  if (reg_write(MNGR_DYN, mngr_dyn.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::BIOZ_Disable_ACleadOFF(void) {

  max30001_cnfg_gen_t cnfg_gen;
  
  ///< CNFG_GEN
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_bloff = 0b0; // Turns of the BIOZ AC Lead OFF feature

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::BIOZ_Enable_BCGMON(void) {
  
  max30001_cnfg_bioz_t cnfg_bioz;
  
  ///< CNFG_BIOZ
  if (reg_read(CNFG_BIOZ, &cnfg_bioz.all) == -1) {
    return -1;
  }

  cnfg_bioz.bit.cgmon = 1;

  if (reg_write(CNFG_BIOZ, cnfg_bioz.all) == -1) {
    return -1;
  }

  return 0;
}


//******************************************************************************
int MAX30001::Enable_LeadON(int8_t Channel) // Channel: ECG = 0b01, BIOZ = 0b10, Disable = 0b00
{
  
  max30001_cnfg_gen_t cnfg_gen;

  ///< CNFG_GEN
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_ecg  = 0b0;
  cnfg_gen.bit.en_bioz = 0b0;
  cnfg_gen.bit.en_pace = 0b0;

  cnfg_gen.bit.en_ulp_lon = Channel; ///< BIOZ ULP lead on detection...

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
int MAX30001::Disable_LeadON(void) {

  max30001_cnfg_gen_t cnfg_gen;
  ///< CNFG_GEN
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }

  cnfg_gen.bit.en_ulp_lon = 0b0;

  if (reg_write(CNFG_GEN, cnfg_gen.all) == -1) {
    return -1;
  }

  return 0;
}

//******************************************************************************
#define LEADOFF_SERVICE_TIME 0x2000 ///< 0x1000 = 1 second
#define LEADOFF_NUMSTATES 2
uint32_t leadoffState = 0;
uint32_t max30001_LeadOffoldTime = 0;
void MAX30001::ServiceLeadoff(uint32_t currentTime) {

  uint32_t delta_Time;

  delta_Time = currentTime - max30001_LeadOffoldTime;

  if (delta_Time > LEADOFF_SERVICE_TIME) {
    switch (leadoffState) {
    case 0: ///< switch to ECG DC Lead OFF
      Enable_DcLeadOFF_Init(0b01, 0b0, 0b001, 0b00);
      break;

    case 1: ///< switch to BIOZ DC Lead OFF
      Enable_DcLeadOFF_Init(0b10, 0b0, 0b001, 0b00);
      break;
    }

    leadoffState++;
    leadoffState %= LEADOFF_NUMSTATES;

    max30001_LeadOffoldTime = currentTime;
  }
}
//******************************************************************************
#define LEADON_SERVICE_TIME 0x2000 // 0x1000 = 1 second
#define LEADON_NUMSTATES 2
uint32_t leadOnState = 0;
uint32_t max30001_LeadOnoldTime = 0;
void MAX30001::ServiceLeadON(uint32_t currentTime) {

  uint32_t delta_Time;

  delta_Time = currentTime - max30001_LeadOnoldTime;

  if (delta_Time > LEADON_SERVICE_TIME) {
    switch (leadOnState) {
    case 0: ///< switch to ECG DC Lead ON
      Enable_LeadON(0b01); 
      break;

    case 1: ///< switch to BIOZ DC Lead ON
      Enable_LeadON(0b10);
      break;
    }

    leadOnState++;
    leadOnState %= LEADON_NUMSTATES;

    max30001_LeadOnoldTime = currentTime;
  }
}

//******************************************************************************
int MAX30001::FIFO_LeadONOff_Read(void) {

  uint8_t result[32 * 3]; ///< 32words - 3bytes each
  uint8_t data_array[4];
  int32_t success = 0;
  int i, j;

  uint32_t total_databytes;
  uint8_t i_index;
  uint8_t data_chunk;
  uint8_t loop_logic;

  uint8_t etag, ptag, btag;

  uint8_t adr;

  int8_t ReadAllPaceOnce;

  static uint8_t dcloffint_OneShot = 0;
  static uint8_t acloffint_OneShot = 0;
  static uint8_t bcgmon_OneShot = 0;
  static uint8_t acleadon_OneShot = 0;

  max30001_mngr_int_t mngr_int;
  max30001_cnfg_gen_t cnfg_gen;

  int8_t ret_val;

  etag = 0;
  if (global_status.bit.eint == 1 || global_status.bit.pint == 1) {
    adr = ECG_FIFO_BURST;
    data_array[0] = ((adr << 1) & 0xff) | 1;

  ///< The SPI routine only sends out data of 32 bytes in size.  Therefore the
  ///< data is being read in
  ///<  smaller chunks in this routine...

  ///< READ  mngr_int  AND cnfg_gen;

  if (reg_read(MNGR_INT, &mngr_int.all) == -1) {
    return -1;
  }
  
  if (reg_read(CNFG_GEN, &cnfg_gen.all) == -1) {
    return -1;
  }  

    total_databytes = (mngr_int.bit.e_fit + 1) * 3;

    i_index = 0;
    loop_logic = 1;

    while (loop_logic) {
      if (total_databytes > 30) {
        data_chunk = 30;
        total_databytes = total_databytes - 30;
      } else {
        data_chunk = total_databytes;
        loop_logic = 0;
      }

      ///< The extra 1 byte is for the extra byte that comes out of the SPI
      success = SPI_Transmit(&data_array[0], 1, &result[i_index], (data_chunk + 1)); // Make a copy of the FIFO over here...

      if (success != 0) {
        return -1;
      }

      ///< This is important, because every transaction above creates an empty
      ///< redundant data at result[0]
      for (j = i_index; j < (data_chunk + i_index); j++) /* get rid of the 1 extra byte by moving the whole array up one */
      {
        result[j] = result[j + 1];
      }

      i_index = i_index + 30; /* point to the next array location to put the data in */
    }

    ReadAllPaceOnce = 0;

    ///< Put the content of the FIFO based on the EFIT value, We ignore the
    ///< result[0] and start concatenating indexes: 1,2,3 - 4,5,6 - 7,8,9 - 
    for (i = 0, j = 0; i < mngr_int.bit.e_fit + 1; i++, j = j + 3) ///< index1=23-16 bit, index2=15-8 bit, index3=7-0 bit
    {
      max30001_ECG_FIFO_buffer[i] = ((uint32_t)result[j] << 16) + (result[j + 1] << 8) + result[j + 2];

      etag = (0b00111000 & result[j + 2]) >> 3;
      ptag = 0b00000111 & result[j + 2];

      if (ptag != 0b111 && ReadAllPaceOnce == 0) {

        ReadAllPaceOnce = 1; ///< This will prevent extra read of PACE, once group
                             ///< 0-5 is read ONCE.

        adr = PACE0_FIFO_BURST;

        data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

        success = SPI_Transmit(&data_array[0], 1, &result[0], 10);

        max30001_PACE[0] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
        max30001_PACE[1] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
        max30001_PACE[2] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

        adr = PACE1_FIFO_BURST;

        data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

        success = SPI_Transmit(&data_array[0], 1, &result[0], 10);

        max30001_PACE[3] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
        max30001_PACE[4] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
        max30001_PACE[5] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

        adr = PACE2_FIFO_BURST;

        data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

        success = SPI_Transmit(&data_array[0], 1, &result[0], 10);

        max30001_PACE[6] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
        max30001_PACE[7] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
        max30001_PACE[8] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

        adr = PACE3_FIFO_BURST;

        data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

        success = SPI_Transmit(&data_array[0], 1, &result[0], 10);

        max30001_PACE[9]  = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
        max30001_PACE[10] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
        max30001_PACE[11] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

        adr = PACE4_FIFO_BURST;

        data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

        success = SPI_Transmit(&data_array[0], 1, &result[0], 10);

        max30001_PACE[12] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
        max30001_PACE[13] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
        max30001_PACE[14] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

        adr = PACE5_FIFO_BURST;

        data_array[0] = ((adr << 1) & 0xff) | 1; ///< For Read Or with 1

        success = SPI_Transmit(&data_array[0], 1, &result[0], 10);

        max30001_PACE[15] = (uint32_t)(result[1] << 16) + (result[2] << 8) + result[3];
        max30001_PACE[16] = (uint32_t)(result[4] << 16) + (result[5] << 8) + result[6];
        max30001_PACE[17] = (uint32_t)(result[7] << 16) + (result[8] << 8) + result[9];

        dataAvailable(MAX30001_DATA_PACE, max30001_PACE, 18); ///< Send out the Pace data once only
      }
    }

    if (etag != 0b110) {

      dataAvailable(MAX30001_DATA_ECG, max30001_ECG_FIFO_buffer, (mngr_int.bit.e_fit + 1));
    }

  } /* End of ECG init */

  /* RtoR */

  if (global_status.bit.rrint == 1) {
    if (reg_read(RTOR, &max30001_RtoR_data) == -1) {
      return -1;
    }

    max30001_RtoR_data = (0x00FFFFFF & max30001_RtoR_data) >> 10;

    hspValMax30001.R2R = (uint16_t)max30001_RtoR_data;
    hspValMax30001.fmstr = (uint16_t)cnfg_gen.bit.fmstr;

    dataAvailable(MAX30001_DATA_RTOR, &max30001_RtoR_data, 1);
  }

  ///< Handling BIOZ data...

  if (global_status.bit.bint == 1) {
    adr = 0x22;
    data_array[0] = ((adr << 1) & 0xff) | 1;

    ///< [(BFIT+1)*3byte]+1extra byte due to the addr

    if (SPI_Transmit(&data_array[0], 1, &result[0],((mngr_int.bit.b_fit + 1) * 3) + 1) == -1) { ///< Make a copy of the FIFO over here...
      return -1;
    }

    btag = 0b00000111 & result[3];

    ///< Put the content of the FIFO based on the BFIT value, We ignore the
    ///< result[0] and start concatenating indexes: 1,2,3 - 4,5,6 - 7,8,9 -
    for (i = 0, j = 0; i < mngr_int.bit.b_fit + 1; i++, j = j + 3) ///< index1=23-16 bit, index2=15-8 bit, index3=7-0 bit
    {
      max30001_BIOZ_FIFO_buffer[i] = ((uint32_t)result[j + 1] << 16) + (result[j + 2] << 8) + result[j + 3];
    }

    if (btag != 0b110) {
      dataAvailable(MAX30001_DATA_BIOZ, max30001_BIOZ_FIFO_buffer, 8);
    }
  }

  ret_val = 0;

  if (global_status.bit.dcloffint == 1) { ///< ECG/BIOZ Lead Off
    dcloffint_OneShot = 1;
    max30001_DCLeadOff = 0;
    max30001_DCLeadOff = max30001_DCLeadOff | (cnfg_gen.bit.en_dcloff << 8) | (global_status.all & 0x00000F);
    dataAvailable(MAX30001_DATA_LEADOFF_DC, &max30001_DCLeadOff, 1);
    ///< Do a FIFO Reset
    reg_write(FIFO_RST, 0x000000);

    ret_val = 0b100;

  } else if (dcloffint_OneShot == 1 && global_status.bit.dcloffint == 0) { ///< Just send once when it comes out of dc lead off
    max30001_DCLeadOff = 0;
    max30001_DCLeadOff = max30001_DCLeadOff | (cnfg_gen.bit.en_dcloff << 8) | (global_status.all & 0x00000F);
    dataAvailable(MAX30001_DATA_LEADOFF_DC, &max30001_DCLeadOff, 1);
    dcloffint_OneShot = 0;
  }

  if (global_status.bit.bover == 1 || global_status.bit.bundr == 1) { ///< BIOZ AC Lead Off
    acloffint_OneShot = 1;
    max30001_ACLeadOff = 0;
    max30001_ACLeadOff =
        max30001_ACLeadOff | ((global_status.all & 0x030000) >> 16);
    dataAvailable(MAX30001_DATA_LEADOFF_AC, &max30001_ACLeadOff, 1);
    ///< Do a FIFO Reset
    reg_write(FIFO_RST, 0x000000);

    ret_val = 0b1000;
  } else if (acloffint_OneShot == 1 && global_status.bit.bover == 0 && global_status.bit.bundr == 0) { ///< Just send once when it comes out of ac lead off
    max30001_ACLeadOff = 0;
    max30001_ACLeadOff = max30001_ACLeadOff | ((global_status.all & 0x030000) >> 16);
    dataAvailable(MAX30001_DATA_LEADOFF_AC, &max30001_ACLeadOff, 1);
    acloffint_OneShot = 0;
  }

  if (global_status.bit.bcgmon == 1) {///< BIOZ BCGMON check
    bcgmon_OneShot = 1;
    max30001_bcgmon = 0;
    max30001_bcgmon = max30001_bcgmon | ((global_status.all & 0x000030) >> 4);
    dataAvailable(MAX30001_DATA_BCGMON, &max30001_bcgmon, 1);
    // Do a FIFO Reset
    reg_write(FIFO_RST, 0x000000);

    ret_val = 0b10000;
  } else if (bcgmon_OneShot == 1 && global_status.bit.bcgmon == 0) {
    max30001_bcgmon = 0;
    max30001_bcgmon = max30001_bcgmon | ((global_status.all & 0x000030) >> 4);
    bcgmon_OneShot = 0;
    dataAvailable(MAX30001_DATA_BCGMON, &max30001_bcgmon, 1);
  }

  if (global_status.bit.lonint == 1 && acleadon_OneShot == 0) {///< AC LeadON Check, when lead is on
    max30001_LeadOn = 0;
    reg_read(STATUS, &global_status.all);
    max30001_LeadOn =
        max30001_LeadOn | (cnfg_gen.bit.en_ulp_lon << 8) |
        ((global_status.all & 0x000800) >>
         11); ///< 0b01 will mean ECG Lead On, 0b10 will mean BIOZ Lead On

    // LEAD ON has been detected... Now take actions
    acleadon_OneShot = 1;
    dataAvailable(MAX30001_DATA_ACLEADON, &max30001_LeadOn, 1); ///< One shot data will be sent...
  } else if (global_status.bit.lonint == 0 && acleadon_OneShot == 1) {
    max30001_LeadOn = 0;
    reg_read(STATUS, &global_status.all);
    max30001_LeadOn =
        max30001_LeadOn | (cnfg_gen.bit.en_ulp_lon << 8) | ((global_status.all & 0x000800) >> 11); ///< 0b01 will mean ECG Lead On, 0b10 will mean BIOZ Lead On
    dataAvailable(MAX30001_DATA_ACLEADON, &max30001_LeadOn, 1); ///< One shot data will be sent...
    acleadon_OneShot = 0;
  }

  return ret_val;
}

//******************************************************************************
int MAX30001::int_handler(void) {

  static uint32_t InitReset = 0;

  int8_t return_value;

  reg_read(STATUS, &global_status.all);

  ///< Inital Reset and any FIFO over flow invokes a FIFO reset
  if (InitReset == 0 || global_status.bit.eovf == 1 || global_status.bit.bovf == 1 || global_status.bit.povf == 1) {
    ///< Do a FIFO Reset
    reg_write(FIFO_RST, 0x000000);

    InitReset++;
    return 2;
  }

  return_value = 0;

  ///< The four data handling goes on over here
  if (global_status.bit.eint == 1 || global_status.bit.pint == 1 || global_status.bit.bint == 1 || global_status.bit.rrint == 1) {
    return_value = return_value | FIFO_LeadONOff_Read();
  }

  ///< ECG/BIOZ DC Lead Off test
  if (global_status.bit.dcloffint == 1) {
    return_value = return_value | FIFO_LeadONOff_Read();
  }

  ///< BIOZ AC Lead Off test
  if (global_status.bit.bover == 1 || global_status.bit.bundr == 1) {
    return_value = return_value | FIFO_LeadONOff_Read();
  }

  ///< BIOZ DRVP/N test using BCGMON.
  if (global_status.bit.bcgmon == 1) {
    return_value = return_value | FIFO_LeadONOff_Read();
  }

  if (global_status.bit.lonint == 1) ///< ECG Lead ON test: i.e. the leads are touching the body...
  {

    FIFO_LeadONOff_Read();
  }

  return return_value;
}


event_callback_t MAX30001::functionpointer;


volatile int MAX30001::xferFlag = 0;


//******************************************************************************
int MAX30001::SPI_Transmit(const uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf, uint32_t rx_size) {
  xferFlag = 0;
  unsigned int i;
  for (i = 0; i < sizeof(buffer); i++) {
    if (i < tx_size) {
      buffer[i] = tx_buf[i];
    }
    else {
      buffer[i] = 0xFF;
    }
  }
  spi->transfer<uint8_t>(buffer, (int)rx_size, rx_buf, (int)rx_size, spiHandler);
  while (xferFlag == 0);
  return 0;
}

//******************************************************************************
void MAX30001::ReadHeartrateData(max30001_bledata_t *_hspValMax30001) {
  _hspValMax30001->R2R = hspValMax30001.R2R;
  _hspValMax30001->fmstr = hspValMax30001.fmstr;
}

//******************************************************************************
void MAX30001::onDataAvailable(PtrFunction _onDataAvailable) {
  onDataAvailableCallback = _onDataAvailable;
}

//******************************************************************************
void MAX30001::dataAvailable(uint32_t id, uint32_t *buffer, uint32_t length) {
  if (onDataAvailableCallback != NULL) {
    (*onDataAvailableCallback)(id, buffer, length);
  }
}

//******************************************************************************
void MAX30001::spiHandler(int events) {
	 xferFlag = 1;
}

//******************************************************************************
static int allowInterrupts = 0;

void MAX30001::Mid_IntB_Handler(void) {
  if (allowInterrupts == 0) {
  	return;
  	}
  MAX30001::instance->int_handler();
}

void MAX30001::Mid_Int2B_Handler(void) {  
  if (allowInterrupts == 0) {
  	return;
  	}
  MAX30001::instance->int_handler();
}

void MAX30001::AllowInterrupts(int state) { 
allowInterrupts = state; 
}
