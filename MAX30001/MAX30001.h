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
 *******************************************************************************/
/**
 *
 * Maxim Integrated MAX30001 ECG/BIOZ chip 
 *
 * @code
 * #include "mbed.h"
 * #include "MAX30001.h"
 * 
 * /// Initialization values for ECG_InitStart()
 * #define EN_ECG     0b1
 * #define OPENP      0b1
 * #define OPENN      0b1
 * #define POL        0b0
 * #define CALP_SEL   0b10
 * #define CALN_SEL   0b11
 * #define E_FIT      31
 * #define RATE       0b00
 * #define GAIN       0b00
 * #define DHPF       0b0
 * #define DLPF       0b01
 * 
 * /// Initialization values for CAL_InitStart() 
 * #define EN_VCAL  0b1
 * #define VMODE    0b1
 * #define VMAG     0b1
 * #define FCAL     0b011
 * #define THIGH    0x7FF
 * #define FIFTY    0b0
 * 
 * /// Initializaton values for Rbias_FMSTR_Init()
 * #define EN_RBIAS 0b01 
 * #define RBIASV   0b10
 * #define RBIASP   0b1
 * #define RBIASN   0b1
 * #define FMSTR    0b00
 * 
 * #define BUFFER_LENGTH 50
 * 
 * // @brief SPI Master 0 with SPI0_SS for use with MAX30001
 * SPI spi(SPI0_MOSI, SPI0_MISO, SPI0_SCK, SPI0_SS); // used by MAX30001
 * 
 * //@brief ECG device
 * MAX30001 max30001(&spi);
 * InterruptIn max30001_InterruptB(P3_6);
 * InterruptIn max30001_Interrupt2B(P4_5);
 * //@brief PWM used as fclk for the MAX30001
 * PwmOut pwmout(P1_7);
 * 
 * //@brief Creating a buffer to hold the data
 * uint32_t ecgBuffer[BUFFER_LENGTH];
 * int ecgIndex = 0;
 * char data_trigger = 0;
 * 
 * 
 * //
 * // @brief Creates a packet that will be streamed via USB Serial
 * //       the packet created will be inserted into a fifo to be streamed at a later time
 * // @param id Streaming ID
 * // @param buffer Pointer to a uint32 array that contains the data to include in the packet
 * // @param number Number of elements in the buffer
 * //
 * void StreamPacketUint32_ecg(uint32_t id, uint32_t *buffer, uint32_t number) {
 *   int i;
 *   if (id == MAX30001_DATA_ECG) {
 *     for (i = 0; i < number; i++) {
 *       ecgBuffer[ecgIndex] = buffer[i];
 *       ecgIndex++;
 *       if (ecgIndex > BUFFER_LENGTH)
 *         {
 *         data_trigger = 1;
 *         ecgIndex = 0;
 *         }
 *     }
 *   }
 *   if (id == MAX30001_DATA_BIOZ) {
 *         /// Add code for reading BIOZ data
 *   }
 *   if (id == MAX30001_DATA_PACE) {
 *         ///  Add code for reading Pace data
 *   }
 *   if (id == MAX30001_DATA_RTOR) {
 *         /// Add code for reading RtoR data
 *   }
 * }
 * 
 * 
 * int main() {
 * 
 *   uint32_t all;
 * 
 *   /// set NVIC priorities for GPIO to prevent priority inversion
 *   NVIC_SetPriority(GPIO_P0_IRQn, 5);
 *   NVIC_SetPriority(GPIO_P1_IRQn, 5);
 *   NVIC_SetPriority(GPIO_P2_IRQn, 5);
 *   NVIC_SetPriority(GPIO_P3_IRQn, 5);
 *   NVIC_SetPriority(GPIO_P4_IRQn, 5);
 *   NVIC_SetPriority(GPIO_P5_IRQn, 5);
 *   NVIC_SetPriority(GPIO_P6_IRQn, 5);
 *   // used by the MAX30001
 *   NVIC_SetPriority(SPI1_IRQn, 0);
 * 
 * 
 *   /// Setup interrupts and callback functions
 *   max30001_InterruptB.disable_irq();
 *   max30001_Interrupt2B.disable_irq();
 * 
 *   max30001_InterruptB.mode(PullUp);
 *   max30001_InterruptB.fall(&MAX30001::Mid_IntB_Handler);
 * 
 *   max30001_Interrupt2B.mode(PullUp);
 *   max30001_Interrupt2B.fall(&MAX30001::Mid_Int2B_Handler);
 * 
 *   max30001_InterruptB.enable_irq();
 *   max30001_Interrupt2B.enable_irq();
 * 
 *   max30001.AllowInterrupts(1);
 * 
 *   // Configuring the FCLK for the ECG, set to 32.768KHZ
 *   pwmout.period_us(31);
 *   pwmout.write(0.5);     // 0-1 is 0-100%, 0.5 = 50% duty cycle.
 *   max30001.sw_rst();     // Do a software reset of the MAX30001
 *   
 *   max30001.INT_assignment(MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,   MAX30001::MAX30001_NO_INT,  //  en_enint_loc,      en_eovf_loc,   en_fstint_loc,
 *                           MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_NO_INT,  //  en_dcloffint_loc,  en_bint_loc,   en_bovf_loc,
 *                           MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_NO_INT,  //  en_bover_loc,      en_bundr_loc,  en_bcgmon_loc,
 *                           MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,   MAX30001::MAX30001_NO_INT,  //  en_pint_loc,       en_povf_loc,   en_pedge_loc,
 *                           MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,  //  en_lonint_loc,     en_rrint_loc,  en_samp_loc,
 *                           MAX30001::MAX30001_INT_ODNR, MAX30001::MAX30001_INT_ODNR);                            //  intb_Type,         int2b_Type)
 * 
 *   max30001.onDataAvailable(&StreamPacketUint32_ecg);  
 * 
 *   /// Set and Start the VCAL input
 *   /// @brief NOTE VCAL must be set first if VCAL is to be used 
 *     max30001.CAL_InitStart(EN_VCAL , VMODE, VMAG, FCAL, THIGH, FIFTY);
 *           
 *   /// ECG Initialization
 *     max30001.ECG_InitStart(EN_ECG, OPENP, OPENN, POL, CALP_SEL, CALN_SEL, E_FIT, RATE, GAIN, DHPF, DLPF);
 * 
 *   /// @details The user can call any of the InitStart functions for Pace, BIOZ and RtoR
 * 
 * 
 *   /// @brief Set Rbias & FMSTR over here
 *       max30001.Rbias_FMSTR_Init(EN_RBIAS, RBIASV, RBIASP, RBIASN,FMSTR);
 * 
 *   max30001.synch();
 * 
 *   /// clear the status register for a clean start
 *   max30001.reg_read(MAX30001::STATUS, &all);  
 *   
 *   printf("Please wait for data to start streaming\n");
 *   fflush(stdout);
 *     
 *   while (1) {
 *     if(data_trigger == 1){
 *     printf("%ld ", ecgBuffer[ecgIndex]);  // Print the ECG data on a serial port terminal software
 *     fflush(stdout);
 *     }
 *   }
 * }
 * @endcode 
 *
 */


#ifndef MAX30001_H_
#define MAX30001_H_

#include "mbed.h"

#define mbed_COMPLIANT ///< Uncomment to Use timer for MAX30001 FCLK (for mbed)
                       ///< Comment to use the RTC clock

#define ASYNC_SPI_BUFFER_SIZE (32 * 3) ///< Maximimum buffer size for async byte transfers

///< Defines for data callbacks
#define MAX30001_DATA_ECG        0x30
#define MAX30001_DATA_PACE       0x31
#define MAX30001_DATA_RTOR       0x32
#define MAX30001_DATA_BIOZ       0x33
#define MAX30001_DATA_LEADOFF_DC 0x34
#define MAX30001_DATA_LEADOFF_AC 0x35
#define MAX30001_DATA_BCGMON     0x36
#define MAX30001_DATA_ACLEADON   0x37

#define MAX30001_SPI_MASTER_PORT 0
#define MAX30001_SPI_SS_INDEX    0

#define MAX30001_INT_PORT_B      3
#define MAX30001_INT_PIN_B       6

#define MAX30001_INT_PORT_2B     4
#define MAX30001_INT_PIN_2B      5

#define MAX30001_INT_PORT_FCLK   1
#define MAX30001_INT_PIN_FCLK    7

#define MAX30001_FUNC_SEL_TMR    2 ///< 0=FW Control, 1= Pulse Train, 2=Timer

#define MAX30001_INDEX    3
#define MAX30001_POLARITY 0
#define MAX30001_PERIOD   30518
#define MAX30001_CYCLE    50

#define MAX30001_IOMUX_IO_ENABLE 1

#define MAX30001_SPI_PORT            0
#define MAX30001_CS_PIN              0
#define MAX30001_CS_POLARITY         0
#define MAX30001_CS_ACTIVITY_DELAY   0
#define MAX30001_CS_INACTIVITY_DELAY 0
#define MAX30001_CLK_HI              1
#define MAX30001_CLK_LOW             1
#define MAX30001_ALT_CLK             0
#define MAX30001_CLK_POLARITY        0
#define MAX30001_CLK_PHASE           0
#define MAX30001_WRITE               1
#define MAX30001_READ                0

#define MAX30001_INT_PORT_B  3
#define MAX30001INT_PIN_B    6

void MAX30001_AllowInterrupts(int state);

/**
* @brief Maxim Integrated MAX30001 ECG/BIOZ chip
*/
class MAX30001 {

public:
  typedef enum { ///< MAX30001 Register addresses
    STATUS     = 0x01,
    EN_INT     = 0x02,
    EN_INT2    = 0x03,
    MNGR_INT   = 0x04,
    MNGR_DYN   = 0x05,
    SW_RST     = 0x08,
    SYNCH      = 0x09,
    FIFO_RST   = 0x0A,
    INFO       = 0x0F,
    CNFG_GEN   = 0x10,
    CNFG_CAL   = 0x12,
    CNFG_EMUX  = 0x14,
    CNFG_ECG   = 0x15,
    CNFG_BMUX  = 0x17,
    CNFG_BIOZ  = 0x18,
    CNFG_PACE  = 0x1A,
    CNFG_RTOR1 = 0x1D,
    CNFG_RTOR2 = 0x1E,

    // Data locations
    ECG_FIFO_BURST = 0x20,
    ECG_FIFO       = 0x21,
    FIFO_BURST     = 0x22,
    BIOZ_FIFO      = 0x23,
    RTOR           = 0x25,

    PACE0_FIFO_BURST = 0x30,
    PACE0_A          = 0x31,
    PACE0_B          = 0x32,
    PACE0_C          = 0x33,

    PACE1_FIFO_BURST = 0x34,
    PACE1_A          = 0x35,
    PACE1_B          = 0x36,
    PACE1_C          = 0x37,

    PACE2_FIFO_BURST = 0x38,
    PACE2_A          = 0x39,
    PACE2_B          = 0x3A,
    PACE2_C          = 0x3B,

    PACE3_FIFO_BURST = 0x3C,
    PACE3_A          = 0x3D,
    PACE3_B          = 0x3E,
    PACE3_C          = 0x3F,

    PACE4_FIFO_BURST = 0x40,
    PACE4_A          = 0x41,
    PACE4_B          = 0x42,
    PACE4_C          = 0x43,

    PACE5_FIFO_BURST = 0x44,
    PACE5_A          = 0x45,
    PACE5_B          = 0x46,
    PACE5_C          = 0x47,

  } MAX30001_REG_map_t;

  /**
   * @brief STATUS (0x01) 
   */
  typedef union max30001_status_reg {
    uint32_t all;

    struct {
      uint32_t loff_nl : 1;
      uint32_t loff_nh : 1;
      uint32_t loff_pl : 1;
      uint32_t loff_ph : 1;

      uint32_t bcgmn     : 1;
      uint32_t bcgmp     : 1;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 1;

      uint32_t pllint : 1;
      uint32_t samp   : 1;
      uint32_t rrint  : 1;
      uint32_t lonint : 1;

      uint32_t pedge  : 1;
      uint32_t povf   : 1;
      uint32_t pint   : 1;
      uint32_t bcgmon : 1;

      uint32_t bundr : 1;
      uint32_t bover : 1;
      uint32_t bovf  : 1;
      uint32_t bint  : 1;

      uint32_t dcloffint : 1;
      uint32_t fstint    : 1;
      uint32_t eovf      : 1;
      uint32_t eint      : 1;

      uint32_t reserved : 8;

    } bit;

  } max30001_status_t;

  
  /**
   * @brief EN_INT (0x02) 
   */

  typedef union max30001_en_int_reg {
    uint32_t all;

    struct {
      uint32_t intb_type : 2;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 1;

      uint32_t reserved3 : 1;
      uint32_t reserved4 : 1;
      uint32_t reserved5 : 1;
      uint32_t reserved6 : 1;

      uint32_t en_pllint : 1;
      uint32_t en_samp   : 1;
      uint32_t en_rrint  : 1;
      uint32_t en_lonint : 1;

      uint32_t en_pedge  : 1;
      uint32_t en_povf   : 1;
      uint32_t en_pint   : 1;
      uint32_t en_bcgmon : 1;

      uint32_t en_bundr : 1;
      uint32_t en_bover : 1;
      uint32_t en_bovf  : 1;
      uint32_t en_bint  : 1;

      uint32_t en_dcloffint : 1;
      uint32_t en_fstint    : 1;
      uint32_t en_eovf      : 1;
      uint32_t en_eint      : 1;

      uint32_t reserved : 8;

    } bit;

  } max30001_en_int_t;

  
  /**
   * @brief EN_INT2 (0x03) 
   */  
  typedef union max30001_en_int2_reg {
    uint32_t all;

    struct {
      uint32_t intb_type : 2;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 1;

      uint32_t reserved3 : 1;
      uint32_t reserved4 : 1;
      uint32_t reserved5 : 1;
      uint32_t reserved6 : 1;

      uint32_t en_pllint : 1;
      uint32_t en_samp   : 1;
      uint32_t en_rrint  : 1;
      uint32_t en_lonint : 1;

      uint32_t en_pedge  : 1;
      uint32_t en_povf   : 1;
      uint32_t en_pint   : 1;
      uint32_t en_bcgmon : 1;

      uint32_t en_bundr  : 1;
      uint32_t en_bover  : 1;
      uint32_t en_bovf   : 1;
      uint32_t en_bint   : 1;

      uint32_t en_dcloffint : 1;
      uint32_t en_fstint    : 1;
      uint32_t en_eovf      : 1;
      uint32_t en_eint      : 1;

      uint32_t reserved : 8;

    } bit;

  } max30001_en_int2_t;

  /**
   * @brief MNGR_INT (0x04) 
   */  
  typedef union max30001_mngr_int_reg {
    uint32_t all;

    struct {
      uint32_t samp_it   : 2;
      uint32_t clr_samp  : 1;
      uint32_t clr_pedge : 1;
      uint32_t clr_rrint : 2;
      uint32_t clr_fast  : 1;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 4;
      uint32_t reserved3 : 4;

      uint32_t b_fit     : 3;
      uint32_t e_fit     : 5;

      uint32_t reserved : 8;

    } bit;

  } max30001_mngr_int_t;

   /**
   * @brief MNGR_DYN (0x05) 
   */ 
  typedef union max30001_mngr_dyn_reg {
    uint32_t all;

    struct {
      uint32_t bloff_lo_it : 8;
      uint32_t bloff_hi_it : 8;
      uint32_t fast_th     : 6;
      uint32_t fast        : 2;
      uint32_t reserved    : 8;
    } bit;

  } max30001_mngr_dyn_t;

  
   /**
   * @brief INFO (0x0F) 
   */
  typedef union max30001_info_reg {
    uint32_t all;
    struct {
      uint32_t serial    : 12;
      uint32_t part_id   : 2;
      uint32_t sample    : 1;
      uint32_t reserved1 : 1;
      uint32_t rev_id    : 4;
      uint32_t pattern   : 4;
      uint32_t reserved  : 8;
    } bit;

  } max30001_info_t;

   /**
   * @brief CNFG_GEN (0x10) 
   */
  typedef union max30001_cnfg_gen_reg {
    uint32_t all;
    struct {
      uint32_t rbiasn     : 1;
      uint32_t rbiasp     : 1;
      uint32_t rbiasv     : 2;
      uint32_t en_rbias   : 2;
      uint32_t vth        : 2;
      uint32_t imag       : 3;
      uint32_t ipol       : 1;
      uint32_t en_dcloff  : 2;
      uint32_t en_bloff   : 2;
      uint32_t reserved1  : 1;
      uint32_t en_pace    : 1;
      uint32_t en_bioz    : 1;
      uint32_t en_ecg     : 1;
      uint32_t fmstr      : 2;
      uint32_t en_ulp_lon : 2;
      uint32_t reserved : 8;
    } bit;

  } max30001_cnfg_gen_t;

  
   /**
   * @brief CNFG_CAL (0x12) 
   */  
  typedef union max30001_cnfg_cal_reg {
    uint32_t all;
    struct {
      uint32_t thigh     : 11;
      uint32_t fifty     : 1;
      uint32_t fcal      : 3;
      uint32_t reserved1 : 5;
      uint32_t vmag      : 1;
      uint32_t vmode     : 1;
      uint32_t en_vcal   : 1;
      uint32_t reserved2 : 1;
      uint32_t reserved  : 8;
    } bit;

  } max30001_cnfg_cal_t;

   /**
   * @brief CNFG_EMUX  (0x14) 
   */
  typedef union max30001_cnfg_emux_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 16;
      uint32_t caln_sel  : 2;
      uint32_t calp_sel  : 2;
      uint32_t openn     : 1;
      uint32_t openp     : 1;
      uint32_t reserved2 : 1;
      uint32_t pol       : 1;
      uint32_t reserved : 8;
    } bit;

  } max30001_cnfg_emux_t;

  
   /**
   * @brief CNFG_ECG   (0x15) 
   */  
  typedef union max30001_cnfg_ecg_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 12;
      uint32_t dlpf      : 2;
      uint32_t dhpf      : 1;
      uint32_t reserved2 : 1;
      uint32_t gain      : 2;
      uint32_t reserved3 : 4;
      uint32_t rate      : 2;

      uint32_t reserved  : 8;
    } bit;

  } max30001_cnfg_ecg_t;

   /**
   * @brief CNFG_BMUX   (0x17) 
   */  
  typedef union max30001_cnfg_bmux_reg {
    uint32_t all;
    struct {
      uint32_t fbist     : 2;
      uint32_t reserved1 : 2;
      uint32_t rmod      : 3;
      uint32_t reserved2 : 1;
      uint32_t rnom      : 3;
      uint32_t en_bist   : 1;
      uint32_t cg_mode   : 2;
      uint32_t reserved3 : 2;
      uint32_t caln_sel  : 2;
      uint32_t calp_sel  : 2;
      uint32_t openn     : 1;
      uint32_t openp     : 1;
      uint32_t reserved4 : 2;
      uint32_t reserved  : 8;
    } bit;

  } max30001_cnfg_bmux_t;

   /**
   * @brief CNFG_BIOZ   (0x18) 
   */ 
  typedef union max30001_bioz_reg {
    uint32_t all;
    struct {
      uint32_t phoff     : 4;
      uint32_t cgmag     : 3;
      uint32_t cgmon     : 1;
      uint32_t fcgen     : 4;
      uint32_t dlpf      : 2;
      uint32_t dhpf      : 2;
      uint32_t gain      : 2;
      uint32_t reserved1 : 1;
      uint32_t ext_rbias : 1;
      uint32_t ahpf      : 3;
      uint32_t rate      : 1;
      uint32_t reserved  : 8;
    } bit;

  } max30001_cnfg_bioz_t;

  
   /**
   * @brief CNFG_PACE   (0x1A) 
   */   
  typedef union max30001_cnfg_pace_reg {
    uint32_t all;

    struct {
      uint32_t dacn        : 4;
      uint32_t dacp        : 4;
      uint32_t reserved1   : 4;
      uint32_t aout        : 2;
      uint32_t aout_lbw    : 1;
      uint32_t reserved2   : 1;
      uint32_t gain        : 3;
      uint32_t gn_diff_off : 1;
      uint32_t reserved3   : 3;
      uint32_t pol         : 1;
      uint32_t reserved    : 8;
    } bit;

  } max30001_cnfg_pace_t;

   /**
   * @brief CNFG_RTOR1   (0x1D) 
   */   
  typedef union max30001_cnfg_rtor1_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 8;
      uint32_t ptsf      : 4;
      uint32_t pavg      : 2;
      uint32_t reserved2 : 1;
      uint32_t en_rtor   : 1;
      uint32_t gain      : 4;
      uint32_t wndw      : 4;
      uint32_t reserved  : 8;
    } bit;

  } max30001_cnfg_rtor1_t;

   /**
   * @brief CNFG_RTOR2 (0x1E) 
   */   
  typedef union max30001_cnfg_rtor2_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 8;
      uint32_t rhsf      : 3;
      uint32_t reserved2 : 1;
      uint32_t ravg      : 2;
      uint32_t reserved3 : 2;
      uint32_t hoff      : 6;
      uint32_t reserved4 : 2;
      uint32_t reserved : 8;
    } bit;

  } max30001_cnfg_rtor2_t;

  /*********************************************************************************/

  typedef enum {
    MAX30001_NO_INT = 0, ///< No interrupt
    MAX30001_INT_B  = 1, ///< INTB selected for interrupt
    MAX30001_INT_2B = 2  ///< INT2B selected for interrupt
  } max30001_intrpt_Location_t;

  typedef enum {
    MAX30001_INT_DISABLED = 0b00,
    MAX30001_INT_CMOS     = 0b01,
    MAX30001_INT_ODN      = 0b10,
    MAX30001_INT_ODNR     = 0b11
  } max30001_intrpt_type_t;

  typedef enum {          ///< Input Polarity selection
    MAX30001_NON_INV = 0, ///< Non-Inverted
    MAX30001_INV     = 1  ///< Inverted
  } max30001_emux_pol_t;

  typedef enum {              ///< OPENP and OPENN setting
    MAX30001_ECG_CON_AFE = 0, ///< ECGx is connected to AFE channel
    MAX30001_ECG_ISO_AFE = 1  ///< ECGx is isolated from AFE channel
  } max30001_emux_openx_t;

  typedef enum {                ///< EMUX_CALP_SEL & EMUX_CALN_SEL
    MAX30001_NO_CAL_SIG = 0b00, ///< No calibration signal is applied
    MAX30001_INPT_VMID  = 0b01, ///< Input is connected to VMID
    MAX30001_INPT_VCALP = 0b10, ///< Input is connected to VCALP
    MAX30001_INPT_VCALN = 0b11  ///< Input is connected to VCALN
  } max30001_emux_calx_sel_t;

  typedef enum {                     ///< EN_ECG, EN_BIOZ, EN_PACE
    MAX30001_CHANNEL_DISABLED = 0b0,
    MAX30001_CHANNEL_ENABLED = 0b1
  } max30001_en_feature_t;

  /*********************************************************************************/
  // Data
  uint32_t max30001_ECG_FIFO_buffer[32]; ///< (303 for internal test)
  uint32_t max30001_BIOZ_FIFO_buffer[8]; ///< (303 for internal test)

  uint32_t max30001_PACE[18]; ///< Pace Data 0-5

  uint32_t max30001_RtoR_data; ///< This holds the RtoR data

  uint32_t max30001_DCLeadOff; ///< This holds the LeadOff data, Last 4 bits give
                               ///< the status, BIT3=LOFF_PH, BIT2=LOFF_PL,
                               ///< BIT1=LOFF_NH, BIT0=LOFF_NL
                               ///< 8th and 9th bits tell Lead off is due to ECG or BIOZ.  
                               ///< 0b01 = ECG Lead Off and 0b10 = BIOZ Lead off

  uint32_t max30001_ACLeadOff; ///< This gives the state of the BIOZ AC Lead Off
                               ///< state.  BIT 1 = BOVER,   BIT 0 = BUNDR

  uint32_t max30001_bcgmon; ///< This holds the BCGMON data, BIT 1 = BCGMP, BIT0 =
                            ///< BCGMN

  uint32_t max30001_LeadOn; ///< This holds the LeadOn data, BIT1 = BIOZ Lead ON,
                            ///< BIT0 = ECG Lead ON, BIT8= Lead On Status Bit

  uint32_t max30001_timeout; ///< If the PLL does not respond, timeout and get out.

  typedef struct { ///< Creating a structure for BLE data
    int16_t R2R;
    int16_t fmstr;
  } max30001_bledata_t;

  max30001_bledata_t hspValMax30001; // R2R, FMSTR

  max30001_status_t global_status;

  /**
  * @brief Constructor that accepts pin names for the SPI interface
  * @param spi pointer to the mbed SPI object
  */
  MAX30001(SPI *spi);

  /**
  * @brief Constructor that accepts pin names for the SPI interface
  * @param mosi master out slave in pin name
  * @param miso master in slave out pin name
  * @param sclk serial clock pin name
  * @param cs chip select pin name
  */
  MAX30001(PinName mosi, PinName miso, PinName sclk, PinName cs);

  /**
   * MAX30001 destructor
   */
  ~MAX30001(void);
  
  /**
   * @brief This function is MAXIM Proprietary.  It channels the RTC crystal
   * @brief clock to P1.7.  Thus providing 32768Hz on FCLK pin of the MAX30001-3
   */ 
  void FCLK_MaximOnly(void);   
  
  /**
   * @brief This function sets up the Resistive Bias mode and also selects the master clock frequency.
   * @brief Uses Register: CNFG_GEN-0x10
   * @param En_rbias: Enable and Select Resitive Lead Bias Mode
   * @param Rbiasv: Resistive Bias Mode Value Selection
   * @param Rbiasp: Enables Resistive Bias on Positive Input
   * @param Rbiasn: Enables Resistive Bias on Negative Input
   * @param Fmstr: Selects Master Clock Frequency
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
  */
  int Rbias_FMSTR_Init(uint8_t En_rbias, uint8_t Rbiasv,
                                uint8_t Rbiasp, uint8_t Rbiasn, uint8_t Fmstr);

  /**
   * @brief This function uses sets up the calibration signal internally.  If it is desired to use the internal signal, then
   * @brief this function must be called and the registers set, prior to setting the CALP_SEL and CALN_SEL in the ECG_InitStart
   * @brief and BIOZ_InitStart functions.
   * @brief Uses Register: CNFG_CAL-0x12
   * @param En_Vcal: Calibration Source (VCALP and VCALN) Enable
   * @param Vmode:   Calibration Source Mode Selection
   * @param Vmag:    Calibration Source Magnitude Selection (VMAG)
   * @param Fcal:    Calibration Source Frequency Selection (FCAL)
   * @param Thigh:   Calibration Source Time High Selection
   * @param Fifty:   Calibration Source Duty Cycle Mode Selection
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int CAL_InitStart(uint8_t En_Vcal, uint8_t Vmode, uint8_t Vmag,
                             uint8_t Fcal, uint16_t Thigh, uint8_t Fifty);

  /**
   * @brief This function disables the VCAL signal
   * @returns 0-if no error.  A non-zero value indicates an error.
   */
  int CAL_Stop(void);

  /**
   * @brief This function handles the assignment of the two interrupt pins (INTB & INT2B) with various
   * @brief functions/behaviors  of the MAX30001.  Also, each pin can be configured for different drive capability.
   * @brief Uses Registers: EN_INT-0x02 and EN_INT2-0x03.
   * @param max30001_intrpt_Locatio_t  <argument>:  All the arguments with the aforementioned enumeration essentially
   *        can be configured to generate an interrupt on either INTB or INT2B or NONE.
   * @param max30001_intrpt_type_t  intb_Type:  INTB Port Type (EN_INT Selections).
   * @param max30001_intrpt_type _t int2b_Type:   INT2B Port Type (EN_INT2 Selections)
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
     int INT_assignment(max30001_intrpt_Location_t en_enint_loc,     max30001_intrpt_Location_t en_eovf_loc,  max30001_intrpt_Location_t en_fstint_loc,
		                 max30001_intrpt_Location_t en_dcloffint_loc, max30001_intrpt_Location_t en_bint_loc,  max30001_intrpt_Location_t en_bovf_loc,
		                 max30001_intrpt_Location_t en_bover_loc,     max30001_intrpt_Location_t en_bundr_loc, max30001_intrpt_Location_t en_bcgmon_loc,
		                 max30001_intrpt_Location_t en_pint_loc,      max30001_intrpt_Location_t en_povf_loc,  max30001_intrpt_Location_t en_pedge_loc,
		                 max30001_intrpt_Location_t en_lonint_loc,    max30001_intrpt_Location_t en_rrint_loc, max30001_intrpt_Location_t en_samp_loc,
		                 max30001_intrpt_type_t  intb_Type,           max30001_intrpt_type_t int2b_Type);



  /**
   * @brief For MAX30001/3 ONLY
   * @brief This function sets up the MAX30001 for the ECG measurements.
   * @brief Registers used:  CNFG_EMUX, CNFG_GEN, MNGR_INT, CNFG_ECG.
   * @param En_ecg: ECG Channel Enable <CNFG_GEN register bits>
   * @param Openp: Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register bits>
   * @param Openn: Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register bits>
   * @param Calp_sel: ECGP Calibration Selection <CNFG_EMUX register bits>
   * @param Caln_sel: ECGN Calibration Selection <CNFG_EMUX register bits>
   * @param E_fit: ECG FIFO Interrupt Threshold (issues EINT based on number of unread FIFO records) <CNFG_GEN register bits>
   * @param Clr_rrint: RTOR R Detect Interrupt (RRINT) Clear Behavior <CNFG_GEN register bits>
   * @param Rate: ECG Data Rate
   * @param Gain: ECG Channel Gain Setting
   * @param Dhpf: ECG Channel Digital High Pass Filter Cutoff Frequency
   * @param Dlpf:  ECG Channel Digital Low Pass Filter Cutoff Frequency
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int ECG_InitStart(uint8_t En_ecg, uint8_t Openp, uint8_t Openn,
                    uint8_t Pol, uint8_t Calp_sel, uint8_t Caln_sel,
                    uint8_t E_fit, uint8_t Rate, uint8_t Gain,
                    uint8_t Dhpf, uint8_t Dlpf);

  /**
   * @brief For MAX30001/3 ONLY
   * @brief This function enables the Fast mode feature of the ECG.
   * @brief Registers used: MNGR_INT-0x04, MNGR_DYN-0x05
   * @param Clr_Fast: FAST MODE Interrupt Clear Behavior <MNGR_INT Register>
   * @param Fast: ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass) <MNGR_DYN Register>
   * @param Fast_Th: Automatic Fast Recovery Threshold
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int ECGFast_Init(uint8_t Clr_Fast, uint8_t Fast, uint8_t Fast_Th);

  /**
  * @brief For MAX30001/3 ONLY
  * @brief This function disables the ECG.
  * @brief Uses Register CNFG_GEN-0x10.
  * @returns 0-if no error.  A non-zero value indicates an error.
  *
  */
  int Stop_ECG(void);

  /**
   *  @brief For MAX30001 ONLY
   *  @brief This function sets up the MAX30001 for pace signal detection.
   *  @brief If both PACE and BIOZ are turned ON, then make sure Fcgen is set for 80K or 40K in the
   *  @brief max30001_BIOZ_InitStart() function.  However, if Only PACE is on but BIOZ off, then Fcgen can be set
   *  @brief for 80K only, in the max30001_BIOZ_InitStart() function
   *  @brief Registers used: MNGR_INT-0x04, CNFG_GEN-0x37, CNFG_PACE-0x1A.
   *  @param En_pace : PACE Channel Enable <CNFG_GEN Register>
   *  @param Clr_pedge : PACE Edge Detect Interrupt (PEDGE) Clear Behavior <MNGR_INT Register>
   *  @param Pol: PACE Input Polarity Selection <CNFG_PACE Register>
   *  @param Gn_diff_off: PACE Differentiator Mode <CNFG_PACE Register>
   *  @param Gain: PACE Channel Gain Selection <CNFG_PACE Register>
   *  @param Aout_lbw:  PACE Analog Output Buffer Bandwidth Mode <CNFG_PACE Register>
   *  @param Aout: PACE Single Ended Analog Output Buffer Signal Monitoring Selection <CNFG_PACE Register>
   *  @param Dacp (4bits): PACE Detector Positive Comparator Threshold <CNFG_PACE Register>
   *  @param Dacn(4bits): PACE Detector Negative Comparator Threshold <CNFG_PACE Register>
   *  @returns 0-if no error.  A non-zero value indicates an error <CNFG_PACE Register>
   *
   */
  int PACE_InitStart(uint8_t En_pace, uint8_t Clr_pedge, uint8_t Pol,
                              uint8_t Gn_diff_off, uint8_t Gain,
                              uint8_t Aout_lbw, uint8_t Aout, uint8_t Dacp,
                              uint8_t Dacn);

  /**
   *@brief For MAX30001 ONLY
   *@param This function disables the PACE.  Uses Register CNFG_GEN-0x10.
   *@returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int Stop_PACE(void);

  /**
   * @brief For MAX30001/2 ONLY
   * @brief This function sets up the MAX30001 for BIOZ measurement.
   * @brief Registers used: MNGR_INT-0x04, CNFG_GEN-0X10, CNFG_BMUX-0x17,CNFG_BIOZ-0x18.
   * @param En_bioz: BIOZ Channel Enable <CNFG_GEN Register>
   * @param Openp: Open the BIP Input Switch <CNFG_BMUX Register>
   * @param Openn: Open the BIN Input Switch <CNFG_BMUX Register>
   * @param Calp_sel: BIP Calibration Selection <CNFG_BMUX Register>
   * @param Caln_sel: BIN Calibration Selection <CNFG_BMUX Register>
   * @param CG_mode:  BIOZ Current Generator Mode Selection <CNFG_BMUX Register>
   * @param B_fit:  BIOZ FIFO Interrupt Threshold (issues BINT based on number of unread FIFO records) <MNGR_INT Register>
   * @param Rate: BIOZ Data Rate <CNFG_BIOZ Register>
   * @param Ahpf: BIOZ/PACE Channel Analog High Pass Filter Cutoff Frequency and Bypass <CNFG_BIOZ Register>
   * @param Ext_rbias:  External Resistor Bias Enable <CNFG_BIOZ Register>
   * @param Gain: BIOZ Channel Gain Setting <CNFG_BIOZ Register>
   * @param Dhpf: BIOZ Channel Digital High Pass Filter Cutoff Frequency <CNFG_BIOZ Register>
   * @param Dlpf:  BIOZ Channel Digital Low Pass Filter Cutoff Frequency <CNFG_BIOZ Register>
   * @param Fcgen:  BIOZ Current Generator Modulation Frequency <CNFG_BIOZ Register>
   * @param Cgmon:  BIOZ Current Generator Monitor <CNFG_BIOZ Register>
   * @param Cgmag:  BIOZ Current Generator Magnitude <CNFG_BIOZ Register>
   * @param Phoff: BIOZ Current Generator Modulation Phase Offset <CNFG_BIOZ Register>
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int BIOZ_InitStart(uint8_t En_bioz, uint8_t Openp, uint8_t Openn,
                              uint8_t Calp_sel, uint8_t Caln_sel,
                              uint8_t CG_mode,
                              /* uint8_t En_bioz,*/ uint8_t B_fit, uint8_t Rate,
                              uint8_t Ahpf, uint8_t Ext_rbias, uint8_t Gain,
                              uint8_t Dhpf, uint8_t Dlpf, uint8_t Fcgen,
                              uint8_t Cgmon, uint8_t Cgmag, uint8_t Phoff);

  /**
   * @brief For MAX30001/2 ONLY
   * @brief This function disables the BIOZ.  Uses Register CNFG_GEN-0x10.
   * @returns 0-if no error.  A non-zero value indicates an error.
       * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int Stop_BIOZ(void);

  /**
   * @brief For MAX30001/2 ONLY
   * @brief BIOZ modulated Resistance Built-in-Self-Test, Registers used: CNFG_BMUX-0x17
   * @param En_bist: Enable Modulated Resistance Built-in-Self-test <CNFG_BMUX Register>
   * @param Rnom: BIOZ RMOD BIST Nominal Resistance Selection <CNFG_BMUX Register>
   * @param Rmod: BIOZ RMOD BIST Modulated Resistance Selection <CNFG_BMUX Register>
   * @param Fbist: BIOZ RMOD BIST Frequency Selection <CNFG_BMUX Register>
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int BIOZ_InitBist(uint8_t En_bist, uint8_t Rnom, uint8_t Rmod,
                             uint8_t Fbist);

  /**
   * @brief For MAX30001/3/4 ONLY
   * @brief Sets up the device for RtoR measurement
   * @param EN_rtor: ECG RTOR Detection Enable <RTOR1 Register>
   * @param Wndw: R to R Window Averaging (Window Width = RTOR_WNDW[3:0]*8mS) <RTOR1 Register>
   * @param Gain: R to R Gain (where Gain = 2^RTOR_GAIN[3:0], plus an auto-scale option) <RTOR1 Register>
   * @param Pavg: R to R Peak Averaging Weight Factor <RTOR1 Register>
   * @param Ptsf: R to R Peak Threshold Scaling Factor <RTOR1 Register>
   * @param Hoff: R to R minimum Hold Off <RTOR2 Register>
   * @param Ravg: R to R Interval Averaging Weight Factor <RTOR2 Register>
   * @param Rhsf: R to R Interval Hold Off Scaling Factor <RTOR2 Register>
   * @param Clr_rrint: RTOR Detect Interrupt Clear behaviour <MNGR_INT Register>
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int RtoR_InitStart(uint8_t En_rtor, uint8_t Wndw, uint8_t Gain,
                              uint8_t Pavg, uint8_t Ptsf, uint8_t Hoff,
                              uint8_t Ravg, uint8_t Rhsf, uint8_t Clr_rrint);

  /**
   * @brief For MAX30001/3/4 ONLY
   * @brief This function disables the RtoR.  Uses Register CNFG_RTOR1-0x1D
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int Stop_RtoR(void);

  /**
   * @brief This is a function that waits for the PLL to lock; once a lock is achieved it exits out. (For convenience only)
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int PLL_lock(void);

  /**
   * @brief This function causes the MAX30001 to reset.  Uses Register SW_RST-0x08
   * @return 0-if no error.  A non-zero value indicates an error.
   *
   */
  int sw_rst(void);

  /**
   * @brief This function provides a SYNCH operation.  Uses Register SYCNH-0x09. Please refer to the data sheet for
   * @brief the details on how to use this.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int synch(void);

  /**
   * @brief This function performs a FIFO Reset.  Uses Register FIFO_RST-0x0A. Please refer to the data sheet
   * @brief for the details on how to use this.
   * @returns 0-if no error.  A non-zero value indicates an error.
   */
  int fifo_rst(void);

  /**
   *
   * @brief This is a callback function which collects all the data from the ECG, BIOZ, PACE and RtoR. It also handles
   * @brief Lead On/Off.  This  function is passed through the argument of max30001_COMMinit().
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int int_handler(void);

  /**
   * @brief This is function called from the max30001_int_handler() function and processes all the ECG, BIOZ, PACE
   * @brief and the RtoR data and sticks them in appropriate arrays and variables each unsigned 32 bits.
   * @param ECG data will be in the array (input): max30001_ECG_FIFO_buffer[]
   * @param Pace data will be in the array (input): max30001_PACE[]
   * @param RtoRdata will be in the variable (input): max30001_RtoR_data
   * @param BIOZ data will be in the array (input): max30001_BIOZ_FIFO_buffer[]
   * @param global  max30001_ECG_FIFO_buffer[]
   * @param global  max30001_PACE[]
   * @param global  max30001_BIOZ_FIFO_buffer[]
   * @param global  max30001_RtoR_data
   * @param global  max30001_DCLeadOff
   * @param global  max30001_ACLeadOff
   * @param global  max30001_LeadON
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int FIFO_LeadONOff_Read(void);

  /**
   * @brief This function allows writing to a register.
   * @param addr:  Address of the register to write to
   * @param data:  24-bit data read from the register.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int reg_write(MAX30001_REG_map_t addr, uint32_t data);

  /**
   * @brief This function allows reading from a register
   * @param addr:   Address of the register to read from.
   * @param *return_data: pointer to the value read from the register.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int reg_read(MAX30001_REG_map_t addr, uint32_t *return_data);

  /**
   * @brief This function enables the DC Lead Off detection. Either ECG or BIOZ can be detected, one at a time.
   * @brief Registers Used:  CNFG_GEN-0x10
   * @param En_dcloff: BIOZ Digital Lead Off Detection Enable
   * @param Ipol: DC Lead Off Current Polarity (if current sources are enabled/connected)
   * @param Imag: DC Lead off current Magnitude Selection
   * @param Vth: DC Lead Off Voltage Threshold Selection
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int Enable_DcLeadOFF_Init(int8_t En_dcloff, int8_t Ipol, int8_t Imag,
                                     int8_t Vth);

  /**
   * @brief This function disables the DC Lead OFF feature, whichever is active.
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int Disable_DcLeadOFF(void);

  /**
   * @brief This function sets up the BIOZ for AC Lead Off test.
   * @brief Registers Used:  CNFG_GEN-0x10, MNGR_DYN-0x05
   * @param En_bloff: BIOZ Digital Lead Off Detection Enable <CNFG_GEN register>
   * @param Bloff_hi_it:      DC Lead Off Current Polarity (if current sources are enabled/connected) <MNGR_DYN register>
   * @param Bloff_lo_it:      DC Lead off current Magnitude Selection <MNGR_DYN register>
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int BIOZ_Enable_ACLeadOFF_Init(uint8_t En_bloff, uint8_t Bloff_hi_it,
                                          uint8_t Bloff_lo_it);

  /**
   * @brief This function Turns of the BIOZ AC Lead OFF feature
   * @brief Registers Used:  CNFG_GEN-0x10
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int BIOZ_Disable_ACleadOFF(void);

  /**
   * @brief This function enables the Current Gnerator Monitor
   * @brief Registers Used:  CNFG_BIOZ-0x18
   * @returns 0-if no error.  A non-zero value indicates an error.
   *
   */
  int BIOZ_Enable_BCGMON(void);

  /**
   *
   * @brief This function enables the Lead ON detection. Either ECG or BIOZ can be detected, one at a time.
   * @brief Also, the en_bioz, en_ecg, en_pace setting is saved so that when this feature is disabled through the
   * @brief max30001_Disable_LeadON() function (or otherwise) the enable/disable state of those features can be retrieved.
   * @param Channel: ECG or BIOZ detection
   * @returns 0-if everything is good.  A non-zero value indicates an error.
   *
   */
  int Enable_LeadON(int8_t Channel);

  /**
   * @brief This function turns off the Lead ON feature, whichever one is active.  Also, retrieves the en_bioz,
   * @brief en_ecg, en_pace and sets it back to as it was.
   * @param 0-if everything is good.  A non-zero value indicates an error.
   *
   */
  int Disable_LeadON(void);

  /**
   *
   * @brief This function is toggled every 2 seconds to switch between ECG Lead ON and BIOZ Lead ON detect
   * @brief Adjust LEADOFF_SERVICE_TIME to determine the duration between the toggles.
   * @param CurrentTime - This gets fed the time by RTC_GetValue function
   *
   */
  void ServiceLeadON(uint32_t currentTime);

  /**
   *
   * @brief This function is toggled every 2 seconds to switch between ECG DC Lead Off and BIOZ DC Lead Off
   * @brief Adjust LEADOFF_SERVICE_TIME to determine the duration between the toggles.
   * @param CurrentTime - This gets fed the time by RTC_GetValue function
   *
   */
  void ServiceLeadoff(uint32_t currentTime);

  /**
   *
   * @brief This function sets current RtoR values and fmstr values in a pointer structure
   * @param hspValMax30001 - Pointer to a structure where to store the values
   *
   */
  void ReadHeartrateData(max30001_bledata_t *_hspValMax30001);

  /**
   * @brief type definition for data interrupt
   */
  typedef void (*PtrFunction)(uint32_t id, uint32_t *buffer, uint32_t length);

  /**
   * @brief Used to connect a callback for when interrupt data is available
   */
  void onDataAvailable(PtrFunction _onDataAvailable);



  /**
   * @brief Preventive measure used to dismiss interrupts that fire too early during
   * @brief initialization on INTB line
   *
   */
  static void Mid_IntB_Handler(void);

  /**
   * @brief Preventive measure used to dismiss interrupts that fire too early during
   * @brief initialization on INT2B line
   *
   */
  static void Mid_Int2B_Handler(void);

  /**
   * @brief Allows Interrupts to be accepted as valid.
   * @param state: 1-Allow interrupts, Any-Don't allow interrupts.
   *
   */
  void AllowInterrupts(int state);


  /// @brief function pointer to the async callback
  static event_callback_t functionpointer;
  
  /// @brief flag used to indicate an async xfer has taken place
  static volatile int xferFlag;

  /**
   * @brief Callback handler for SPI async events
   * @param events description of event that occurred
   */
  static void spiHandler(int events);


  static MAX30001 *instance;

private:

  /**
   * @brief Used to notify an external function that interrupt data is available
   * @param id type of data available
   * @param buffer 32-bit buffer that points to the data
   * @param length length of 32-bit elements available
   */
  void dataAvailable(uint32_t id, uint32_t *buffer, uint32_t length);
  
  /**
   * @brief Transmit and recieve QUAD SPI data
   * @param tx_buf pointer to transmit byte buffer
   * @param tx_size number of bytes to transmit
   * @param rx_buf pointer to the recieve buffer
   * @param rx_size number of bytes to recieve
   */
  int SPI_Transmit(const uint8_t *tx_buf, uint32_t tx_size, uint8_t *rx_buf,
                   uint32_t rx_size);

  /// pointer to mbed SPI object
  SPI *spi;
  /// is this object the owner of the spi object
  bool spi_owner;
  /// buffer to use for async transfers
  uint8_t buffer[ASYNC_SPI_BUFFER_SIZE];
  /// function pointer to the async callback
 // event_callback_t functionpointer;
  /// callback function when interrupt data is available
  PtrFunction onDataAvailableCallback;

}; // End of MAX30001 Class


#endif /* MAX30001_H_ */
