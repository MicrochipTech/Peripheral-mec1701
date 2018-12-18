/**************************************************************************//**
 * @file     MEC17xx.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device MEC17xx
 * @version  V5.00
 * @date     10. January 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 * Copyright (c) 2018 Microchip Technology Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MCHP_MEC1701_H
#define MCHP_MEC1701_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MCHP
  * @{
  */


/** @addtogroup MEC17xx
  * @{
  */


/** @addtogroup Configuration_of_CMSIS
  * @{
  */



/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */

typedef enum IRQn
{
/* =======================================  ARM Cortex-M# Specific Interrupt Numbers  ======================================== */

  Reset_IRQn                = -15,              /*!< -15  Reset Vector, invoked on Power up and warm reset                     */
  NonMaskableInt_IRQn       = -14,              /*!< -14  Non maskable Interrupt, cannot be stopped or preempted               */
  HardFault_IRQn            = -13,              /*!< -13  Hard Fault, all classes of Fault                                     */
  MemoryManagement_IRQn     = -12,              /*!< -12  Memory Management, MPU mismatch, including Access Violation
                                                          and No Match                                                         */
  BusFault_IRQn             = -11,              /*!< -11  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                          related Fault                                                        */
  UsageFault_IRQn           = -10,              /*!< -10  Usage Fault, i.e. Undef Instruction, Illegal State Transition        */
  SVCall_IRQn               =  -5,              /*!< -5 System Service Call via SVC instruction                                */
  DebugMonitor_IRQn         =  -4,              /*!< -4 Debug Monitor                                                          */
  PendSV_IRQn               =  -2,              /*!< -2 Pendable request for system service                                    */
  SysTick_IRQn              =  -1,              /*!< -1 System Tick Timer                                                      */

/* ===========================================  MEC17xx Specific Interrupt Numbers  ========================================= */

    GIRQ08_IRQn                 = 0,  /*!< GPIO 0140 - 0176 */
    GIRQ09_IRQn                 = 1,  /*!< GPIO 0100 - 0136 */
    GIRQ10_IRQn                 = 2,  /*!< GPIO 0040 - 0076 */
    GIRQ11_IRQn                 = 3,  /*!< GPIO 0000 - 0036 */
    GIRQ12_IRQn                 = 4,  /*!< GPIO 0200 - 0236 */
    GIRQ13_IRQn                 = 5,  /*!< SMBus Aggregated */
    GIRQ14_IRQn                 = 6,  /*!< DMA Aggregated */
    GIRQ15_IRQn                 = 7,
    GIRQ16_IRQn                 = 8,
    GIRQ17_IRQn                 = 9,
    GIRQ18_IRQn                 = 10,
    GIRQ19_IRQn                 = 11,
    GIRQ20_IRQn                 = 12,
    GIRQ21_IRQn                 = 13,
    GIRQ23_IRQn                 = 14,
    GIRQ24_IRQn                 = 15,
    GIRQ25_IRQn                 = 16,
    GIRQ26_IRQn                 = 17,  /*!< GPIO 0240 - 0276 */
    /* Reserved gap, 18-19 */
    /* GIRQ's 8 - 12, 22, 24 - 26 no direct */
    /* GIRQ13 */
    I2C0_IRQn                   = 20,
    I2C1_IRQn                   = 21,
    I2C2_IRQn                   = 22,
    I2C3_IRQn                   = 23,
    /* GIRQ14 */
    DMA0_IRQn                   = 24,
    DMA1_IRQn                   = 25,
    DMA2_IRQn                   = 26,
    DMA3_IRQn                   = 27,
    DMA4_IRQn                   = 28,
    DMA5_IRQn                   = 29,
    DMA6_IRQn                   = 30,
    DMA7_IRQn                   = 31,
    DMA8_IRQn                   = 32,
    DMA9_IRQn                   = 33,
    DMA10_IRQn                  = 34,
    DMA11_IRQn                  = 35,
    DMA12_IRQn                  = 36,
    DMA13_IRQn                  = 37,
    /* Reserved gap, 38-39 */
    /* GIRQ15 */
    UART0_IRQn                  = 40,
    UART1_IRQn                  = 41,
    EMI0_IRQn                   = 42,
    EMI1_IRQn                   = 43,
    EMI2_IRQn                   = 44,
    ACPI_EC0_IBF_IRQn           = 45,
    ACPI_EC0_OBE_IRQn           = 46,
    ACPI_EC1_IBF_IRQn           = 47,
    ACPI_EC1_OBE_IRQn           = 48,
    ACPI_EC2_IBF_IRQn           = 49,
    ACPI_EC2_OBE_IRQn           = 50,
    ACPI_EC3_IBF_IRQn           = 51,
    ACPI_EC3_OBE_IRQn           = 52,
    ACPI_EC4_IBF_IRQn           = 53,
    ACPI_EC4_OBE_IRQn           = 54,
    PM1_CTL_IRQn                = 55,
    PM1_EN_IRQn                 = 56,
    PM1_STS_IRQn                = 57,
    MIF8042_OBE_IRQn            = 58,
    MIF8042_IBF_IRQn            = 59,
    MBOX_IRQn                   = 60,
    /* reserved gag 61 */
    P80A_IRQn                   = 62,
    P80B_IRQn                   = 63,
    LenAsic_IRQn                = 64,
    /* GIRQ16 */
    PKE_ERR_IRQn                = 65,
    PKE_END_IRQn                = 66,
    RNG_IRQn                    = 67,
    AES_IRQn                    = 68,
    HASH_IRQn                   = 69,
    /* GIRQ17 */
    PECI_IRQn                   = 70,
    TACH0_IRQn                  = 71,
    TACH1_IRQn                  = 72,
    TACH2_IRQn                  = 73,
    R2P0_FAIL_IRQn              = 74,
    R2P0_STALL_IRQn             = 75,
    R2P1_FAIL_IRQn              = 76,
    R2P1_STALL_IRQn             = 77,
    ADC_SNGL_IRQn               = 78,
    ADC_RPT_IRQn                = 79,
    RCID0_IRQn                  = 80,
    RCID1_IRQn                  = 81,
    RCID2_IRQn                  = 82,
    LED0_IRQn                   = 83,
    LED1_IRQn                   = 84,
    LED2_IRQn                   = 85,
    LED3_IRQn                   = 86,
    PHOT_IRQn                   = 87,
    PWRGD0_IRQn                 = 88,
    PWRGD1_IRQn                 = 89,
    /* GIRQ18 */
    LPCBERR_IRQn                = 90,
    QMSPI_IRQn                  = 91,
    GPSPI0_TX_IRQn              = 92,
    GPSPI0_RX_IRQn              = 93,
    GPSPI1_TX_IRQn              = 94,
    GPSPI1_RX_IRQn              = 95,
    BC0_BUSY_IRQn               = 96,
    BC0_ERR_IRQn                = 97,
    BC1_BUSY_IRQn               = 98,
    BC1_ERR_IRQn                = 99,
    PS2_0_IRQn                  = 100,
    PS2_1_IRQn                  = 101,
    PS2_2_IRQn                  = 102,
    /* GIRQ19 */
    ESPI_PC_IRQn                = 103,
    ESPI_BM1_IRQn               = 104,
    ESPI_BM2_IRQn               = 105,
    ESPI_LTR_IRQn               = 106,
    ESPI_OOB_UP_IRQn            = 107,
    ESPI_OOB_DN_IRQn            = 108,
    ESPI_FLASH_IRQn             = 109,
    ESPI_RESET_IRQn             = 110,
    /* GIRQ20 no direct */
    /* GIRQ21 */
    RTMR_IRQn                   = 111,
    HTMR0_IRQn                  = 112,
    HTMR1_IRQn                  = 113,
    WK_IRQn                     = 114,
    WKSUB_IRQn                  = 115,
    WKSEC_IRQn                  = 116,
    WKSUBSEC_IRQn               = 117,
    SYSPWR_IRQn                 = 118,
    RTC_IRQn                    = 119,
    RTC_ALARM_IRQn              = 120,
    VCI_OVRD_IN_IRQn            = 121,
    VCI_IN0_IRQn                = 122,
    VCI_IN1_IRQn                = 123,
    VCI_IN2_IRQn                = 124,
    VCI_IN3_IRQn                = 125,
    VCI_IN4_IRQn                = 126,
    VCI_IN5_IRQn                = 127,
    VCI_IN6_IRQn                = 128,
    PS20A_WAKE_IRQn             = 129,
    PS20B_WAKE_IRQn             = 130,
    PS21A_WAKE_IRQn             = 131,
    PS21B_WAKE_IRQn             = 132,
    PS21_WAKE_IRQn              = 133,
    ENVMON_IRQn                 = 134,
    KEYSCAN_IRQn                = 135,
    /* GIRQ22 wake only no EC connection */
    /* GIRQ23 */
    BTMR0_IRQn                  = 136,
    BTMR1_IRQn                  = 137,
    BTMR2_IRQn                  = 138,
    BTMR3_IRQn                  = 139,
    BTMR4_IRQn                  = 140,
    BTMR5_IRQn                  = 141,
    EVTMR0_IRQn                 = 142,
    EVTMR1_IRQn                 = 143,
    EVTMR2_IRQn                 = 144,
    EVTMR3_IRQn                 = 145,
    CAPTMR_IRQn                 = 146,
    CAP0_IRQn                   = 147,
    CAP1_IRQn                   = 148,
    CAP2_IRQn                   = 149,
    CAP3_IRQn                   = 150,
    CAP4_IRQn                   = 151,
    CAP5_IRQn                   = 152,
    CMP0_IRQn                   = 153,
    CMP1_IRQn                   = 154,
    PROMSPI_IRQn                = 155,
    ESPI_VWIRE_IRQn             = 156,
    MAX_IRQn
} IRQn_Type;



/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/* ===========================  Configuration of the Arm Cortex-M4 Processor and Core Peripherals  =========================== */

#define __CM4_REV                 0x0201    /*!< Core Revision r2p1 */

#define __MPU_PRESENT             1         /*!< Set to 1 if MPU is present */
#define __VTOR_PRESENT            1         /*!< Set to 1 if VTOR is present */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1         /*!< Set to 1 if FPU is present */
#define __FPU_DP                  0         /*!< Set to 1 if FPU is double precision FPU (default is single precision FPU) */
#define __ICACHE_PRESENT          0         /*!< Set to 1 if I-Cache is present */
#define __DCACHE_PRESENT          0         /*!< Set to 1 if D-Cache is present */
#define __DTCM_PRESENT            0         /*!< Set to 1 if DTCM is present */


/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm4.h"                           /*!< Arm Cortex-M4 processor and core peripherals */
#include "system_mchp_mec1701.h"                /*!< MEC1701 System */


/* ========================================  Start of section using anonymous unions  ======================================== */
#if   defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

#define MASK(r, v, m) ((r) & ~(m)) | ((v) & (m))
#define BITSET(r, b) (r) |= (1ul << (b))
#define BITCLR(r, b) (r) &= ~(1ul << (b))
#define BITXOR(r, b) (r) ^= (1ul << (b))

/* ======================================================================== */
/* ================   Device Specific Peripheral Section   ================ */
/* ======================================================================== */


/** @addtogroup MEC17xx_Peripheral_peripherals
  * @{
  */

/* =========================================================================*/
/* ================   WDT                                  ================ */
/* =========================================================================*/

/**
  * @brief Watch Dog Timer (WDT)
  */
typedef struct
{                           /*!< (@ 0x40000000) WDT Structure               */
  __IOM uint16_t LOAD;      /*!< (@ 0x00000000) WDT Load                    */
        uint16_t RSVDA;
  __IOM uint8_t  CONTROL;   /*!< (@ 0x00000004) WDT Control                 */
        uint8_t  RSVDB[3];
  __IOM uint8_t  KICK;      /*!< (@ 0x00000008) WDT Kick                    */
        uint8_t  RSVDC[3];
  __IOM uint16_t COUNT;     /*!< (@ 0x0000000C) WDT Count                   */
        uint16_t RSVDD;
} WDT_TypeDef;

/* =========================================================================*/
/* ================   32 and 16-bit Basic Timers           ================ */
/* =========================================================================*/

#define BTMR_BLOCK_SPACING          0x20
#define BTMR_BLOCK_SPACING_PWROF2   5

/**
  * @brief 16-bit Basic Timer (B16TMR)
  * @note COUNT and PRELOAD are 32-bit or 16-bit. Bits[31:16] are
  * read-only 0 for 16-bit versions of the timer.
  */
typedef struct
{                           /*!< (@ 0x40000C00) B16TMR Structure            */
  __IOM uint32_t COUNT;     /*!< (@ 0x00000000) B16TMR Count                */
  __IOM uint32_t PRELOAD;   /*!< (@ 0x00000004) B16TMR Count                */
  __IOM uint8_t  STATUS;    /*!< (@ 0x00000008) B16TMR Count                */
        uint8_t  RSVDC[3];
  __IOM uint8_t  IEN;       /*!< (@ 0x0000000C) B16TMR Count                */
        uint8_t  RSVDD[3];
        uint32_t CONTROL;   /*!< (@ 0x00000010) B16TMR Count                */
} BTMR_TypeDef;

/* =========================================================================*/
/* ================   Event Timer/Counter                  ================ */
/* =========================================================================*/

/**
  * @brief Event Timer/Counter (EVT)
  */
typedef struct
{                               /*!< (@ 0x40000D00) EVT Structure            */
  __IOM uint16_t CONTROL;       /*!< (@ 0x00000000) EVT Control              */
        uint16_t RSVDA;
  __IOM uint16_t CLK_EV_CTRL;   /*!< (@ 0x00000004) EVT Clock/Event Control  */
        uint16_t RSVDB;
  __IOM uint16_t RELOAD;        /*!< (@ 0x00000008) EVT Control              */
        uint16_t RSVDC;
  __IOM uint16_t COUNT;         /*!< (@ 0x00000008) EVT Count                */
        uint16_t RSVDD;
} EVT_TypeDef;

/* =========================================================================*/
/* ================   Capture/Compare Timer                ================ */
/* =========================================================================*/

/**
  * @brief Capture/Compare Timer (CCT)
  */
typedef struct
{                                   /*!< (@ 0x40001000) CCT Structure         */
    __IOM uint32_t CONTROL;         /*!< (@ 0x00000000) CCT Control           */
    __IOM uint32_t CAP0_CTRL;       /*!< (@ 0x00000004) CCT Capture 0 Control */
    __IOM uint32_t CAP1_CTRL;       /*!< (@ 0x00000008) CCT Capture 1 Control */
    __IOM uint32_t FREE_RUN;        /*!< (@ 0x0000000C) CCT Free run counter  */
    __IOM uint32_t CAPTURE0;        /*!< (@ 0x00000010) CCT Capture 0         */
    __IOM uint32_t CAPTURE1;        /*!< (@ 0x00000014) CCT Capture 1         */
    __IOM uint32_t CAPTURE2;        /*!< (@ 0x00000018) CCT Capture 2         */
    __IOM uint32_t CAPTURE3;        /*!< (@ 0x0000001C) CCT Capture 3         */
    __IOM uint32_t CAPTURE4;        /*!< (@ 0x00000020) CCT Capture 4         */
    __IOM uint32_t CAPTURE5;        /*!< (@ 0x00000024) CCT Capture 5         */
    __IOM uint32_t COMPARE0;        /*!< (@ 0x00000028) CCT Compare 0         */
    __IOM uint32_t COMPARE1;        /*!< (@ 0x0000002C) CCT Compare 1         */
} CCT_TypeDef;

/* =========================================================================*/
/* ================            RCID                        ================ */
/* =========================================================================*/

/**
  * @brief RC Identification (RCID)
  */
typedef struct
{                           /*!< (@ 0x40001400) RCID Structure        */
  __IOM uint16_t CONTROL;   /*!< (@ 0x00000000) RCID Control          */
        uint16_t RSVDA;
  __IOM uint16_t RCID_DATA; /*!< (@ 0x00000004) RCID Data             */
        uint16_t RSVDB;
} RCID_TypeDef;

/* =========================================================================*/
/* ================            DMA                         ================ */
/* =========================================================================*/

#define DMA_MAX_CHAN                14
#define DMA_CHAN_SPACING            0x40
#define DMA_CHAN_SPACING_PWROF2     6

/**
  * @brief DMA Channel
  */
typedef struct
{
    __IOM uint8_t  ACTIVATE;        /*!< (@ 0x00000040) DMA channel activate  */
          uint8_t  RSVD3[3];
    __IOM uint32_t MEM_ADDR_START;  /*!< (@ 0x00000044) DMA channel memory start address  */
    __IOM uint32_t MEM_ADDR_END;    /*!< (@ 0x00000048) DMA channel memory end address  */
    __IOM uint32_t DEV_ADDR;        /*!< (@ 0x0000004C) DMA channel device address  */
    __IOM uint32_t CONTROL;         /*!< (@ 0x00000050) DMA channel control  */
    __IOM uint8_t  ISTATUS;         /*!< (@ 0x00000054) DMA channel interrupt status  */
          uint8_t  RSVD4[3];
    __IOM uint8_t  IEN;             /*!< (@ 0x00000058) DMA channel interrupt enable  */
          uint8_t  RSVD5[3];
    __IOM uint32_t FSM;             /*!< (@ 0x0000005C) DMA channel FSM (RO)  */
    __IOM uint8_t  ALU_EN;          /*!< (@ 0x00000060) DMA channels [0-1] ALU Enable */
          uint8_t  RSVD6[3];
    __IOM uint32_t ALU_DATA;        /*!< (@ 0x00000064) DMA channels [0-1] ALU Data */
    __IOM uint8_t  ALU_POST_STS;    /*!< (@ 0x00000068) DMA channels [0-1] ALU post status (RO) */
          uint8_t  RSVD7[3];
    __IOM uint32_t ALU_FSM;         /*!< (@ 0x0000006C) DMA channels [0-1] ALU FSM (RO) */
          uint32_t RSVD8[4];
} DMA_CHAN_TypeDef;

/**
  * @brief DMA Main (DMA_MAIN)
  */
typedef struct
{                           /*!< (@ 0x40002400) DMA Structure        */
    __IOM uint8_t  ENABLE;    /*!< (@ 0x00000000) DMA block enable     */
          uint8_t  RSVD1[3];
    __IOM uint32_t DATA_PKT;  /*!< (@ 0x00000004) DMA data packet (RO) */
    __IOM uint32_t ARB_FSM;   /*!< (@ 0x00000008) DMA Arbiter FSM (RO) */
          uint32_t RSVD2[13];
    DMA_CHAN_TypeDef CHAN[DMA_MAX_CHAN]; /*!< (@ 0x00000040) DMA Channels */
} DMA_TypeDef;

/* =========================================================================*/
/* ================            PROCHOT                     ================ */
/* =========================================================================*/

/**
  * @brief PROCHOT Monitor (PROCHOT)
  */
typedef struct
{                             /*!< (@ 0x40003400) PROCHOT Structure        */
    __IOM uint32_t CUMUL_CNT;       /*!< (@ 0x00000000) PROCHOT Cumulative Count */
    __IOM uint32_t DUTY_CYCLEC_CNT; /*!< (@ 0x00000004) PROCHOT Duty Cycle Count (RO) */
    __IOM uint32_t DUTY_CYCLE_PER;  /*!< (@ 0x00000008) PROCHOT Duty Cycle Period */
    __IOM uint16_t CTRL_STS;        /*!< (@ 0x0000000C) PROCHOT Control/status */
         uint16_t  RSVDA;
    __IOM uint16_t ASSERTION_CNT;   /*!< (@ 0x00000010) PROCHOT Assertion counter (RO) */
          uint16_t RSVDB;
    __IOM uint16_t ASSERTION_LIMIT; /*!< (@ 0x00000014) PROCHOT Assertion counter limit */
          uint16_t RSVDC;
} PROCHOT_TypeDef;

/* =========================================================================*/
/* ================            I2CSMB                      ================ */
/* =========================================================================*/
#define I2CSMB_MAX_INSTANCES       (4)
#define I2CSMB_INSTANCE_SPACING    (0x400)
#define I2CSMB_INSTANCE_SPACING_P2 (10)

/**
  * @brief I2CSMB Block (I2CSMB)
  */
typedef struct
{                               /*!< (@ 0x40004000) I2CSMB Structure        */
    __IOM uint8_t   CTRLW_STSR;     /*!< (@ 0x00000000) I2CSMB Control(WO), Status(RO) */
          uint8_t   RSVD1[3];
    union {                         /*!< (@ 0x00000004) I2CSMB Own address 0 */
    __IOM uint32_t  OWN_ADDR32;
    __IOM uint16_t  OWN_ADDR16[2];
    __IOM uint8_t   OWN_ADDR8[4];
    };
    __IOM uint8_t   I2C_DATA;       /*!< (@ 0x00000008) I2CSMB I2C Data */
          uint8_t   RSVD2[3];
    union {
        __IOM uint32_t MCMD32;       /*!< (@ 0x0000000C) I2CSMB SMB master command */
        __IOM uint16_t MCMD16[2];
        __IOM uint8_t  MCMD8[4];
    };
    union {
        __IOM uint32_t  SCMD32;      /*!< (@ 0x00000010) I2CSMB SMB slave command */
        __IOM uint16_t  SCMD16[2];
        __IOM uint16_t  SCMD8[4];
    };
    __IOM uint8_t   PEC;        /*!< (@ 0x00000014) I2CSMB PEC value */
          uint8_t   RSVD3[3];
    __IOM uint8_t   RPT_START_HOLD_TM;  /*!< (@ 0x00000018) I2CSMB Repeated-Start hold time */
          uint8_t   RSVD4[3];
          uint32_t  RSVD5[1];
    __IOM uint32_t  COMPLETION; /*!< (@ 0x00000020) I2CSMB Completion */
    __IOM uint32_t  IDLE_SCALING; /*!< (@ 0x00000024) I2CSMB Idle scaling */
    __IOM uint32_t  CONFIG;     /*!< (@ 0x00000028) I2CSMB Configuration */
    __IOM uint32_t  BUS_CLOCK;  /*!< (@ 0x0000002C) I2CSMB Bus Clock */
    __IOM uint8_t   BLOCK_ID;   /*!< (@ 0x00000030) I2CSMB Block ID */
          uint8_t   RSVD6[3];
    __IOM uint8_t   BLOCK_REV;  /*!< (@ 0x00000034) I2CSMB Block revision */
          uint8_t   RSVD7[3];
    __IOM uint8_t   BB_CTRL;    /*!< (@ 0x00000038) I2CSMB Bit-Bang control */
          uint8_t   RSVD8[3];
    __IOM uint32_t  MCHP_RSVD1; /*!< (@ 0x0000003C) I2CSMB MCHP Reserved 1 */
    __IOM uint32_t  DATA_TIMING;    /*!< (@ 0x00000040) I2CSMB Data timing */
    __IOM uint32_t  TMOUT_SCALING;  /*!< (@ 0x00000044) I2CSMB Time-out scaling */
    __IOM uint8_t   SLV_TX_BUF;     /*!< (@ 0x00000048) I2CSMB SMB slave TX buffer */
          uint8_t   RSVD9[3];
    __IOM uint8_t   SLV_RX_BUF;     /*!< (@ 0x0000004C) I2CSMB SMB slave RX buffer */
          uint8_t   RSVD10[3];
    __IOM uint8_t   MTR_TX_BUF; /*!< (@ 0x00000050) I2CSMB SMB Master TX buffer */
          uint8_t   RSVD11[3];
    __IOM uint8_t   MTR_RX_BUF; /*!< (@ 0x00000054) I2CSMB SMB Master RX buffer */
          uint8_t   RSVD12[3];
    __IOM uint32_t  FSM;        /*!< (@ 0x00000058) I2CSMB FSM (RO) */
    __IOM uint32_t  FSM_SMB;    /*!< (@ 0x0000005C) I2CSMB FSM SMB (RO) */
    __IOM uint8_t   WAKE_STS;   /*!< (@ 0x00000060) I2CSMB Wake status */
          uint8_t   RSVD13[3];
    __IOM uint8_t   WAKE_EN;    /*!< (@ 0x00000064) I2CSMB Wake enable */
          uint8_t   RSVD14[3];
    __IOM uint32_t  MCHP_RSVD2; /*!< (@ 0x00000068) I2CSMB MCHP Reserved 2 */
} I2CSMB_TypeDef;

/* =========================================================================*/
/* ================            QMSPI                       ================ */
/* =========================================================================*/
#define QMSPI_MAX_DESCRIPTOR     (5)

/**
  * @brief Quad Master SPI (QMSPI)
  */
typedef struct
{                       /*!< (@ 0x40005400) QMSPI Structure        */
    __IOM uint8_t  ACTSRST;     /*!< (@ 0x00000000) QMSPI Activate Soft-Reset */
    __IOM uint8_t  PHASE_CTRL;  /*!< (@ 0x00000001) QMSPI signalling phase control */
    __IOM uint16_t CLK_DIV;     /*!< (@ 0x00000002) QMSPI clock divider */
    __IOM uint32_t CTRL;        /*!< (@ 0x00000004) QMSPI Control */
    __IOM uint32_t EXE;         /*!< (@ 0x00000008) QMSPI Execute */
    __IOM uint32_t IFCTRL;      /*!< (@ 0x0000000C) QMSPI Interface control */
    __IOM uint32_t STATUS;      /*!< (@ 0x00000010) QMSPI Status */
    __IOM uint16_t TX_BUF_CNT;  /*!< (@ 0x00000014) QMSPI TX Buffer Count (RO) */
    __IOM uint16_t RX_BUF_CNT;  /*!< (@ 0x00000016) QMSPI RX Buffer Count (RO) */
    __IOM uint32_t IEN;         /*!< (@ 0x00000018) QMSPI Interrupt Enable */
    __IOM uint32_t TX_BUF_CNT_TRIG; /*!< (@ 0x0000001C) QMSPI TX Buffer Count Trigger */
    __IOM uint32_t RX_BUF_CNT_TRIG; /*!< (@ 0x0000001E) QMSPI RX Buffer Count Trigger */
    __IOM uint32_t TX_FIFO;     /*!< (@ 0x00000020) QMSPI TX FIFO */
    __IOM uint32_t RX_FIFO;     /*!< (@ 0x00000024) QMSPI RX FIFO */
          uint32_t RSVDA[2];
    __IOM uint32_t DESCR[QMSPI_MAX_DESCRIPTOR]; /*!< (@ 0x00000030) QMSPI Descriptors 0-4 */
} QMSPI_TypeDef;

/* =========================================================================*/
/* ================            PWM                         ================ */
/* =========================================================================*/

#define PWM_BLOCK_SPACING     0x10
#define PWM_SPACING_PWROF2    4

/**
  * @brief Pulse Width Modulator (PWM)
  */
typedef struct
{                               /*!< (@ 0x40005800) PWM Structure        */
    __IOM uint16_t COUNT_ON;    /*!< (@ 0x00000000) PWM Counter On count */
          uint16_t RSVDA;
    __IOM uint16_t COUNT_OFF;   /*!< (@ 0x00000004) PWM Counter Off count */
          uint16_t RSVDB;
    __IOM uint8_t  CONFIG;      /*!< (@ 0x00000008) PWM Control */
          uint8_t  RSVDC[3];
          uint32_t RSVDD[1];
} PWM_TypeDef;

/* =========================================================================*/
/* ================            TACH                        ================ */
/* =========================================================================*/

#define TACH_BLOCK_SPACING    0x10
#define TACH_SPACING_PWROF2   4

/**
  * @brief Tachometer (TACH)
  */
typedef struct
{                               /*!< (@ 0x40006000) TACH Structure   */
    __IOM uint16_t CONTROL;     /*!< (@ 0x00000000) TACH Control */
    __IOM uint16_t COUNT;       /*!< (@ 0x00000002) TACH Counter */
    __IOM uint8_t  STATUS;      /*!< (@ 0x00000004) TACH Status */
          uint8_t  RSVDA[3];
    __IOM uint16_t LIMIT_HI;    /*!< (@ 0x00000008) TACH Hi Limit */
          uint16_t RSVDB;
    __IOM uint16_t LIMIT_LO;    /*!< (@ 0x0000000C) TACH Lo Limit */
          uint16_t RSVDC;
} TACH_TypeDef;

/* =========================================================================*/
/* ================            PECI                        ================ */
/* =========================================================================*/

/**
  * @brief Platform Environment Control Interface (PECI)
  */
typedef struct
{                               /*!< (@ 0x40006400) PECI Structure   */
  __IOM uint8_t  WDATA;
        uint8_t  RSVDA[3];
  __IOM uint8_t  RDATA;
        uint8_t  RSVDB[3];
  __IOM uint8_t  CTRL;
        uint8_t  RSVDC[1];
  __IOM uint16_t REQTMR;
  __IOM uint8_t  STATUS1;
        uint8_t  RSVDD[3];
  __IOM uint8_t  STATUS2;
        uint8_t  RSVDE[3];
  __IOM uint8_t  ERROR;
        uint8_t  RSVDF[3];
  __IOM uint8_t  IEN1;
        uint8_t  RSVDG[3];
  __IOM uint8_t  IEN2;
        uint8_t  RSVDH[3];
  __IOM uint8_t  OPTBTLO;
        uint8_t  RSVDI[3];
  __IOM uint8_t  OPTBTHI;
        uint8_t  RSVDJ[3];
} PECI_TypeDef;

/* =========================================================================*/
/* ================            RTMR                        ================ */
/* =========================================================================*/

/**
  * @brief RTOS Timer (RTMR)
  */
typedef struct
{                           /*!< (@ 0x40007400) RTMR Structure   */
    __IOM uint32_t COUNT;   /*!< (@ 0x00000000) RTMR Counter */
    __IOM uint32_t PRELOAD; /*!< (@ 0x00000004) RTMR Preload */
    __IOM uint8_t  CONTROL; /*!< (@ 0x00000008) RTMR Control */
          uint8_t  RSVDA[3];
    __IOM uint32_t SOFT_IRQ; /*!< (@ 0x0000000C) RTMR Soft IRQ */
} RTMR_TypeDef;

/* =========================================================================*/
/* ================            ADC                         ================ */
/* =========================================================================*/

/**
  * @brief Analog to Digital Converter (ADC)
  */
#define ADC_MAX_CHAN    (16u)

typedef struct
{                               /*!< (@ 0x40007C00) ADC Structure   */
    __IOM uint8_t  CONTROL;     /*!< (@ 0x00000000) ADC Control */
          uint8_t  RSVDA[3];
    __IOM uint16_t START_DELAY;     /*!< (@ 0x00000004) ADC Start Delay */
    __IOM uint16_t REPEAT_DELAY;    /*!< (@ 0x00000006) ADC Repeat Delay */
    __IOM uint32_t CHAN_STATUS;     /*!< (@ 0x00000008) ADC Channel status */
    __IOM uint32_t SINGLE_EN;       /*!< (@ 0x0000000C) ADC Single Enable */
    __IOM uint32_t REPEAT_EN;       /*!< (@ 0x00000010) ADC Repeat Enable */
    __IOM uint32_t READING[ADC_MAX_CHAN];   /*!< (@ 0x00000014) ADC channel reading registers */
          uint32_t RSVDB[(0x7C-0x54)/4];
    __IOM uint32_t CONFIG;          /*!< (@ 0x0000007C) ADC Configuration */
} ADC_TypeDef;

/* =========================================================================*/
/* ================            TFDP                        ================ */
/* =========================================================================*/

/**
  * @brief Trace FIFO Debug Port (TFDP)
  */
typedef struct
{                           /*!< (@ 0x40008C00) TFDP Structure   */
    __IOM uint8_t MS_DATA;      /*!< (@ 0x00000000) TFDP Data */
          uint8_t RSVDA[3];
    __IOM uint8_t CTRL;         /*!< (@ 0x00000004) TFDP Control */
          uint8_t RSVDB[3];
} TFDP_TypeDef;

/* =========================================================================*/
/* ================            PS2                         ================ */
/* =========================================================================*/

/**
  * @brief PS/2 Keyboard/Mouse (PS2)
  */
typedef struct
{                           /*!< (@ 0x40009000) PS2 Structure   */
    __IOM uint8_t RX_TX_DATA;   /*!< (@ 0x00000000) PS2 Transmit Data */
          uint8_t RSVDA[3];
    __IOM uint8_t CONTROL;      /*!< (@ 0x00000004) PS2 Control */
          uint8_t RSVDB[3];
    __IOM uint8_t STATUS;       /*!< (@ 0x00000008) PS2 Status */
          uint8_t RSVDC[3];
} PS2_TypeDef;

/* =========================================================================*/
/* ================            GPSPI                       ================ */
/* =========================================================================*/

/**
  * @brief General Purpose SPI Master (GPSPI)
  */
typedef struct
{                           /*!< (@ 0x40009400) GPSPI Structure   */
    __IOM uint8_t ENABLE;   /*!< (@ 0x00000000) GPSPI Enable */
          uint8_t RSVDA[3];
    __IOM uint8_t CONTROL;  /*!< (@ 0x00000004) GPSPI Control */
          uint8_t RSVDB[3];
    __IOM uint8_t STATUS;   /*!< (@ 0x00000008) GPSPI Status */
          uint8_t RSVDC[3];
    __OM  uint8_t TX_DATA;  /*!< (@ 0x0000000C) TX Data */
          uint8_t RSVDD[3];
    __IM  uint8_t RX_DATA;  /*!< (@ 0x00000010) RX Data */
          uint8_t RSVDE[3];
    __IOM uint8_t CLOCK_CONTROL;    /*!< (@ 0x00000014) GPSPI Clock Control */
          uint8_t RSVDF[3];
    __IOM uint8_t CLOCK_GEN;    /*!< (@ 0x00000018) GPSPI Clock generator */
          uint8_t RSVDG[3];
} GPSPI_TypeDef;

/* =========================================================================*/
/* ================            HTMR                        ================ */
/* =========================================================================*/

/**
  * @brief Hibernation Timer (HTMR)
  */
typedef struct
{                           /*!< (@ 0x40009800) HTMR Structure   */
    __IOM uint16_t PRELOAD; /*!< (@ 0x00000000) HTMR Preload */
          uint16_t RSVDA;
    __IOM uint16_t CONTROL; /*!< (@ 0x00000004) HTMR Control */
          uint16_t RSVDB;
    __IOM uint16_t COUNT;   /*!< (@ 0x00000008) HTMR Count (RO) */
          uint16_t RSVDC;
} HTMR_TypeDef;

/* =========================================================================*/
/* ================            KSCAN                       ================ */
/* =========================================================================*/

/**
  * @brief Keyboard Scan (KSCAN)
  */
typedef struct
{                       /*!< (@ 0x40009C00) KSCAN Structure   */
          uint32_t RSVDA;
    __IOM uint8_t  KSO_SEL;     /*!< (@ 0x00000004) KSCAN KSO Select */
          uint8_t  RSVDB[3];
    __IOM uint8_t  KSI;         /*!< (@ 0x00000008) KSCAN KSI pin states */
          uint8_t  RSVDC[3];
    __IOM uint8_t  KSI_STS;     /*!< (@ 0x0000000C) KSCAN KSI status */
          uint8_t  RSVDD[3];
    __IOM uint8_t  KSI_IEN;     /*!< (@ 0x00000010) KSCAN KSI interrupt enable */
          uint8_t  RSVDE[3];
    __IOM uint8_t  EXT_CTRL;    /*!< (@ 0x00000014) KSCAN extended control */
          uint8_t  RSVDF[3];
} KSCAN_TypeDef;

/* =========================================================================*/
/* ================            RPM2PWM                     ================ */
/* =========================================================================*/

/**
  * @brief RPM to PWM fan control (RPM2PWM)
  */

typedef struct
{               /*!< (@ 0x4000A000) RPM2PWM Structure   */
    __IOM uint16_t FAN_SETTING;     /*! (@ 0x00000000) RPM2PWM Fan setting */
    __IOM uint16_t FAN_CONFIG;      /*! (@ 0x00000002) RPM2PWM Fan configuration */
    __IOM uint8_t  PWM_DIVIDER;     /*! (@ 0x00000004) RPM2PWM PWM divider */
    __IOM uint8_t  GAIN;            /*! (@ 0x00000005) RPM2PWM Gain */
    __IOM uint8_t  SPIN_UP_CFG;     /*! (@ 0x00000006) RPM2PWM Fan spin-up configuration */
    __IOM uint8_t  STEP;            /*! (@ 0x00000007) RPM2PWM Fan step */
    __IOM uint8_t  MIN_DRIVE;       /*! (@ 0x00000008) RPM2PWM Fan minimum drive */
    __IOM uint8_t  VALID_TACH_CNT;  /*! (@ 0x00000009) RPM2PWM valid tachometer count */
    __IOM uint16_t DRIVE_FAIL_BAND; /*! (@ 0x0000000A) RPM2PWM Fan drive fail band */
    __IOM uint16_t TACH_TARGET;     /*! (@ 0x0000000C) RPM2PWM Tachometer target */
    __IOM uint16_t TACH_READING;    /*! (@ 0x0000000E) RPM2PWM Tachometer reading */
    __IOM uint8_t  PWM_BASE_FREQ;   /*! (@ 0x00000010) RPM2PWM PWM driver base frequency */
    __IOM uint8_t  FAN_STATUS;      /*! (@ 0x00000011) RPM2PWM Fan status */
            uint8_t RSVDA[2];
} RPM2PWM_TypeDef;

/* =========================================================================*/
/* ================            VBATR                       ================ */
/* =========================================================================*/

/**
  * @brief VBAT Register Bank (VBATR)
  */
typedef struct
{                           /*!< (@ 0x4000A400) VBATR Structure   */
    __IOM uint8_t  PFRS;            /*! (@ 0x00000000) VBATR Power Fail Reset Status */
          uint8_t  RSVD1[7];
    __IOM uint8_t  CLK32_EN;        /*! (@ 0x00000008) VBATR 32K clock enable */
          uint8_t  RSVD2[0x20 - 0x09];
    __IOM uint32_t MONOTONIC_CNT_LO; /*! (@ 0x00000020) VBATR monotonic count lo */
    __IOM uint32_t MONOTONIC_CNT_HI; /*! (@ 0x00000024) VBATR monotonic count hi */
    __IOM uint32_t VW_BCKUP;         /*! (@ 0x00000028) VBATR VWIRE backup */
} VBATR_TypeDef;

/* =========================================================================*/
/* ================            VBATM                       ================ */
/* =========================================================================*/

/**
  * @brief VBAT Memory (VBATM)
  */
#define VBAT_MEM_LEN    64

typedef struct
{               /*!< (@ 0x4000A800) VBATM Structure   */
    union {
        uint32_t MEM32[(VBAT_MEM_LEN)/4];
        uint16_t MEM16[(VBAT_MEM_LEN)/2];
        uint8_t  MEM8[VBAT_MEM_LEN];
    };
} VBATM_TypeDef;

/* =========================================================================*/
/* ================            WKTMR                       ================ */
/* =========================================================================*/

/**
  * @brief Week Timer (WKTMR)
  */

typedef struct
{               /*!< (@ 0x4000AC80) WKTMR Structure   */
    __IOM uint8_t  CONTROL;         /*! (@ 0x00000000) WKTMR control */
          uint8_t  RSVD1[3];
    __IOM uint32_t ALARM_CNT;       /*! (@ 0x00000004) WKTMR Week alarm counter */
    __IOM uint32_t TMR_COMPARE;     /*! (@ 0x00000008) WKTMR Week timer compare */
    __IOM uint32_t CLK_DIV;         /*! (@ 0x0000000C) WKTMR Clock Divider */
    __IOM uint8_t  SUBSEC_INTR_SEL; /*! (@ 0x00000010) WKTMR Sub-second interrupt select */
          uint8_t  RSVD2[3];
    __IOM uint32_t SUBWK_CTRL;      /*! (@ 0x00000014) WKTMR Sub-week control */
    __IOM uint32_t SUBWK_ALARM;     /*! (@ 0x00000018) WKTMR Sub-week alarm */
    __IOM uint32_t BGPO_DATA;       /*! (@ 0x0000001C) WKTMR BGPO data */
    __IOM uint32_t BGPO_POWER;      /*! (@ 0x00000020) WKTMR BGPO power */
    __IOM uint32_t BGPO_RESET;      /*! (@ 0x00000024) WKTMR BGPO reset */
} WKTMR_TypeDef;

/* =========================================================================*/
/* ================            VCI                         ================ */
/* =========================================================================*/

/**
  * @brief VBAT Powered Control Interface (VCI)
  */

typedef struct
{               /*!< (@ 0x4000AE00) VCI Structure   */
    __IOM uint32_t VCI_CTRL_STS;    /*! (@ 0x00000000) VCI control and status */
    __IOM uint32_t LATCH_EN;        /*! (@ 0x00000004) VCI latch enable */
    __IOM uint32_t LATCH_RST;       /*! (@ 0x00000008) VCI latch reset */
    __IOM uint32_t INPUT_EN;        /*! (@ 0x0000000C) VCI input enable */
    __IOM uint32_t HOLD_OFF_CNT;    /*! (@ 0x00000010) VCI hold off count */
    __IOM uint32_t POLARITY;        /*! (@ 0x00000014) VCI polarity */
    __IOM uint32_t POS_EDGE_DET;    /*! (@ 0x00000018) VCI positive edge detect */
    __IOM uint32_t NEG_EDGE_DET;    /*! (@ 0x0000001C) VCI negative edge detect */
    __IOM uint32_t BUFFER_EN;       /*! (@ 0x00000020) VCI buffer enable */
} VCI_TypeDef;

/* =========================================================================*/
/* ================            BBLED                       ================ */
/* =========================================================================*/

/**
  * @brief Breathing-Blinking LED (BBLED)
  */
typedef struct
{               /*!< (@ 0x4000B800) BBLED Structure   */
    __IOM uint32_t CONFIG;      /*! (@ 0x00000000) BBLED configuration */
    __IOM uint32_t LIMIT;       /*! (@ 0x00000004) BBLED limits */
    __IOM uint32_t DELAY;       /*! (@ 0x00000008) BBLED delay */
    __IOM uint32_t STEP;        /*! (@ 0x0000000C) BBLED update step size */
    __IOM uint32_t INTERVAL;    /*! (@ 0x00000010) BBLED update interval */
    __IOM uint32_t OUT_DELAY;   /*! (@ 0x00000014) BBLED output delay */
} BBLED_TypeDef;

/* =========================================================================*/
/* ================            BC-Link                     ================ */
/* =========================================================================*/

/**
  * @brief BC-Link Master (BCM)
  */
typedef struct
{               /*!< (@ 0x4000CD00) BBLED Structure   */
    __IOM uint8_t  STATUS;      /*! (@ 0x00000000) Status */
          uint8_t  RSVD1[3];
    __IOM uint8_t  SLV_ADDR;    /*! (@ 0x00000004) Companion(slave) address */
          uint8_t  RSVD2[3];
    __IOM uint8_t  BC_DATA;     /*! (@ 0x00000008) Data */
          uint8_t  RSVD3[3];
    __IOM uint8_t  CLK_SEL;     /*! (@ 0x0000000C) Clock select */
          uint8_t  RSVD4[3];
} BCM_TypeDef;

/* =========================================================================*/
/* ================            ECIA                        ================ */
/* =========================================================================*/

/**
  * @brief EC Interrupt Aggregator (ECIA)
  */
#define GIRQ_START_NUM              8
#define GIRQ_LAST_NUM               26
#define GIRQ_IDX(girq)              ((girq) - 8)
#define GIRQ_IDX_MAX                19

typedef struct
{               /*!< (@ 0x4000E000) ECIA Structure   */
    struct {
        __IOM uint32_t SOURCE;
        __IOM uint32_t EN_SET;
        __IOM uint32_t RESULT;
        __IOM uint32_t EN_CLR;
              uint32_t RSVDA;
    } GIRQ[GIRQ_IDX_MAX];
    uint32_t RSVDA[((0x200 - ((GIRQ_IDX_MAX) * (20))) / 4)]; /* offsets 0x0120 - 0x1FF */
    __IOM uint32_t BLOCK_EN_SET;
    __IOM uint32_t BLOCK_EN_CLR;
    __IOM uint32_t BLOCK_IRQ_VECT;
} ECIA_TypeDef;

/* =========================================================================*/
/* ================            ECS                         ================ */
/* =========================================================================*/

/**
  * @brief EC Subsystem (ECS)
  */
typedef struct
{                               /*!< (@ 0x4000FC00) ECS Structure   */
    __IOM uint32_t  MSIZE;
    __IOM uint32_t  AHB_ERR_ADDR;
    __IOM uint32_t  TESTA;
    __IOM uint32_t  TESTB;
    __IOM uint32_t  OSC_ID;
    __IOM uint32_t  AHB_ERR_CTRL;
    __IOM uint32_t  INTR_CTRL;
    __IOM uint32_t  ETM_ENABLE;
    __IOM uint32_t  JTAG_ENABLE;
    __IOM uint32_t  OTP_LOCK;
    __IOM uint32_t  WDT_COUNT;
    __IOM uint32_t  AES_HASH_SW;
    __IOM uint32_t  BOOTROM_SCRATCH1;
    __IOM uint32_t  VOLT_REG_TRIM;
    __IOM uint32_t  SYS_SHUTDOWN_RESET;
    __IOM uint32_t  ADC_BIAS_ADJ;
    __IOM uint32_t  BOOTROM_SCRATCH2;
    __IOM uint32_t  GPIO_PAD_TEST;
    __IOM uint32_t  BOOTROM_SCRATCH3;
          uint32_t  RSVDA[4];
    __IOM uint32_t  CRYPTO_SOFT_RST;
    __IOM uint32_t  PWR_GRD_TEST1;
    __IOM uint32_t  GPIO_BANK_POWER;
    __IOM uint32_t  FEATURE_LOCK;
    __IOM uint32_t  MISC_LOCK;
    __IOM uint32_t  JTAG_MASTER_CFG;
    __IOM uint32_t  JTAG_MASTER_STS;
    __IOM uint32_t  JTAG_MASTER_TDO;
    __IOM uint32_t  JTAG_MASTER_TDI;
    __IOM uint32_t  JTAG_MASTER_TMS;
    __IOM uint32_t  JTAG_MASTER_CMD;
          uint32_t  RSVDB[2];
    __IOM uint32_t  FW_OVERRIDE_TST;
          uint32_t  RSVDC[((0x100ul - 0x90ul)>>2)-1];
    __IOM uint32_t  BSCAN_ID;
} ECS_TypeDef;

/* =========================================================================*/
/* ================            PCR                         ================ */
/* =========================================================================*/

/**
  * @brief Power Control Reset (PCR)
  */
#define PCR_MAX_SLPEN_IDX   5

typedef struct
{           /*!< (@ 0x40080100) PCR Structure   */
    __IOM uint32_t SYS_SLP_CTRL;
    __IOM uint32_t PROC_CLK_CTRL;
    __IOM uint32_t SLOW_CLK_CTRL;
    __IOM uint32_t OSC_ID;
    __IOM uint32_t PWR_RST_STS;
    __IOM uint32_t PWR_RST_CTRL;
    __IOM uint32_t SYS_RESET;
    __IOM uint32_t PKE_CLK_CTRL;
    __IOM uint32_t TEST_OSC_CAL;
          uint32_t RSVDA[3];
    __IOM uint32_t SLEEP_EN[PCR_MAX_SLPEN_IDX];
          uint32_t RSVDB[3];
    __IOM uint32_t CLOCK_REQ[PCR_MAX_SLPEN_IDX];
          uint32_t RSVDC[3];
    __IOM uint32_t RESET_EN[PCR_MAX_SLPEN_IDX];
} PCR_TypeDef;

/* =========================================================================*/
/* ================            GPIO                        ================ */
/* =========================================================================*/

/**
  * @brief GPIO Control (GPIO)
  */
#define MAX_GPIO_PIN    0xADul
#define MAX_GPIO_BANK   6u
#define GPIO_LOCK5_IDX  (0u)
#define GPIO_LOCK4_IDX  (1u)
#define GPIO_LOCK3_IDX  (2u)
#define GPIO_LOCK2_IDX  (3u)
#define GPIO_LOCK1_IDX  (4u)
#define GPIO_LOCK0_IDX  (5u)
#define GPIO_LOCK_MAX_IDX (6u)

#define GPIO_CTRL_BEGIN     0
#define GPIO_CTRL_END       0x2B4
#define GPIO_PIN_BEGIN      0x300
#define GPIO_PIN_END        0x318
#define GPIO_POUT_BEGIN     0x380
#define GPIO_POUT_END       0x398
#define GPIO_LOCK_BEGIN     0x3E8
#define GPIO_LOCK_END       0x400
#define GPIO_CTRL2_BEGIN    0x500
#define GPIO_CTRL2_END      0x7B4

typedef struct
{                   /*!< (@ 0x40081000) GPIO Structure   */
    __IOM uint32_t CTRL[MAX_GPIO_BANK * 32];        /*!< (@ 0x00000000) GPIO Control */
    __IOM uint32_t PARIN[MAX_GPIO_BANK];            /*!< (@ 0x00000300) GPIO Parallel input */
          uint32_t RSVDA[(0x380 - 0x318)/4];
    __IOM uint32_t PAROUT[MAX_GPIO_BANK];           /*!< (@ 0x00000380) GPIO Parallel output */
          uint32_t RSVDB[(0x3E8 - 0x398)/4];
    __IOM uint32_t LOCK[MAX_GPIO_BANK];             /*!< (@ 0x000003E8) GPIO Lock 0x3E8 - 0x3FF */
          uint32_t RSVDC[(0x500 - 0x400)/4];
    __IOM uint32_t CTRL2[MAX_GPIO_BANK * 32];       /*!< (@ 0x00000500) GPIO Control 2 */
} GPIO_TypeDef;

/* =========================================================================*/
/* ================            EFUSE                       ================ */
/* =========================================================================*/

/**
  * @brief EFUSE (EFUSE)
  */
#define EFUSE_BITLEN        (4096ul)
#define EFUSE_BYTELEN       (4096ul >> 3)
#define EFUSE_HWORDLEN      (4096ul >> 4)

typedef struct
{               /*!< (@ 0x40082000) EFUSE Structure   */
    __IOM uint16_t CONTROL;
          uint16_t RSVDA[1];
    __IOM uint16_t MAN_CONTROL;
    __IOM uint16_t MAN_MODE;
          uint32_t RSVDC[1];
    __IOM uint16_t MAN_DATA;
          uint16_t RSVDD[1];
    union {
        __IOM uint16_t MEM16[EFUSE_HWORDLEN];
        __IOM uint8_t  MEM8[EFUSE_BYTELEN];
    };
} EFUSE_TypeDef;

/* =========================================================================*/
/* ================            MBOX                        ================ */
/* =========================================================================*/

/**
  * @brief Mailbox (MBOX)
  */
#define MBOX_SLOTS  32

typedef struct
{               /*!< (@ 0x400F0000) MBOX Structure   */
    __IOM uint8_t  INDEX;
    __IOM uint8_t  MB_DATA;
          uint8_t  RSVDA[2];
          uint32_t RSVDB[(0x100 - 0x04)/4];
    __IOM uint32_t HOST_TO_EC;
    __IOM uint32_t EC_TO_HOST;
    __IOM uint32_t SMI_SRC;
    __IOM uint32_t SMI_MASK;
    union {
        __IOM uint32_t MB32[MBOX_SLOTS/4];
        __IOM uint16_t MB16[MBOX_SLOTS/2];
        __IOM uint8_t  MB8[MBOX_SLOTS];
    };
} MBOX_TypeDef;

/* =========================================================================*/
/* ================            KBC                         ================ */
/* =========================================================================*/

/**
  * @brief MIF8042 keyboard controller (KBC)
  */
typedef struct
{               /*!< (@ 0x400F0400) MBOX Structure   */
    __IOM uint8_t  RT_DATA;                 /*!< (@ 0x400F0400) KBC Run-time Host Cmd/Data   */
          uint8_t  RSVD1[3];
    __IOM uint8_t  RT_STATUS_CMD;           /*!< (@ 0x400F0404) KBC Run-time Host Status/Cmd   */
          uint8_t  RSVD2[0x100 - 0x05];
    __IOM uint8_t  EC_HOST_DATA;            /*!< (@ 0x400F0500) KBC EC-only Host to/from EC Data   */
          uint8_t  RSVD3[3];
    __IOM uint8_t  EC_STATUS;               /*!< (@ 0x400F0504) KBC EC-only EC Status   */
          uint8_t  RSVD4[3];
    __IOM uint8_t  EC_CONTROL;                /*!< (@ 0x400F0508) KBC EC-only Control   */
          uint8_t  RSVD5[3];
    __IOM uint8_t  EC_AUX_DATA;             /*!< (@ 0x400F050C) KBC EC-only Aux Data   */
          uint8_t  RSVD6[7];
    __IOM uint8_t  EC_PC_OBF;               /*!< (@ 0x400F0514) KBC EC-only PC OBF   */
          uint8_t  RSVDC[0x330 - 0x115];
    __IOM uint8_t  EC_ACTIVATE;             /*!< (@ 0x400F0830) KBC EC-only Activate   */
          uint8_t  RSVDD[3];
} KBC_TypeDef;

/* =========================================================================*/
/* ================            ACPIEC                      ================ */
/* =========================================================================*/

/**
  * @brief ACPI EC interface (ACPI_EC)
  */

typedef struct
{               /*!< (@ 0x400F0800) ACPI EC Structure   */
    union {
        __IOM uint32_t RT_DATA;
        __IOM uint8_t  RT_DATA8[4];
    };
    __IOM uint8_t RT_CMD_STS;
    __IOM uint8_t RT_BYTE_CTRL;
          uint8_t RSVDA[2];
          uint8_t RSVDB[0x100 - 0x08];
    union {
        __IOM uint32_t EC2OS_DATA;
        __IOM uint8_t  EC2OS_DATA8[4];
    };
    __IOM uint8_t EC_STATUS;
    __IOM uint8_t EC_BYTE_CTRL;
          uint8_t RSVDC[2];
    union {
        __IOM uint32_t OS2EC_DATA;
        __IOM uint8_t  OS2EC_DATA8[4];
    };
} ACPI_EC_TypeDef;

/* =========================================================================*/
/* ================            ACPIPM1                     ================ */
/* =========================================================================*/

/**
  * @brief ACPI PM1 interface (ACPIPM1)
  */

typedef struct
{               /*!< (@ 0x400F1C00) ACPIPM1 Structure   */
    __IOM uint8_t PM1_STS1;
    __IOM uint8_t PM1_STS2;
    __IOM uint8_t PM1_EN1;
    __IOM uint8_t PM1_EN2;
    __IOM uint8_t PM1_CTRL1;
    __IOM uint8_t PM1_CTRL2;
    __IOM uint8_t PM2_CTRL1;
    __IOM uint8_t PM2_CTRL2;
} ACPI_PM1_TypeDef;

/* =========================================================================*/
/* ================            PORT92                      ================ */
/* =========================================================================*/

/**
  * @brief Port 92h interface (PORT92)
  */

typedef struct
{               /*!< (@ 0x400F2000) PORT92 Structure   */
    __IOM uint8_t  RT_PORT92;
          uint8_t  RSVD1[0x100 - 0x01];
    __IOM uint8_t  GATEA20_CTRL;
          uint8_t  RSVD2[7];
    __IOM uint8_t  SETGA20L;
          uint8_t  RSVD3[3];
    __IOM uint8_t  RSTGA20L;
          uint8_t  RSVD4[0x330 - 0x10D];
    __IOM uint8_t  PORT92_EN;
          uint8_t  RSVD5[3];
} PORT92_TypeDef;

/* =========================================================================*/
/* ================            UART                        ================ */
/* =========================================================================*/

/**
  * @brief UART interface (UART)
  */

typedef struct
{               /*!< (@ 0x400F2400) UART Structure   */
    union {
        __OM  uint8_t TXB;
        __IM  uint8_t RXB;
        __IOM uint8_t BRG_LSB;
    };
    union {
        __IOM uint8_t IER;
        __IOM uint8_t BRG_MSB;
    };
    union {
        __OM  uint8_t FCR;
        __IM  uint8_t IIR;
    };
    __IOM uint8_t LCR;
    __IOM uint8_t MCR;
    __IOM uint8_t LSR;
    __IOM uint8_t MSR;
    __IOM uint8_t SCR;
          uint8_t RSVDA[0x330 - 0x08];
    __IOM uint8_t ACTIVATE;
          uint8_t RSVDB[0x3F0 - 0x331];
    __IOM uint8_t CONFIG_SEL;
} UART_TypeDef;

/* =========================================================================*/
/* ================            LPC                         ================ */
/* =========================================================================*/

/**
  * @brief LPC interface (LPC)
  */
#define LPC_SERIRQ_MAX      16

#define LPC_IO_BAR_LPC_CFG  0
#define LPC_IO_BAR_MBOX     1
#define LPC_IO_BAR_KBC      2
#define LPC_IO_BAR_ACPI_EC0 3
#define LPC_IO_BAR_ACPI_EC1 4
#define LPC_IO_BAR_ACPI_EC2 5
#define LPC_IO_BAR_ACPI_EC3 6
#define LPC_IO_BAR_ACPI_EC4 7
#define LPC_IO_BAR_ACPI_PM1 8
#define LPC_IO_BAR_PORT92   9
#define LPC_IO_BAR_UART0    10
#define LPC_IO_BAR_UART1    11
#define LPC_IO_BAR_EMI0     12
#define LPC_IO_BAR_EMI1     13
#define LPC_IO_BAR_EMI2     14
#define LPC_IO_BAR_P80A     15
#define LPC_IO_BAR_P80B     16
#define LPC_IO_BAR_RTC      17
#define LPC_IO_BAR_RSVD1    18
#define LPC_IO_BAR_LASIC    19
#define LPC_IO_BAR_MAX      20

/* Mailbox Memory Bar */
#define LPC_MB_MBOX_MASK        0
#define LPC_MB_MBOX_FRAME       1
#define LPC_MB_MBOX_HOST_ADDR0  2
#define LPC_MB_MBOX_HOST_ADDR1  3
#define LPC_MB_MBOX_HOST_ADDR2  4
#define LPC_MB_MBOX_HOST_ADDR3  5
/* ACPI EC0 Memory Bar */
#define LPC_MB_AEC0_MASK        6
#define LPC_MB_AEC0_FRAME       7
#define LPC_MB_AEC0_HOST_ADDR0  8
#define LPC_MB_AEC0_HOST_ADDR1  9
#define LPC_MB_AEC0_HOST_ADDR2  10
#define LPC_MB_AEC0_HOST_ADDR3  11
/* ACPI EC1 Memory Bar */
#define LPC_MB_AEC1_MASK        12
#define LPC_MB_AEC1_FRAME       13
#define LPC_MB_AEC1_HOST_ADDR0  14
#define LPC_MB_AEC1_HOST_ADDR1  15
#define LPC_MB_AEC1_HOST_ADDR2  16
#define LPC_MB_AEC1_HOST_ADDR3  17
/* ACPI EC2 Memory Bar */
#define LPC_MB_AEC2_MASK        18
#define LPC_MB_AEC2_FRAME       19
#define LPC_MB_AEC2_HOST_ADDR0  20
#define LPC_MB_AEC2_HOST_ADDR1  21
#define LPC_MB_AEC2_HOST_ADDR2  22
#define LPC_MB_AEC2_HOST_ADDR3  23
/* ACPI EC3 Memory Bar */
#define LPC_MB_AEC3_MASK        24
#define LPC_MB_AEC3_FRAME       25
#define LPC_MB_AEC3_HOST_ADDR0  26
#define LPC_MB_AEC3_HOST_ADDR1  27
#define LPC_MB_AEC3_HOST_ADDR2  28
#define LPC_MB_AEC3_HOST_ADDR3  29
/* ACPI EC4 Memory Bar */
#define LPC_MB_AEC4_MASK        30
#define LPC_MB_AEC4_FRAME       31
#define LPC_MB_AEC4_HOST_ADDR0  32
#define LPC_MB_AEC4_HOST_ADDR1  33
#define LPC_MB_AEC4_HOST_ADDR2  34
#define LPC_MB_AEC4_HOST_ADDR3  35
/* EMI0 Memory Bar */
#define LPC_MB_EMI0_MASK        36
#define LPC_MB_EMI0_FRAME       37
#define LPC_MB_EMI0_HOST_ADDR0  38
#define LPC_MB_EMI0_HOST_ADDR1  39
#define LPC_MB_EMI0_HOST_ADDR2  40
#define LPC_MB_EMI0_HOST_ADDR3  41
/* EMI1 Memory Bar */
#define LPC_MB_EMI1_MASK        42
#define LPC_MB_EMI1_FRAME       43
#define LPC_MB_EMI1_HOST_ADDR0  44
#define LPC_MB_EMI1_HOST_ADDR1  45
#define LPC_MB_EMI1_HOST_ADDR2  46
#define LPC_MB_EMI1_HOST_ADDR3  47
/* EMI2 Memory Bar */
#define LPC_MB_EMI2_MASK        48
#define LPC_MB_EMI2_FRAME       49
#define LPC_MB_EMI2_HOST_ADDR0  50
#define LPC_MB_EMI2_HOST_ADDR1  51
#define LPC_MB_EMI2_HOST_ADDR2  52
#define LPC_MB_EMI2_HOST_ADDR3  53

#define LPC_MEM8_BAR_MAX    (0x400 - 0x3C0)

typedef struct
{               /*!< (@ 0x400F3000) LPC Structure   */
          uint32_t RSVDA[0x104/4];
    __IOM uint32_t BUS_MONITOR;     /*! (@ 0x00000104) LPC Bus Monitor */
    __IOM uint32_t HOST_BUS_ERR;    /*! (@ 0x00000108) LPC Host Bus Error */
    __IOM uint32_t EC_SERIRQ;       /*! (@ 0x0000010C) LPC EC SERIRQ */
    __IOM uint32_t EC_CLK_CTRL;     /*! (@ 0x00000110) LPC EC Clock Control */
    __IOM uint32_t MCHP_TEST1;      /*! (@ 0x00000114) LPC MCHP Test 1 */
    __IOM uint32_t MCHP_TEST2;      /*! (@ 0x00000118) LPC MCHP Test 2 */
          uint32_t RSVDB[1];
    __IOM uint32_t BAR_INHIBIT;     /*! (@ 0x00000120) LPC BAR Inhibit */
    __IOM uint32_t MCHP_TEST3;      /*! (@ 0x00000124) LPC MCHP Test 3 */
    __IOM uint32_t MCHP_TEST4;      /*! (@ 0x00000128) LPC MCHP Test 4 */
    __IOM uint32_t MCHP_TEST5;      /*! (@ 0x0000012C) LPC MCHP Test 5 */
    __IOM uint32_t BAR_INIT;        /*! (@ 0x00000130) LPC BAR Init */
          uint32_t RSVDC[(0x1F8 - 0x134)/4];
    __IOM uint32_t SRAM0_BAR;       /*! (@ 0x000001F8) LPC SRAM 0 BAR */
    __IOM uint32_t SRAM1_BAR;       /*! (@ 0x000001FC) LPC SRAM 1 BAR */
          uint32_t RSVDD[(0x330 - 0x200)/4];
    __IOM uint32_t ACTIVATE;        /*! (@ 0x00000330) LPC SRAM 1 BAR */
          uint32_t RSVDE[(0x340 - 0x334)/4];
    __IOM uint8_t  SERIRQ_CFG[LPC_SERIRQ_MAX];   /*! (@ 0x00000340) LPC SERIRQ 0-15 Config */
          uint8_t  RSVDF[0x360 - 0x350];
    __IOM uint32_t IO_BAR[LPC_IO_BAR_MAX];       /*! (@ 0x00000360) LPC SERIRQ 0-15 Config */
    __IOM uint32_t SRAM0_BASE_CFG;      /*! (@ 0x000003B0) LPC SRAM 0 Base Config */
    __IOM uint32_t SRAM0_HOST_ADDR;     /*! (@ 0x000003B4) LPC SRAM 0 Base Host Addr */
    __IOM uint32_t SRAM1_BASE_CFG;      /*! (@ 0x000003B8) LPC SRAM 1 Base Config */
    __IOM uint32_t SRAM1_HOST_ADDR;     /*! (@ 0x000003BC) LPC SRAM 1 Base Host Addr */
    __IOM uint8_t  MEM_BAR8[LPC_MEM8_BAR_MAX];
} LPC_TypeDef;

/* =========================================================================*/
/* ================           eSPI IO Component            ================ */
/* =========================================================================*/

/**
  * @brief ESPI Host interface IO Component (ESPI_IO)
  */

/* Each IO BAR is 4 bytes */
#define ESPI_IO_BAR_SIZE            4
#define ESPI_IO_BAR_SIZE_PWROF2     2

/* IO BAR indices */
#define ESPI_IO_BAR_IOC     0
#define ESPI_IO_BAR_MEMC    1
#define ESPI_IO_BAR_MBOX    2
#define ESPI_IO_BAR_KBC     3
#define ESPI_IO_BAR_AEC0    4
#define ESPI_IO_BAR_AEC1    5
#define ESPI_IO_BAR_AEC2    6
#define ESPI_IO_BAR_AEC3    7
#define ESPI_IO_BAR_AEC4    8
#define ESPI_IO_BAR_PM1     9
#define ESPI_IO_BAR_PORT92  10
#define ESPI_IO_BAR_UART0   11
#define ESPI_IO_BAR_UART1   12
#define ESPI_IO_BAR_EMI0    13
#define ESPI_IO_BAR_EMI1    14
#define ESPI_IO_BAR_EMI2    15
#define ESPI_IO_BAR_P80CAP0 16
#define ESPI_IO_BAR_P80CAP1 17
#define ESPI_IO_BAR_RTC     18
#define ESPI_IO_BAR_LASIC   19
#define ESPI_IO_BAR_16B     20
#define ESPI_IO_BAR_MAX     21

/* SERIRQ indices */
#define ESPI_SERIRQ_MBOX0   0
#define ESPI_SERIRQ_MBOX1   1
#define ESPI_SERIRQ_KBC0    2
#define ESPI_SERIRQ_KBC1    3
#define ESPI_SERIRQ_AEC0    4
#define ESPI_SERIRQ_AEC1    5
#define ESPI_SERIRQ_AEC2    6
#define ESPI_SERIRQ_AEC3    7
#define ESPI_SERIRQ_AEC4    8
#define ESPI_SERIRQ_UART0   9
#define ESPI_SERIRQ_UART1   10
#define ESPI_SERIRQ_EMI0_0  11
#define ESPI_SERIRQ_EMI0_1  12
#define ESPI_SERIRQ_EMI1_0  13
#define ESPI_SERIRQ_EMI1_1  14
#define ESPI_SERIRQ_EMI2_0  15
#define ESPI_SERIRQ_EMI2_1  16
#define ESPI_SERIRQ_RTC     17
#define ESPI_SERIRQ_EC      18
#define ESPI_SERIRQ_MAX     19

typedef struct
{       /*!< (@ 0x400F3400 + 0x0100) ESPI IO Component Structure Peripheral Channel */
    __IOM uint32_t PC_LAST_CYCLE_ADDR[2];       /*! (@ 0x00000100) Periph Chan Last Cycle address */
    __IOM uint32_t PC_LAST_CYCLE_LEN_TYPE_TAG;  /*! (@ 0x00000108) Periph Chan Last Cycle length/type/tag */
    __IOM uint32_t PC_ERR_ADDR[2];              /*! (@ 0x0000010C) Periph Chan Error Address */
    __IOM uint32_t PC_STATUS;                   /*! (@ 0x00000114) Periph Chan Status */
    __IOM uint32_t PC_IEN;                      /*! (@ 0x00000118) Periph Chan IEN */
          uint32_t RSVD1[1];
    __IOM uint32_t BAR_INHIBIT[2];              /*! (@ 0x00000120) I/O BAR Inhibit */
    __IOM uint32_t BAR_INIT;                    /*! (@ 0x00000128) BAR Init */
    __IOM uint32_t EC_IRQ;                      /*! (@ 0x0000012C) EC IRQ */
          uint32_t RSVD2[1];
    __IOM uint32_t IO_BAR_LDN_MASK[ESPI_IO_BAR_MAX]; /*! (@ 0x00000134) IO BAR LDN & Mask */
          uint32_t RSVD3[((0x334 - 0x134) - (ESPI_IO_BAR_MAX * 4))/4];
    __IOM uint32_t IO_BAR_HADDR_EN[ESPI_IO_BAR_MAX]; /*! (@ 0x00000334) IO BAR Host Address & Enable */
          uint32_t RSVD4[((0x3AC - 0x334) - (ESPI_IO_BAR_MAX * 4))/4];
    __IOM uint8_t  SERIRQ[ESPI_SERIRQ_MAX]; /*! (@ 0x000003AC) IO SERIRQ */
} ESPI_IO_PC_TypeDef;

typedef struct
{       /*!< (@ 0x400F3400 + 0x0220) ESPI IO Component LTR */
    __IOM uint32_t STATUS;      /*! (@ 0x00000220) LTR Periph Status */
    __IOM uint32_t ENABLE;      /*! (@ 0x00000224) LTR Periph Enable */
    __IOM uint32_t CONTROL;     /*! (@ 0x00000228) LTR Periph Control */
    __IOM uint32_t MESG;        /*! (@ 0x0000022C) LTR Periph Message */
} ESPI_IO_LTR_TypeDef;

typedef struct
{       /*!< (@ 0x400F3400 + 0x0240) ESPI IO Component OOB */
    __IOM uint32_t RX_ADDR_LO;  /*! (@ 0x00000240) OOB Receive Address bits[31:0] */
    __IOM uint32_t RX_ADDR_HI;  /*! (@ 0x00000244) OOB Receive Address bits[63:32] */
    __IOM uint32_t TX_ADDR_LO;  /*! (@ 0x00000248) OOB Transmit Address bits[31:0] */
    __IOM uint32_t TX_ADDR_HI;  /*! (@ 0x0000024C) OOB Transmit Address bits[63:32] */
    __IOM uint32_t RX_LEN;      /*! (@ 0x00000250) OOB Receive length */
    __IOM uint32_t TX_LEN;      /*! (@ 0x00000254) OOB Transmit length */
    __IOM uint32_t RX_CTRL;     /*! (@ 0x00000258) OOB Receive control */
    __IOM uint32_t RX_IEN;      /*! (@ 0x0000025C) OOB Receive interrupt enable */
    __IOM uint32_t RX_STATUS;   /*! (@ 0x00000260) OOB Receive interrupt status */
    __IOM uint32_t TX_CTRL;     /*! (@ 0x00000264) OOB Transmit control */
    __IOM uint32_t TX_IEN;      /*! (@ 0x00000268) OOB Transmit interrupt enable */
    __IOM uint32_t TX_STATUS;   /*! (@ 0x0000026C) OOB Transmit interrupt status */
} ESPI_IO_OOB_TypeDef;

typedef struct
{       /*!< (@ 0x400F3400 + 0x0280) ESPI IO Component FC(Flash Channel) */
    __IOM uint32_t FLASH_ADDR_LO;    /*! (@ 0x00000280) FC flash address bits[31:0] */
    __IOM uint32_t FLASH_ADDR_HI;    /*! (@ 0x00000284) FC flash address bits[63:32] */
    __IOM uint32_t BUF_ADDR_LO;      /*! (@ 0x00000288) FC EC buffer address bits[31:0] */
    __IOM uint32_t BUF_ADDR_HI;      /*! (@ 0x0000028C) FC EC buffer address bits[63:32] */
    __IOM uint32_t XFR_LEN;          /*! (@ 0x00000290) FC transfer length */
    __IOM uint32_t CONTROL;          /*! (@ 0x00000294) FC Control */
    __IOM uint32_t IEN;              /*! (@ 0x00000298) FC interrupt enable */
    __IOM uint32_t CONFIG;           /*! (@ 0x0000029C) FC configuration */
    __IOM uint32_t STATUS;           /*! (@ 0x000002A0) FC status */
} ESPI_IO_FC_Typedef;

typedef struct
{       /*!< (@ 0x400F3400 + 0x2B0) ESPI IO Component Structure Capabilities */
    __IOM uint32_t VW_STATUS;           /*! (@ 0x000002B0) VW status */
          uint32_t RSVD1[(0x2E0 - 0x2B4)/4];
    __IOM uint8_t  CAP_ID;              /*! (@ 0x000002E0) Capabilities ID */
    __IOM uint8_t  GLB_CAP0;            /*! (@ 0x000002E1) Global Capabilities 0 */
    __IOM uint8_t  GLB_CAP1;            /*! (@ 0x000002E2) Global Capabilities 1 */
    __IOM uint8_t  PC_CAP;              /*! (@ 0x000002E3) Periph Chan Capabilities */
    __IOM uint8_t  VW_CAP;              /*! (@ 0x000002E4) Virtual Wire Chan Capabilities */
    __IOM uint8_t  OOB_CAP;             /*! (@ 0x000002E5) OOB Chan Capabilities */
    __IOM uint8_t  FC_CAP;              /*! (@ 0x000002E6) Flash Chan Capabilities */
    __IOM uint8_t  PC_READY;            /*! (@ 0x000002E7) PC ready */
    __IOM uint8_t  OOB_READY;           /*! (@ 0x000002E8) OOB ready */
    __IOM uint8_t  FC_READY;            /*! (@ 0x000002E9) OOB ready */
    __IOM uint8_t  RESET_STS;           /*! (@ 0x000002EA) eSPI Reset interrupt status */
    __IOM uint8_t  RESET_IEN;           /*! (@ 0x000002EB) eSPI Reset interrupt enable */
    __IOM uint8_t  PLTRST_SRC;          /*! (@ 0x000002EC) Platform Reset Source */
    __IOM uint8_t  VW_READY;            /*! (@ 0x000002ED) VW ready */
          uint8_t  RSVD2[2];
          uint8_t  RSVD3[(0x330 - 0x2F0)];
    __IOM uint8_t  ACTIVATE;            /*! (@ 0x00000330) eSPI IO activate */
          uint8_t  RSVD4[0x3F0 - 0x331];
    __IOM uint32_t VW_ERROR;            /*! (@ 0x000003F0) VW Error */
} ESPI_IO_CAP_TypeDef;


/* =========================================================================*/
/* ================           eSPI Memory Component        ================ */
/* =========================================================================*/

/**
  * @brief ESPI Host interface Memory Component (ESPI_MEM)
  */
typedef struct
{               /*!< (@ 0x400F3800 + 0x200) ESPI Memory Component Structure   */
    __IOM uint32_t BM_STATUS;           /*! (@ 0x00000200) Bus Master Status */
    __IOM uint32_t BM_IEN;              /*! (@ 0x00000204) Bus Master interrupt enable */
    __IOM uint32_t BM_CFG;              /*! (@ 0x00000208) Bus Master configuration */
            uint32_t RSVDE[1];
    __IOM uint32_t BM1_CTRL;            /*! (@ 0x00000210) Bus Master 1 control */
    __IOM uint32_t BM1_HOST_ADDR_LO;    /*! (@ 0x00000214) Bus Master 1 host address bits[31:0] */
    __IOM uint32_t BM1_HOST_ADDR_HI;    /*! (@ 0x00000218) Bus Master 1 host address bits[63:32] */
    __IOM uint32_t BM1_EC_ADDR_LO;      /*! (@ 0x0000021C) Bus Master 1 EC address bits[31:0] */
    __IOM uint32_t BM1_EC_ADDR_HI;      /*! (@ 0x00000220) Bus Master 1 EC address bits[63:32] */
    __IOM uint32_t BM2_CTRL;            /*! (@ 0x00000224) Bus Master 2 control */
    __IOM uint32_t BM2_HOST_ADDR_LO;    /*! (@ 0x00000228) Bus Master 2 host address bits[31:0] */
    __IOM uint32_t BM2_HOST_ADDR_HI;    /*! (@ 0x0000022C) Bus Master 2 host address bits[63:32] */
    __IOM uint32_t BM2_EC_ADDR_LO;      /*! (@ 0x00000230) Bus Master 2 EC address bits[31:0] */
    __IOM uint32_t BM2_EC_ADDR_HI;      /*! (@ 0x00000234) Bus Master 2 EC address bits[63:32] */
} ESPI_MEM_BM_Typedef;

#define ESPI_MEM_BAR_SIZE       10

typedef struct
{       /*!< (@ 0x400F3800 + 0x130) ESPI Memory Component Structure   */
    uint16_t MASK_LDN_MBOX;         /*!< (@ 0x0130) Mailbox BAR Mask & LDN   */
    uint16_t VIRT_MBOX;             /*!< (@ 0x0132) Mailbox BAR Virtualized control */
    uint16_t RSVD1[3];
    uint16_t MASK_LDN_ACPI_EC0;     /*!< (@ 0x013A) ACPI EC0 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EC0;
    uint16_t RSVD2[3];
    uint16_t MASK_LDN_ACPI_EC1;     /*!< (@ 0x0144) ACPI EC1 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EC1;
    uint16_t RSVD3[3];
    uint16_t MASK_LDN_ACPI_EC2;     /*!< (@ 0x014E) ACPI EC2 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EC2;
    uint16_t RSVD4[3];
    uint16_t MASK_LDN_ACPI_EC3;     /*!< (@ 0x0158) ACPI EC3 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EC3;
    uint16_t RSVD5[3];
    uint16_t MASK_LDN_ACPI_EC4;     /*!< (@ 0x0162) ACPI EC4 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EC4;
    uint16_t RSVD6[3];
    uint16_t MASK_LDN_EMI0;         /*!< (@ 0x016C) ACPI EMI0 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EMI0;
    uint16_t RSVD7[3];
    uint16_t MASK_LDN_EMI1;         /*!< (@ 0x0176) ACPI EMI1 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EMI1;
    uint16_t RSVD8[3];
    uint16_t MASK_LDN_EMI2;         /*!< (@ 0x0180) ACPI EMI2 BAR Mask & LDN   */
    uint16_t VIRT_ACPI_EMI2;
    uint16_t RSVD9[3];
    uint16_t RSVD10[(0x1AC - 0x18A)/2];
    uint16_t SRAM0_EC_CTRL;         /*!< (@ 0x01AC) SRAM0 BAR EC Control */
    uint16_t SRAM0_EC_ADDR[2];
    uint16_t RSVD11[2];
    uint16_t SRAM1_EC_CTRL;         /*!< (@ 0x01B6) SRAM1 BAR EC Control */
    uint16_t SRAM1_EC_ADDR[2];
    uint16_t RSVD12[2];
    uint16_t RSVD13[(0x330 - 0x1C0)/2];
    uint16_t VALID_MBOX;            /*!< (@ 0x0330) Mailbox BAR Valid   */
    uint16_t HOST_ADDR_MBOX[4];
    uint16_t VALID_ACPI_EC0;            /*!< (@ 0x033A) ACPI EC0 BAR Valid   */
    uint16_t HOST_ADDR_ACPI_EC0[4];
    uint16_t VALID_ACPI_EC1;            /*!< (@ 0x0344) ACPI EC1 BAR Valid   */
    uint16_t HOST_ADDR_ACPI_EC1[4];
    uint16_t VALID_ACPI_EC2;            /*!< (@ 0x034E) ACPI EC2 BAR Valid   */
    uint16_t HOST_ADDR_ACPI_EC2[4];
    uint16_t VALID_ACPI_EC3;            /*!< (@ 0x0358) ACPI EC3 BAR Valid   */
    uint16_t HOST_ADDR_ACPI_EC3[4];
    uint16_t VALID_ACPI_EC4;            /*!< (@ 0x0362) ACPI EC4 BAR Valid   */
    uint16_t HOST_ADDR_ACPI_EC4[4];
    uint16_t VALID_EMI0;                /*!< (@ 0x036C) EMI0 BAR Valid   */
    uint16_t HOST_ADDR_EMI0[4];
    uint16_t VALID_EMI1;                /*!< (@ 0x0376) EMI1 BAR Valid   */
    uint16_t HOST_ADDR_EMI1[4];
    uint16_t VALID_EMI2;                /*!< (@ 0x0380) EMI2 BAR Valid   */
    uint16_t HOST_ADDR_EMI2[4];
    uint16_t RSVD14[(0x3AC - 0x38A)/2];
    uint16_t SRAM0_HOST_CTRL;           /*!< (@ 0x03AC) SRRAM0 BAR Host Control */
    uint16_t SRAM0_HOST_ADDR[4];
    uint16_t SRAM1_HOST_CTRL;           /*!< (@ 0x03B6) SRRAM1 BAR Host Control */
    uint16_t SRAM1_HOST_ADDR[4];
} ESPI_MEM_BAR_Typedef;

/* =========================================================================*/
/* ================           EMI                          ================ */
/* =========================================================================*/

#define EMI_BLOCK_SPACING       0x400
#define EMI_SPACING_PWROF2      10

/**
  * @brief EMI interface (EMI)
  */
typedef struct
{               /*!< (@ 0x400F4000) EMI Structure   */
    __IOM uint8_t  RT_HOST_TO_EC;       /*! (@ 0x00000000) Host to EC command */
    __IOM uint8_t  RT_EC_TO_HOST;       /*! (@ 0x00000001) EC to Host response */
    __IOM uint16_t RT_EC_ADDR;          /*! (@ 0x00000002) Offset in EC memory mapped by EMI */
    union {
        __IOM uint32_t RT_DATA;         /*! (@ 0x00000004) Data */
        __IOM uint16_t RT_DATA16[2];
        __IOM uint8_t  RT_DATA8[4];
    };
    __IOM uint16_t RT_INTR_SRC;         /*! (@ 0x00000008) Interrupt source bit map */
    __IOM uint16_t RT_INTR_MASK;        /*! (@ 0x0000000A) Interrupt source mask */
    __IOM uint32_t RT_APP_ID;           /*! (@ 0x0000000C) application ID */
            uint8_t  RSVDA[0x100 - 0x10];
    __IOM uint8_t HOST_TO_EC;           /*! (@ 0x00000100) EC only copy of Host to EC */
    __IOM uint8_t EC_TO_HOST;           /*! (@ 0x00000101) EC only copy of EC to Host */
            uint8_t RSVDB[2];
    __IOM uint32_t MEM0_BASE;           /*! (@ 0x00000104) Memory region 0 base */
    __IOM uint16_t MEM0_READ_LIMIT;     /*! (@ 0x00000108) Memory region 0 read limit */
    __IOM uint16_t MEM0_WRITE_LIMIT;    /*! (@ 0x0000010A) Memory region 0 write limit */
    __IOM uint32_t MEM1_BASE;           /*! (@ 0x0000010C) Memory region 1 base */
    __IOM uint16_t MEM1_READ_LIMIT;     /*! (@ 0x00000110) Memory region 1 read limit */
    __IOM uint16_t MEM1_WRITE_LIMIT;    /*! (@ 0x00000112) Memory region 1 write limit */
    __IOM uint16_t INTR_SET;            /*! (@ 0x00000114) Interrupt Set */
    __IOM uint16_t HOST_CLR_EN;         /*! (@ 0x00000116) Host Clear Enable */
} EMI_TypeDef;

/* =========================================================================*/
/* ================           RTC                          ================ */
/* =========================================================================*/

/**
  * @brief Real Time Clock (RTC)
  */
typedef struct
{                           /*!< (@ 0x400F5000) RTC Structure   */
    __IOM uint8_t  RSECONDS;         /*! (@ 0x00000000) Seconds */
    __IOM uint8_t  RSEC_ALARM;       /*! (@ 0x00000001) Seconds Alarm */
    __IOM uint8_t  RMINUTES;         /*! (@ 0x00000002) Minutes */
    __IOM uint8_t  RMIN_ALARM;       /*! (@ 0x00000003) Minutes Alarm */
    __IOM uint8_t  RHOURS;           /*! (@ 0x00000004) Hours */
    __IOM uint8_t  RHOURS_ALARM;     /*! (@ 0x00000005) Hours Alarm */
    __IOM uint8_t  RDAY_OF_WEEK;     /*! (@ 0x00000006) Day of week */
    __IOM uint8_t  RDAY_OF_MONTH;    /*! (@ 0x00000007) Day of month */
    __IOM uint8_t  RMONTH;           /*! (@ 0x00000008) Month */
    __IOM uint8_t  RYEAR;            /*! (@ 0x00000009) Year */
    __IOM uint8_t  REGA;            /*! (@ 0x0000000A) REGA */
    __IOM uint8_t  REGB;            /*! (@ 0x0000000B) REGB */
    __IOM uint8_t  REGC;            /*! (@ 0x0000000C) REGC */
    __IOM uint8_t  REGD;            /*! (@ 0x0000000D) REGD */
            uint8_t  RSVDA[2];
    __IOM uint32_t CONTROL;         /*! (@ 0x00000010) Control */
    __IOM uint32_t WEEK_ALARM;      /*! (@ 0x00000014) Week Alarm */
    __IOM uint32_t DAYLIGHT_SAVF;   /*! (@ 0x00000018) Daylight saving forward */
    __IOM uint32_t DAYLIGHT_SAVB;   /*! (@ 0x0000001C) Daylight saving backward */
} RTC_TypeDef;

/* =========================================================================*/
/* ================           P80CAP                       ================ */
/* =========================================================================*/

/**
  * @brief Port 80h capture (P80CAP)
  */
typedef struct
{                   /*!< (@ 0x400F8000) P80CAP Structure   */
    __IOM uint32_t HOST_DATA;   /*! (@ 0x00000000) host data */
            uint32_t RSVDA[(0x100 - 0x04)/4];
    __IOM uint32_t EC_DATA;     /*! (@ 0x00000100) EC data */
    __IOM uint32_t CONFIG;      /*! (@ 0x00000104) Configuration */
    __IOM uint32_t STATUS;      /*! (@ 0x00000108) Status */
    __IOM uint32_t COUNT;       /*! (@ 0x0000010C) Count */
            uint32_t RSVDB[(0x330 - 0x110)/4];
    __IOM uint32_t ACTIVATE;    /*! (@ 0x00000330) Activate */
} P80CAP_TypeDef;

/* =========================================================================*/
/* ================           ESPI_VW                      ================ */
/* =========================================================================*/

/**
  * @brief eSPI Virtual Wires (ESPI_VW)
  */

#define ESPI_MSVW_MAX       11
#define ESPI_SMVW_MAX       8

/* Master-to-Slave VW byte indices */
#define MSVW_INDEX          0
#define MSVW_MTOS           1
#define MSVW_SRC0_IRQ_SEL   4
#define MSVW_SRC1_IRQ_SEL   5
#define MSVW_SRC2_IRQ_SEL   6
#define MSVW_SRC3_IRQ_SEL   7
#define MSVW_SRC0           8
#define MSVW_SRC1           9
#define MSVW_SRC2           10
#define MSVW_SRC3           11

/* Slave-to-Master VW byte indices */
#define SMVW_INDEX          0
#define SMVW_STOM           1
#define SMVW_CHANGED        2
#define SMVW_SRC0           4
#define SMVW_SRC1           5
#define SMVW_SRC2           6
#define SMVW_SRC3           7

typedef struct
{       /*!< (@ 0x400F9C00) ESPI_VW_M2S Structure   */
    struct {
        union {
            __IOM uint32_t w[3];
            __IOM uint16_t hw[6];
            __IOM uint8_t  b[12];
        };
    } REG96[ESPI_MSVW_MAX];
} ESPI_VW_M2S_TypeDef;

typedef struct
{       /*!< (@ 0x400F9E00) ESPI_VW_S2M Structure   */
    struct {
        __IOM uint32_t w[2];
        __IOM uint16_t hw[4];
        __IOM uint8_t  b[8];
    } REG64[ESPI_SMVW_MAX];
} ESPI_VW_S2M_TypeDef;

/* =========================================================================*/
/* ================           GCFG                         ================ */
/* =========================================================================*/

/**
  * @brief Global Configuration (GCFG)
  */
typedef struct
{                   /*!< (@ 0x400FFF00) GCFG Structure   */
            uint8_t  RSVDA[7];
    __IOM uint8_t  LDN;
            uint8_t  RSVDB[0x20 - 0x08];
    __IOM uint8_t  DEVICE_ID;
    __IOM uint8_t  REVISION;
            uint8_t  RSVDC[2];
} GCFG_TypeDef;

/*@}*/ /* end of group MEC17xx_Peripherals */


/* =========================================  End of section using anonymous unions  ========================================= */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* =========================================================================*/
/* ================ Device Specific Peripheral Address Map ================ */
/* =========================================================================*/

/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

/* Peripheral and SRAM base address */
#define CODE_SRAM_BASE          (0x000B0000UL)    /*!< (FLASH     ) Base Address */
#define DATA_SRAM_BASE          (0x00118000UL)    /*!< (SRAM      ) Base Address */
#define PERIPH_BASE             (0x40000000UL)    /*!< (Peripheral) Base Address */

/* Peripheral memory map */
#define WDT_BASE            (PERIPH_BASE)           /*!< (WDT0   )  Base Address */
#define BTMR0_BASE          (PERIPH_BASE + 0x0C00)  /*!< (Basic Timer 0 16-bit ) Base Address */
#define BTMR1_BASE          (PERIPH_BASE + 0x0C20)  /*!< (Basic Timer 1 16-bit ) Base Address */
#define BTMR2_BASE          (PERIPH_BASE + 0x0C40)  /*!< (Basic Timer 2 16-bit ) Base Address */
#define BTMR3_BASE          (PERIPH_BASE + 0x0C60)  /*!< (Basic Timer 3 16-bit ) Base Address */
#define BTMR4_BASE          (PERIPH_BASE + 0x0C80)  /*!< (Basic Timer 4 32-bit ) Base Address */
#define BTMR5_BASE          (PERIPH_BASE + 0x0CA0)  /*!< (Basic Timer 5 32-bit ) Base Address */
#define EVT0_BASE           (PERIPH_BASE + 0x0D00)  /*!< (EVT0 ) Base Address */
#define EVT1_BASE           (PERIPH_BASE + 0x0D20)  /*!< (EVT1 ) Base Address */
#define EVT2_BASE           (PERIPH_BASE + 0x0D40)  /*!< (EVT2 ) Base Address */
#define EVT3_BASE           (PERIPH_BASE + 0x0D60)  /*!< (EVT3 ) Base Address */
#define CCT_BASE            (PERIPH_BASE + 0x1000)  /*!< (CCT0 ) Base Address */
#define RCID0_BASE          (PERIPH_BASE + 0x1400)  /*!< (RCID0 ) Base Address */
#define RCID1_BASE          (PERIPH_BASE + 0x1480)  /*!< (RCID1 ) Base Address */
#define RCID2_BASE          (PERIPH_BASE + 0x1500)  /*!< (RCID2 ) Base Address */
#define DMA_BASE            (PERIPH_BASE + 0x2400)  /*!< (DMA ) Base Address */
#define DMA_BASE_CHAN(n)    ((DMA_BASE) + 0x40 + ((n)<<6))
#define PROCHOT_BASE        (PERIPH_BASE + 0x3400)  /*!< (PROCHOT ) Base Address */
#define I2CSMB0_BASE        (PERIPH_BASE + 0x4000)  /*!< (I2CSMB0 ) Base Address */
#define I2CSMB1_BASE        (PERIPH_BASE + 0x4400)  /*!< (I2CSMB1 ) Base Address */
#define I2CSMB2_BASE        (PERIPH_BASE + 0x4800)  /*!< (I2CSMB2 ) Base Address */
#define I2CSMB3_BASE        (PERIPH_BASE + 0x4C00)  /*!< (I2CSMB3 ) Base Address */
#define QMSPI_BASE          (PERIPH_BASE + 0x5400)  /*!< (QMSPI0 ) Base Address */
#define PWM0_BASE           (PERIPH_BASE + 0x5800)  /*!< (PWM0 ) Base Address */
#define PWM1_BASE           (PERIPH_BASE + 0x5810)  /*!< (PWM1 ) Base Address */
#define PWM2_BASE           (PERIPH_BASE + 0x5820)  /*!< (PWM2 ) Base Address */
#define PWM3_BASE           (PERIPH_BASE + 0x5830)  /*!< (PWM3 ) Base Address */
#define PWM4_BASE           (PERIPH_BASE + 0x5840)  /*!< (PWM4 ) Base Address */
#define PWM5_BASE           (PERIPH_BASE + 0x5850)  /*!< (PWM5 ) Base Address */
#define PWM6_BASE           (PERIPH_BASE + 0x5860)  /*!< (PWM6 ) Base Address */
#define PWM7_BASE           (PERIPH_BASE + 0x5870)  /*!< (PWM7 ) Base Address */
#define PWM8_BASE           (PERIPH_BASE + 0x5880)  /*!< (PWM8 ) Base Address */
#define PWM9_BASE           (PERIPH_BASE + 0x5890)  /*!< (PWM9 ) Base Address */
#define PWM10_BASE          (PERIPH_BASE + 0x58A0)  /*!< (PWM10 ) Base Address */
#define PWM11_BASE          (PERIPH_BASE + 0x58B0)  /*!< (PWM11 ) Base Address */
#define TACH0_BASE          (PERIPH_BASE + 0x6000)  /*!< (TACH0 ) Base Address */
#define TACH1_BASE          (PERIPH_BASE + 0x6010)  /*!< (TACH1 ) Base Address */
#define TACH2_BASE          (PERIPH_BASE + 0x6020)  /*!< (TACH2 ) Base Address */
#define PECI_BASE           (PERIPH_BASE + 0x6400)  /*!< (PECI ) Base Address */
#define RTMR_BASE           (PERIPH_BASE + 0x7400)  /*!< (RTMR ) Base Address */
#define ADC_BASE            (PERIPH_BASE + 0x7C00)  /*!< (ADC ) Base Address */
#define TFDP_BASE           (PERIPH_BASE + 0x8C00)  /*!< (TFDP ) Base Address */
#define PS2_0_BASE          (PERIPH_BASE + 0x9000)  /*!< (PS2 0 ) Base Address */
#define PS2_1_BASE          (PERIPH_BASE + 0x9040)  /*!< (PS2 1 ) Base Address */
#define PS2_2_BASE          (PERIPH_BASE + 0x9080)  /*!< (PS2 2 ) Base Address */
#define GPSPI0_BASE         (PERIPH_BASE + 0x9400)  /*!< (GPSPI 0 ) Base Address */
#define GPSPI1_BASE         (PERIPH_BASE + 0x9480)  /*!< (GPSPI 1 ) Base Address */
#define HTMR0_BASE          (PERIPH_BASE + 0x9800)  /*!< (HTMR0 ) Base Address */
#define HTMR1_BASE          (PERIPH_BASE + 0x9820)  /*!< (HTMR1 ) Base Address */
#define KSCAN_BASE          (PERIPH_BASE + 0x9C00)  /*!< (KSCAN ) Base Address */
#define RPM2PWM0_BASE       (PERIPH_BASE + 0xA000)  /*!< (RPM2PWM0 ) Base Address */
#define RPM2PWM1_BASE       (PERIPH_BASE + 0xA080)  /*!< (RPM2PWM1 ) Base Address */
#define VBATR_BASE          (PERIPH_BASE + 0xA400)  /*!< (VBATR ) Base Address */
#define VBATM_BASE          (PERIPH_BASE + 0xA800)  /*!< (VBATM ) Base Address */
#define WKTMR_BASE          (PERIPH_BASE + 0xAC80)  /*!< (WKTMR ) Base Address */
#define VCI_BASE            (PERIPH_BASE + 0xAE00)  /*!< (VCI ) Base Address */
#define BBLED0_BASE         (PERIPH_BASE + 0xB800)  /*!< (BBLED0 ) Base Address */
#define BBLED1_BASE         (PERIPH_BASE + 0xB900)  /*!< (BBLED1 ) Base Address */
#define BBLED2_BASE         (PERIPH_BASE + 0xBA00)  /*!< (BBLED2 ) Base Address */
#define BBLED3_BASE         (PERIPH_BASE + 0xBB00)  /*!< (BBLED3 ) Base Address */
#define BCM0_BASE           (PERIPH_BASE + 0xCD00)  /*!< (BCM0 ) Base Address */
#define BCM1_BASE           (PERIPH_BASE + 0xCD20)  /*!< (BCM1 ) Base Address */
#define ECIA_BASE           (PERIPH_BASE + 0xE000)  /*!< (ECIA ) Base Address */
#define ECS_BASE            (PERIPH_BASE + 0xFC00)  /*!< (ECS ) Base Address */
#define PCR_BASE            (PERIPH_BASE + 0x80100)  /*!< (PCR ) Base Address */
#define GPIO_BASE           (PERIPH_BASE + 0x81000)  /*!< (GPIO ) Base Address */
#define GPIO_PARIN_BASE     (PERIPH_BASE + 0x81300)  /*!< (GPIO Control Parallel Input) Base Address */
#define GPIO_PAROUT_BASE    (PERIPH_BASE + 0x81380)  /*!< (GPIO Control Parallel Output) Base Address */
#define GPIO_LOCK_BASE      (PERIPH_BASE + 0x813E8)  /*!< (GPIO Control Lock) Base Address */
#define GPIO_CTRL2_BASE     (PERIPH_BASE + 0x81500)  /*!< (GPIO Control 2) Base Address */
#define EFUSE_BASE          (PERIPH_BASE + 0x82000)  /*!< (EFUSE ) Base Address */
#define MBOX_BASE           (PERIPH_BASE + 0xF0000)  /*!< (MBOX ) Base Address */
#define KBC_BASE            (PERIPH_BASE + 0xF0400)  /*!< (KBC ) Base Address */
#define ACPI_EC0_BASE       (PERIPH_BASE + 0xF0800)  /*!< (ACPI EC0 ) Base Address */
#define ACPI_EC1_BASE       (PERIPH_BASE + 0xF0C00)  /*!< (ACPI EC1 ) Base Address */
#define ACPI_EC2_BASE       (PERIPH_BASE + 0xF1000)  /*!< (ACPI EC2 ) Base Address */
#define ACPI_EC3_BASE       (PERIPH_BASE + 0xF1400)  /*!< (ACPI EC3 ) Base Address */
#define ACPI_EC4_BASE       (PERIPH_BASE + 0xF1800)  /*!< (ACPI EC4 ) Base Address */
#define ACPI_PM1_BASE       (PERIPH_BASE + 0xF1C00)  /*!< (ACPI PM1 ) Base Address */
#define PORT92_BASE         (PERIPH_BASE + 0xF2000)  /*!< (PORT92 ) Base Address */
#define UART0_BASE          (PERIPH_BASE + 0xF2400)  /*!< (UART0 ) Base Address */
#define UART1_BASE          (PERIPH_BASE + 0xF2800)  /*!< (UART1 ) Base Address */
#define LPC_BASE            (PERIPH_BASE + 0xF3000)  /*!< (LPC ) Base Address */
#define ESPI_IO_BASE        (PERIPH_BASE + 0xF3400)  /*!< (ESPI IO Component) Base Address */
#define ESPI_IO_BAR_MLDN_BASE   (ESPI_IO_BASE + 0x0134) /*!< (ESPI IO Component IO BAR Mask & LDN) Base Address */
#define ESPI_IO_BAR_HAV_BASE    (ESPI_IO_BASE + 0x0334) /*!< (ESPI IO Component IO BAR Host Addr & Valid) Base Address */
#define ESPI_IO_PC_BASE     ((ESPI_IO_BASE) + 0x0100) /*!< (ESPI IO Component Peripheral Channel) Base Address */
#define ESPI_IO_LTR_BASE    ((ESPI_IO_BASE) + 0x0220) /*!< (ESPI IO Component LTR) Base Address */
#define ESPI_IO_OOB_BASE    ((ESPI_IO_BASE) + 0x0240) /*!< (ESPI IO Component OOB) Base Address */
#define ESPI_IO_FC_BASE     ((ESPI_IO_BASE) + 0x0280) /*!< (ESPI IO Component Flash Channel) Base Address */
#define ESPI_IO_CAP_BASE    ((ESPI_IO_BASE) + 0x02B0) /*!< (ESPI IO Component Capabilities) Base Address */
#define ESPI_MEM_BASE       (PERIPH_BASE + 0xF3800)  /*!< (ESPI Memory Component) Base Address */
#define ESPI_MEM_BAR_BASE       (ESPI_MEM_BASE + 0x0130) /*!< (ESPI Memory Component BARs) Base Address */
#define ESPI_MEM_BAR_CFG_BASE   (ESPI_MEM_BASE + 0x0330) /*!< (ESPI Memory Component BARs) Config Base Address */
#define ESPI_MEM_BM_BASE    (ESPI_MEM_BASE + 0x0200) /*!< (ESPI Memory Component Bus Master) Base Address */
#define EMI0_BASE           (PERIPH_BASE + 0xF4000)  /*!< (EMI0 ) Base Address */
#define EMI1_BASE           (PERIPH_BASE + 0xF4400)  /*!< (EMI1 ) Base Address */
#define EMI2_BASE           (PERIPH_BASE + 0xF4800)  /*!< (EMI2 ) Base Address */
#define RTC_BASE            (PERIPH_BASE + 0xF5000)  /*!< (RTC ) Base Address */
#define P80CAP0_BASE        (PERIPH_BASE + 0xF8000)  /*!< (P80CAP0 ) Base Address */
#define P80CAP1_BASE        (PERIPH_BASE + 0xF8400)  /*!< (P80CAP1 ) Base Address */
#define ESPI_VW_BASE        (PERIPH_BASE + 0xF9C00)  /*!< (ESPI VW Component) Base Address */
#define ESPI_VW_M2S_BASE    (ESPI_VW_BASE + 0x0000)  /*!< (ESPI VW Component Master-to-Slave) Base Address */
#define ESPI_VW_S2M_BASE    (ESPI_VW_BASE + 0x0200)  /*!< (ESPI VW Component Slave-to-Master) Base Address */
#define GCFG_BASE           (PERIPH_BASE + 0xFFF00)  /*!< (GCFG ) Base Address */


/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define WDT_REGS            ((WDT_TypeDef *) WDT_BASE)
#define BTMR0_REGS          ((BTMR_TypeDef *) BTMR0_BASE)
#define BTMR1_REGS          ((BTMR_TypeDef *) BTMR1_BASE)
#define BTMR2_REGS          ((BTMR_TypeDef *) BTMR2_BASE)
#define BTMR3_REGS          ((BTMR_TypeDef *) BTMR3_BASE)
#define BTMR4_REGS          ((BTMR_TypeDef *) BTMR4_BASE)
#define BTMR5_REGS          ((BTMR_TypeDef *) BTMR5_BASE)
#define EVT0_REGS           ((EVT_TypeDef *) EVT0_BASE)
#define EVT1_REGS           ((EVT_TypeDef *) EVT1_BASE)
#define EVT2_REGS           ((EVT_TypeDef *) EVT2_BASE)
#define EVT3_REGS           ((EVT_TypeDef *) EVT3_BASE)
#define CCT_REGS            ((CCT_TypeDef *) CCT_BASE)
#define RCID0_REGS          ((RCID_TypeDef *) RCID0_BASE)
#define RCID1_REGS          ((RCID_TypeDef *) RCID1_BASE)
#define RCID2_REGS          ((RCID_TypeDef *) RCID2_BASE)
#define DMA_REGS            ((DMA_TypeDef *) DMA_BASE)
#define DMA_CH0_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(0))
#define DMA_CH1_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(1))
#define DMA_CH2_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(2))
#define DMA_CH3_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(3))
#define DMA_CH4_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(4))
#define DMA_CH5_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(5))
#define DMA_CH6_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(6))
#define DMA_CH7_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(7))
#define DMA_CH8_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(8))
#define DMA_CH9_REGS        ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(9))
#define DMA_CH10_REGS       ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(10))
#define DMA_CH11_REGS       ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(11))
#define DMA_CH12_REGS       ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(12))
#define DMA_CH13_REGS       ((DMA_CHAN_TypeDef *) DMA_BASE_CHAN(13))
#define PROCHOT_REGS        ((PROCHOT_TypeDef *) PROCHOT_BASE)
#define I2CSMB0_REGS        ((I2CSMB_TypeDef *) I2CSMB0_BASE)
#define I2CSMB1_REGS        ((I2CSMB_TypeDef *) I2CSMB1_BASE)
#define I2CSMB2_REGS        ((I2CSMB_TypeDef *) I2CSMB2_BASE)
#define I2CSMB3_REGS        ((I2CSMB_TypeDef *) I2CSMB3_BASE)
#define QMSPI_REGS          ((QMSPI_TypeDef *) QMSPI_BASE)
#define PWM0_REGS           ((PWM_TypeDef *) PWM0_BASE)
#define PWM1_REGS           ((PWM_TypeDef *) PWM1_BASE)
#define PWM2_REGS           ((PWM_TypeDef *) PWM2_BASE)
#define PWM3_REGS           ((PWM_TypeDef *) PWM3_BASE)
#define PWM4_REGS           ((PWM_TypeDef *) PWM4_BASE)
#define PWM5_REGS           ((PWM_TypeDef *) PWM5_BASE)
#define PWM6_REGS           ((PWM_TypeDef *) PWM6_BASE)
#define PWM7_REGS           ((PWM_TypeDef *) PWM7_BASE)
#define PWM8_REGS           ((PWM_TypeDef *) PWM8_BASE)
#define PWM9_REGS           ((PWM_TypeDef *) PWM9_BASE)
#define PWM10_REGS          ((PWM_TypeDef *) PWM10_BASE)
#define PWM11_REGS          ((PWM_TypeDef *) PWM11_BASE)
#define TACH0_REGS          ((TACH_TypeDef *) TACH0_BASE)
#define TACH1_REGS          ((TACH_TypeDef *) TACH1_BASE)
#define TACH2_REGS          ((TACH_TypeDef *) TACH2_BASE)
#define RTMR_REGS           ((RTMR_TypeDef *) RTMR_BASE)
#define ADC_REGS            ((ADC_TypeDef *) ADC_BASE)
#define TFDP_REGS           ((TFDP_TypeDef *) TFDP_BASE)
#define PS2_0_REGS          ((PS2_TypeDef *) PS2_0_BASE)
#define PS2_1_REGS          ((PS2_TypeDef *) PS2_1_BASE)
#define PS2_2_REGS          ((PS2_TypeDef *) PS2_2_BASE)
#define GPSPI0_REGS         ((GPSPI_TypeDef *) GPSPI0_BASE)
#define GPSPI1_REGS         ((GPSPI_TypeDef *) GPSPI1_BASE)
#define HTMR0_REGS          ((HTMR_TypeDef *) HTMR0_BASE)
#define HTMR1_REGS          ((HTMR_TypeDef *) HTMR1_BASE)
#define KSCAN_REGS          ((KSCAN_TypeDef *) KSCAN_BASE)
#define RPM2PWM0_REGS       ((RPM2PWM_TypeDef *) RPM2PWM0_BASE)
#define RPM2PWM1_REGS       ((RPM2PWM_TypeDef *) RPM2PWM1_BASE)
#define VBATR_REGS          ((VBATR_TypeDef *) VBATR_BASE)
#define VBATM_REGS          ((VBATM_TypeDef *) VBATM_BASE)
#define WKTMR_REGS          ((WKTMR_TypeDef *) WKTMR_BASE)
#define VCI_REGS            ((VCI_TypeDef *) VCI_BASE)
#define BBLED0_REGS         ((BBLED_TypeDef *) BBLED0_BASE)
#define BBLED1_REGS         ((BBLED_TypeDef *) BBLED1_BASE)
#define BBLED2_REGS         ((BBLED_TypeDef *) BBLED2_BASE)
#define BBLED3_REGS         ((BBLED_TypeDef *) BBLED3_BASE)
#define BCM0_REGS           ((BCM_TypeDef *) BCM0_BASE)
#define BCM1_REGS           ((BCM_TypeDef *) BCM1_BASE)
#define ECIA_REGS           ((ECIA_TypeDef *) ECIA_BASE)
#define ECS_REGS            ((ECS_TypeDef *) ECS_BASE)
#define PCR_REGS            ((PCR_TypeDef *) PCR_BASE)
#define GPIO_REGS           ((GPIO_TypeDef *) GPIO_BASE)
#define EFUSE_REGS          ((EFUSE_TypeDef *) EFUSE_BASE)
#define MBOX_REGS           ((MBOX_TypeDef *) MBOX_BASE)
#define KBC_REGS            ((KBC_TypeDef *) KBC_BASE)
#define ACPI_EC0_REGS       ((ACPI_EC_TypeDef *) ACPI_EC0_BASE)
#define ACPI_EC1_REGS       ((ACPI_EC_TypeDef *) ACPI_EC1_BASE)
#define ACPI_EC2_REGS       ((ACPI_EC_TypeDef *) ACPI_EC2_BASE)
#define ACPI_EC3_REGS       ((ACPI_EC_TypeDef *) ACPI_EC3_BASE)
#define ACPI_EC4_REGS       ((ACPI_EC_TypeDef *) ACPI_EC4_BASE)
#define APM1_REGS           ((ACPI_PM1_TypeDef *) ACPI_PM1_BASE)
#define PORT92_REGS         ((PORT92_TypeDef *) PORT92_BASE)
#define UART0_REGS          ((UART_TypeDef *) UART0_BASE)
#define UART1_REGS          ((UART_TypeDef *) UART1_BASE)
#define LPC_REGS            ((LPC_TypeDef *) LPC_BASE)
#define ESPI_IO_PC_REGS     ((ESPI_IO_PC_TypeDef *)(ESPI_IO_BASE + 0x0100))
#define ESPI_IO_LTR_REGS    ((ESPI_IO_LTR_TypeDef *)(ESPI_IO_BASE + 0x0220))
#define ESPI_IO_OOB_REGS    ((ESPI_IO_OOB_TypeDef *)(ESPI_IO_BASE + 0x0240))
#define ESPI_IO_FC_REGS     ((ESPI_IO_FC_Typedef *)(ESPI_IO_BASE + 0x0280))
#define ESPI_IO_CAP_REGS    ((ESPI_IO_CAP_TypeDef *)(ESPI_IO_BASE + 0x02B0))
#define ESPI_MEM_BM_REGS    ((ESPI_MEM_BM_Typedef *) ESPI_MEM_BM_BASE)
#define ESPI_MEM_BAR_REGS   ((ESPI_MEM_BAR_Typedef *) ESPI_MEM_BAR_BASE)
#define EMI0_REGS           ((EMI_TypeDef *) EMI0_BASE)
#define EMI1_REGS           ((EMI_TypeDef *) EMI1_BASE)
#define EMI2_REGS           ((EMI_TypeDef *) EMI2_BASE)
#define RTC_REGS            ((RTC_TypeDef *) RTC_BASE)
#define P80CAP0_REGS        ((P80CAP_TypeDef *) P80CAP0_BASE)
#define P80CAP1_REGS        ((P80CAP_TypeDef *) P80CAP1_BASE)
#define ESPI_VW_M2S_REGS    ((ESPI_VW_M2S_TypeDef *) ESPI_VW_M2S_BASE)
#define ESPI_VW_S2M_REGS    ((ESPI_VW_S2M_TypeDef *) ESPI_VW_S2M_BASE)
#define GCFG_REGS           ((GCFG_TypeDef *) GCFG_BASE)

/** @} */ /* End of group MEC17xx */

/** @} */ /* End of group MCHP */

#ifdef __cplusplus
}
#endif

#endif  /* MCHP_MEC1701_H */
