/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
#ifndef __PERIPH_H
#define __PERIPH_H

#include <stdint.h>
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "userparms.h"
#include "sys_config.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// OSCILLATOR Related Definitions
#define OSC_FRC                 0               // Internal RC Oscllator
#define OSC_XTAL                1               // Primary oscillator - XTAL
// Select Oscillator Mode
#define OSC_MODE                OSC_FRC
// Instruction cycle frequency (Hz)
#define FCY_MHZ                 (uint32_t)70
// Instruction cycle frequency (MHz)
#define FCY_HZ                  (uint32_t)(FCY_MHZ*1000000)
// Instruction cycle period (sec)
#define TCY_SEC                 (1.0/FCY_HZ)

// DEVICE RELATED SPECIFICATIONS
// Specify device operative voltage VDD in volts
#define DEVICE_VDD_VOLTS        3.3
// Specify device analog supply voltage VDD in volts
#define DEVICE_AVDD_VOLTS       DEVICE_VDD_VOLTS

// Digital I/O definitions
// MC PWM MODULE Related Definitions
// Calculating dead time in units of Tcy(Center aligned Mode)
#define PWM_DT_ERRATA

#define DDEADTIME               (uint16_t)(2*DEADTIME_MICROSEC*FCY_MHZ)
// Basic loop period in units of Tcy
#define LOOPTIME_TCY            (uint16_t)(FCY_HZ/PWMFREQUENCY_HZ)

#ifdef PWM_DT_ERRATA
// Should be >= DDEADTIME/2 for PWM Errata workaround
    #define MIN_DUTY            (uint16_t)(DDEADTIME/2 + 1)
#else
    #define MIN_DUTY            0x0000
#endif

// OPAMP/CMP MODULE Related Defintions
// Comaprator Voltage Reference 
#define CVREF_1    				0xF

// ADC MODULE Related Defintions
// Specify Minimum ADC Clock Period (TAD)in uSec
#define ADC_MIN_TAD_MICROSEC     0.1
// Specify Max Time to stabilize Analog Stage from ADC OFF to ADC ON in uSecs
// The parameter, tDPU, is the time required for the ADC module to stabilize at
// the appropriate level when the module is turned on (AD1CON1<ADON> = 1).
// During this time, the ADC result is indeterminate.
#define ADC_TON_DELAY_MICROSEC  300

// Definitions for Channels 1, 2, 3 Positive Input Select bits
/* 001 = CH1 positive input is OA1(AN3),
         CH2 positive input is OA2(AN0),
         CH3 positive input is OA3(AN6) */
#define CH123_IS_OA1_OA2_OA3    0x01
/* 000 = CH1 positive input is AN0,
         CH2 positive input is AN1,
         CH3 positive input is AN2 */
#define CH123_IS_AN0_AN1_AN2    0x00

// Setting Channel No connected to ADC1 Sample/Hold Channel #0(ADC1-CH0)
// IBUS is connected for sample/conversion by ADC1 CH0
#define ADC1_ANx_CH0            2
// Setting Channels to be connected to ADC1 Sample/Hold Channels 1,2,3
// for simultaneous sampling  : AN0(IB),AN1(IA),AN2(IBUS)
#define ADC1_ANx_CH123          CH123_IS_OA1_OA2_OA3
// Calculating  ADC conversion clock count ADCS from Min ADC Clock Period (TAD)
// TAD = Tp*(ADCS<7:0> + 1)= (1/Fp)*(ADCS<7:0> + 1)
// ADCS<7:0> = (MIN_TAD * Fp ) - 1 ~ (MIN_TAD * Fp )
// Subtraction by 1 is ignored as Min TAD cycle has to be met
#define ADC_MIN_ADCS_COUNTS     (uint8_t)((ADC_MIN_TAD_MICROSEC * FCY_MHZ))

#define EnableADCInterrupt()   IEC0bits.AD1IE = 1
#define DisableADCInterrupt()  IEC0bits.AD1IE = 0
#define ClearADCIF()           IFS0bits.AD1IF = 0
#define _ADCInterrupt _AD1Interrupt        
        
/* This defines number of current offset samples for averaging 
 * If the 2^n samples are considered specify n(in this case 2^7(= 128)=> 7*/
#define  CURRENT_OFFSET_SAMPLE_SCALER         7 
// Clear CN interrupt         
#define ClearCNIF() IFS1bits.CNIF = 0       
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* ADC initial offset storage data type

  Description:
    This structure will host parameters related to ADC module offset for all
    four sample and hold channels.
*/
typedef struct
{
    // ch0 component
    int16_t ch0;

    // ch1 component
    int16_t ch1;

    // ch2 component
    int16_t ch2;

    // ch3 component
    int16_t ch3;
} ADC_OFFSET_T;
extern ADC_OFFSET_T adcOffset;
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitOscillator(void);
void InitADCModule1(ADC_OFFSET_T *);
void InitAmplifiersComparator(void);
void InitPWMGenerators(void);
void SetupGPIOPorts(void);
void InitPeripherals(void);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_H */
