/*******************************************************************************
  Hardware specific routine definition and interfaces Header File

  File Name:
    sys_config.h

  Summary:
    This header file lists hardware specific initialisations for dsPIC DSC
    modules ADC,PWM,OP-AMP,Comparator and PPS and GPIO pins

  Description:
    Definitions in the file are for dsPIC33EDV64MC205 Motor Control Development
 *  board from Microchip
*******************************************************************************/
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
#ifndef _SYSCONFIG_H
#define _SYSCONFIG_H

#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// Digital I/O definitions
// Push button Switches
// SW1 : PIM #83 (RG7)
#define S2                  !PORTCbits.RC6
// SW2 : PIM #84 (RG6)
#define S3                  !PORTGbits.RG8

// SW1 : PIM #69 - Used as START/STOP button of Motor A
#define BUTTON_START_STOP          S2
// SW2 : PIM #84 - Used as Speed HALF/DOUBLE button of Motor A
#define BUTTON_SPEED_HALF_DOUBLE   S3

// Debug LEDs
// LED1 : PIM #01(RD6)
#define LED1                    LATBbits.LATB7
// LED2 : PIM #60(RD5)
#define LED2                    LATAbits.LATA4

// Value in Ohms of shunt resistors used.        
#define RSHUNT					0.010	
// Gain of differential amplifier.
#define DIFF_AMP_GAIN			37.5
// maximum current (peak to peak) that can be read by adc with the shunt/opamp settings above	
#define IPEAK                   4.4      


// MC PWM MODULE Related Definitions
#define INVERTERA_PWM_PDC1      PDC1
#define INVERTERA_PWM_PDC2      PDC2
#define INVERTERA_PWM_PDC3      PDC3

// ADC MODULE Related Definitions
// Analog Channel No of Potentiometer #1 - used as Speed Reference
// POT1 : PIM #32 (AN13)
#define ANx_POT_1               0
// Analog Channel No of Inverter A DC bus voltage VDC_A
// VBUS_A : PIM #35 (AN10)
#define ANx_VBUS_A              1

// CH123SA = 1 is M1_IB = OA1/AN3 POT1 = OA2/AN0 M1_IA = OA3/AN6
#define ADCBUF_SPEED_REF_A      ADC1BUF2
#define ADCBUF_INV_A_IPHASE1    ADC1BUF1
#define ADCBUF_INV_A_IPHASE2    ADC1BUF3
#define ADCBUF_INV_A_IBUS       ADC1BUF0

// Specify bootstrap charging time in no of us
#define BOOTSTRAP_CHARGING_TIME 30000
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void MapGPIOHWFuntion(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of SYSCONFIG_H


/* EOF */