/*******************************************************************************
  Hardware specific routine definition and interfaces Header File

  File Name:
    sys_config.h

  Summary:
    This header file lists hardware specific initialisations for dsPIC DSC
    modules ADC,PWM,OP-AMP,Comparator and PPS and GPIO pins

  Description:
    Definitions in the file are for dsPIC33EVGM106 PIM plugged onto
    Low-Voltage Motor Control Development board bundle from Microchip
Bundle
*******************************************************************************/
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
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
#define DIFF_AMP_GAIN			7.5
// maximum current (peak to peak) that can be read by adc with the shunt/opamp settings above	
#define IPEAK                   22.0      


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