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
#include <xc.h>
#include <stdint.h>
#include "sys_config.h"
void configureGateDriverFault(void);
// *****************************************************************************
/* Function:
    Map_GPIO_HW_Funtion()

  Summary:
    Routine to setup GPIO pin used as input/output analog/digital etc

  Description:
    Function initialises GPIO pins as input or output port pins,analog/digital 
    pins,remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void MapGPIOHWFuntion(void)
{
 
    /* ANALOG SIGNALS */

    // Configure Port pins for Motor Current Sensing
	
    // IPHASE2_A/IB_A(AN0) : PIM #42
    TRISCbits.TRISC0 = 1;    // AN6/OA3OUT/RC0
    ANSELCbits.ANSC0 = 1;
    TRISCbits.TRISC1 = 1;    // AN7/C3IN1-/RC1
    ANSELCbits.ANSC1 = 1;
    TRISCbits.TRISC2 = 1;    // AN8/C3IN1+/RC2
    ANSELCbits.ANSC2 = 1;

    // IPHASE1_A/IA_A(AN1) : PIM #41
    TRISBbits.TRISB1 = 1;    // AN3//OA1OUT/RB1
    ANSELBbits.ANSB1 = 1;
    TRISBbits.TRISB2 = 1;    // AN4/C1IN1+/RB2
    ANSELBbits.ANSB2 = 1;
    TRISBbits.TRISB3 = 1;    // AN5/C1IN1-/RB3
    ANSELBbits.ANSB3 = 1;

    // IBUS_A(AN2) : PIM #43
    TRISBbits.TRISB0 = 1;    // PGED3/VREF-/AN2/C2IN1-/SS1/RPI32/CTED2/RB0
    ANSELBbits.ANSB0 = 1;

    // Potentiometer #1 input - used as Speed Reference
    // POT1 : PIM #32
    TRISAbits.TRISA0 = 1;    // AN0/OA2OUT/RA0
    ANSELAbits.ANSA0 = 1;

    // Inverter DC bus voltage Sensing
    // VBUS_A : PIM #35
    TRISAbits.TRISA1 = 1;    // AN1/C2IN1+/RA1
    ANSELAbits.ANSA1 = 1;

    // DIGITAL INPUT/OUTPUT PINS

    // Inverter Control - PWM 0utputs
    // PWM1L : PIM #93  (RPI47/PWM1L1/T5CK/RB15)
    // PWM1H : PIM #94  (RPI46/PWM1H1/T3CK/RB14)
    // PWM2L : PIM #98  (RP45/PWM1L2/CTPLS/RB13)
    // PWM2H : PIM #99  (RP44/PWM1H2/RB12)
    // PWM3L : PIM #100 (RP43/PWM1L3/RB11)
    // PWM3H : PIM #03  (RP42/PWM1H3/RB10)
    TRISB = (TRISB & 0x03FF);        // 0b0000 00XX XXXX XXXX
    LATB  = (LATB & 0x03FF);         // Setting all PWM outputs as 'LOW'
    
    // Debug LEDs
    // LED1 : PIM #01
    TRISBbits.TRISB7 = 0;           // AN56/RA10
    // LED2 : PIM #60
    TRISAbits.TRISA4 = 0;           // RPI72/RD8

    // Push button Switches
    // SW1 : PIM #83
    TRISCbits.TRISC6 = 1;            // AN30/CVREF+/RPI52/RC4
    // SW2 : PIM #84
    TRISGbits.TRISG8 = 1;            // AN19/RP118/RG6

    // UART - for RTDM/DMCI Communication
    TRISAbits.TRISA8 = 1;               // UART1 RX input - SDA2/RPI24/RA8
    TRISBbits.TRISB4 = 0;               // UART1 TX output - FLT32/SCL2/RP36/RB4
 

    _U1RXR = 24;                // UART1 RX - RP24
    _RP36R = 1;                 // UART1 TX - RP36

    
    _CNPUG6 = 1;
    _CNPUA7 = 1;
    _CNPUA10 = 1;
    TRISAbits.TRISA7 = 0;
    
     configureGateDriverFault();
    _U2RXR = 118;
    _RP118R = 3;

}
void configureGateDriverFault(void)
{
    _CNPUA10 = 1;
    
    TRISAbits.TRISA10 = 1;
    // Enable RA10 pin for interrupt detection
    CNENAbits.CNIEA10 = 1; 
    // Enable CN interrupts
    IEC1bits.CNIE = 1; 
    // Reset CN interrupt
    IFS1bits.CNIF = 0; 
   _CNIP  = 7;
}