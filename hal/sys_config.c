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
#include <xc.h>
#include <stdint.h>
#include "sys_config.h"

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

    // Configure Port pins for Motor Current Snesing
	
    // IPHASE2_A/IB_A(AN0) : PIM #42
    TRISAbits.TRISA0 = 1;    // AN0/OA2OUT/RA0
    ANSELAbits.ANSA0 = 1;

    // IPHASE1_A/IA_A(AN1) : PIM #41
    TRISAbits.TRISA1 = 1;    // AN1/C2IN1+/RA1
    ANSELAbits.ANSA1 = 1;

    // IBUS_A(AN2) : PIM #43
    TRISBbits.TRISB0 = 1;    // PGED3/VREF-/AN2/C2IN1-/SS1/RPI32/CTED2/RB0
    ANSELBbits.ANSB0 = 1;

    // Potentiometer #1 input - used as Speed Reference
    // POT1 : PIM #32
    TRISEbits.TRISE13 = 1;          // AN13/C3IN2-/U2CTS/RE13
    ANSELEbits.ANSE13 = 1;

    // Inverter DC bus voltage Sensing
    // VBUS_A : PIM #35
    TRISAbits.TRISA12 = 1;          // AN10/RPI28/RA12
    ANSELAbits.ANSA12 = 1;

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
    
    // FAULT Pins
    // FAULT : PIM #18
    TRISBbits.TRISB4 = 1;            // FLT32/SCL2/RP36/RB4

    // Hall inputs - Inverter 
    // HALLA : PIM #80
    TRISAbits.TRISA8 = 1;            // SDA2/RPI24/RA8
    // HALLB: PIM #47
    TRISCbits.TRISC6 = 1;            // RP54/RC6
    // HALLC : PIM #48
    TRISFbits.TRISF0 = 1;            // RPI96/RF0
    // HOME: PIM #61
    TRISCbits.TRISC10 = 1;           // RPI58/RC10

    // Debug LEDs
    // LED1 : PIM #01
    TRISDbits.TRISD5 = 0;           // AN56/RA10
    // LED2 : PIM #60
    TRISDbits.TRISD6 = 0;           // RPI72/RD8

    // Push button Switches
    // SW1 : PIM #83
    TRISGbits.TRISG7 = 1;            // AN30/CVREF+/RPI52/RC4
    // SW2 : PIM #84
    TRISGbits.TRISG6 = 1;            // AN19/RP118/RG6

    // UART - for RTDM/DMCI Communication
    // UART_RX : PIM #49 (Input)
    TRISCbits.TRISC5 = 1;            // SCL1/RPI53/RC5
    // UART_TX : PIM #50(Output)
    TRISFbits.TRISF1 = 0;            // RP97/RF1

    /************************** Remappable PIn configuration ******************/

    //Unlock registers by clearing IOLOCK bit OSCCON(OSCONbits.IOLOCK = 0)
    __builtin_write_OSCCONL(OSCCON & (~(1 << 6))); 

    // RTDM Communication RX and TX configuration ( UART #1)
    // UART_RX : PIM #49 (Input)
    // Configure RP53 as U1RX
    _U1RXR = 53;         // SCL1/RPI53/RC5
    // UART_TX : PIM #50 (Output)
    // Remap RP53 as U1RX
    _RP97R = 0x01;                  // RP97/RF1

    // Lock registers by setting IOLOCK bit OSCCON(OSCONbits.IOLOCK = 1)
    __builtin_write_OSCCONL(OSCCON | (1 << 6)); // Set bit 6

    /**************************************************************************/

    return;
}

/*EOF*/
