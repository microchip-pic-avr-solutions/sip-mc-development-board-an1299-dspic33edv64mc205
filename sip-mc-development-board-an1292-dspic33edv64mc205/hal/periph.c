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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "periph.h"
#include "sys_config.h"
#include "delay.h"
// *****************************************************************************
// *****************************************************************************
// Section: Configuration bits
// *****************************************************************************
// *****************************************************************************

// DSPIC33EP256MC506 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// FICD
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#if (OSC_MODE == OSC_XTAL)
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#elif(OSC_MODE == OSC_FRC)
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#endif

#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF            // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************
int32_t iADCOffsetCH0 = 0, iADCOffsetCH1 = 0,
        iADCOffsetCH2 = 0, iADCOffsetCH3 = 0;
ADC_OFFSET_T adcOffset;
// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void SetupGPIOPorts(void);
void InitOscillator(void);
void InitPWM123Generators(void);
void InitADCModule1(ADC_OFFSET_T *);
void InitAmplifiersComparator(void);
void InitPWMGenerators(void);
void ChargeBootstarpCapacitors(void);

// *****************************************************************************
/* Function:
    SetupGPIOPorts()

  Summary:
    Routine to set-up GPIO ports

  Description:
    Function initialises GPIO pins for input or output ports,analog/digital pins,
    remap the peripheral functions to desires RPx pins.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

void SetupGPIOPorts(void)
{
    // Reset all PORTx register (all inputs)
    #ifdef TRISA
        TRISA = 0x1F93;             // 0bUUU1 1111 1UU1 UU11
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;            // 0bUUU0 0DDD DUU0 UU00
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;             // 0b1111 1111 1111 1111
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;            // 0bDDDD DDD0 DDDD 0000
    #endif

    #ifdef TRISC
        TRISC = 0xBFFF;             // 0b1U11 1111 1111 1111
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;            // 0bDUDD 0DDD DDDD D000
    #endif

    #ifdef TRISD
        TRISD = 0x0160;             // 0bUUUU UUU1 U11U UUUU
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;            // 0bUUUU UUUU UUUU UUUU
    #endif

    #ifdef TRISE
        TRISE = 0xF000;             // 0b1111 UUUU UUUU UUUU
        LATE  = 0x0000;
    #endif
    #ifdef ANSELE
        ANSELE = 0x0000;            // 0b0000 UUUU UUUU UUUU
    #endif

    #ifdef TRISF
        TRISF = 0x0003;             // 0bUUUU UUUU UUUU UU11
        LATF  = 0x0000;
    #endif
    #ifdef ANSELF
        ANSELF = 0x0000;            // 0bUUUU UUUU UUUU UUUU
    #endif

    #ifdef TRISG
        TRISG = 0x03C0;             // 0bUUUU UU11 11UU UUUU 
        LATG  = 0x0000;
    #endif
    #ifdef ANSELG
        ANSELG = 0x0000;            // 0bUUUU UUUU UUUU UUUU 
    #endif

    MapGPIOHWFuntion();

}

// *****************************************************************************
/* Function:
    void InitOscillator(void)

  Summary:
    Routine to configure controller oscillator

  Description:
    Function configure oscillator PLL to generate desired processor clock

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitOscillator(void)

{
    /* Below equation provides relation between input frequency FIN(in MHz)and
     * Oscillator output frequency FOSC(in MHz)
     * Device Operating Frequency FCY(in MHz)is FOSC /2
     *
     *                    M                 (PLLDIV + 2)
     * FOSC  =  FIN * ---------- = -----------------------------------
     *                 (N1 * N2)    ((PLLPRE + 2)*(2 x (PLLPOST + 1)))
     *
     * where,
     * N1 = PLLPRE + 2
     * N2 = 2 x (PLLPOST + 1)
     * M = PLLDIV + 2
     */

#if (OSC_MODE == OSC_XTAL)

    /* If FIN (input frequency) = 8 MHz
     * FOSC (Oscillator output frequency)and FCY (Device Operating Frequency)is:
     *
     *                 M                   (68 + 2)            8 * 70
     * FOSC = FIN * --------- = 8 * ----------------------- = ------- = 140 MHz
     *              (N1 * N2)       ((0 + 2)*(2 x (0 + 1)))    2 * 2
     *
     * FCY = 140 MHz / 2 =  70 MHz
     *
     * where,
     * N1 = PLLPRE + 2 = 0 + 2 = 0
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2
     * M = PLLDIV + 2 = 68 + 2 = 70
     */

    /* PLLFBD: PLL FEEDBACK DIVISOR REGISTER(denoted as M,PLL multiplier)
     * M = (PLLFBDbits.PLLDIV+2)= (68+2) = 70                                 */
    PLLFBD = 68;

    /* PLL VCO Output Divider Select bits(denoted as N2, PLL post-scaler
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2                                   */
    CLKDIVbits.PLLPOST = 0;

    /* PLL Phase Detector I/P Divider Select bits(denoted as N1,PLL pre-scaler)
     * N1 = PLLPRE + 2 = 0 + 2 = 0                                            */
    CLKDIVbits.PLLPRE = 0;

    /* Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
     *  NOSC = 0b011 = Primary Oscillator with PLL (XTPLL, HSPLL, ECPLL)      */
    __builtin_write_OSCCONH(0x03);

    /* Request oscillator switch to selection specified by the NOSC<2:0>bits  */
    __builtin_write_OSCCONL(0x01);

    /* Wait for Clock switch to occur */
    while (OSCCONbits.COSC != 0b011);

    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1);

#elif(OSC_MODE == OSC_FRC)

    /* If FIN (input frequency) = 7.37 MHz Internal RC Oscillator
     * FOSC (Oscillator output frequency)and FCY (Device Operating Frequency)is:
     *
     *         FIN * M         7.37 * (74 + 2)         7.37 * 76
     * FOSC = --------- = -----------------------  = ------------ = 140.03 MHz
     *        (N1 * N2)     ((0 + 2)*(2 x (0 + 1)))    (2 * 2)
     *
     * FCY = 140.03 MHz / 2 =  70.015 MHz
     *
     * where,
     * N1 = PLLPRE + 2 = 0 + 2 = 0
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2
     * M = PLLDIV + 2 = 74 + 2 = 76
     */

    /* PLLFBD: PLL FEEDBACK DIVISOR REGISTER(denoted as M,PLL multiplier)
     * M = (PLLFBDbits.PLLDIV+2)= (74+2) = 76                                 */
    PLLFBD = 74;

    /* PLL VCO Output Divider Select bits(denoted as N2, PLL post-scaler
     * N2 = 2 * (PLLPOST + 1)= 2 * (0+1)= 2                                   */
    CLKDIVbits.PLLPOST = 0;

    /* PLL Phase Detector I/P Divider Select bits(denoted as N1,PLL pre-scaler)
     * N1 = PLLPRE + 2 = 0 + 2 = 0                                            */
    CLKDIVbits.PLLPRE = 0;

    /* Initiate Clock Switch to FRC Oscillator with PLL (NOSC=0b001)
     *  NOSC = 0b001 = Fast RC Oscillator with PLL (FRCPLL)                   */
    __builtin_write_OSCCONH(0x01);

    /* Request oscillator switch to selection specified by the NOSC<2:0>bits  */
    __builtin_write_OSCCONL(0x01);

    /* Wait for Clock switch to occur */
    while (OSCCONbits.COSC != 0b001);

    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1);

#endif
}

// *****************************************************************************
/* Function:
    InitAmplifiersComparator()

  Summary:
    Routine to initialise Op-Amps and Comparator

  Description:
    Function initialise Op-Amps for motor current amplification;comparators for
    over current detection

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitAmplifiersComparator(void)
{
     // Initialise OPAMP/Comparator 
    // OpAmp 1,2,3 for Signal Conditioning Currents & Comparator 4 for Fault Generation

                             // C1IN1+ :IBUS+ ; C1IN1- : IBUS- 
    CM1CON = 0x8C00;	     // OpAmp1- OA1OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C1IN1- pin
                             // C2IN1+ :IA+ ; C2IN1- : IBUS+ 
    CM3CON = 0x8C00;		// OpAmp3- OA3OUT is connected to pin,operates as an Op amp,Inverting I/P of Op amp is  C3IN1- pin
   
    // OP-AMP/COMPARATOR 2 CONTROL REGISTER
    // Not used int this application
    CM2CON = 0xC010;            // 0b000U U000 00U0 UU00 ;
    CM2CONbits.CCH = 0;
    CM2FLTR = 0x000F;
    CVRCON = 0x008F;        // CVREF = (0.1031)*CVR + 0.825, CVR = 15

}

// *****************************************************************************
/* Function:
    InitPWMGenerators()

  Summary:
    Routine to initialise PWM Module for Inverters

  Description:
    Function initialises  and enable the PWM Module after configuration

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPWMGenerators(void)
{
    // PTCON: PWMx TIME BASE CONTROL REGISTER
    // PTEN(15): PWMx Module Enable bit
    // Unimplemented(14): Read as ?0?
    // PTSIDL(13): PWMx Time Base Stop in Idle Mode bit
    // SESTAT(12): Special Event Interrupt Status bit
    // SEIEN(11): Special Event Interrupt Enable bit
    // EIPU(1): Enable Immediate Period Updates bit(1)
    // SYNCPOL(9): Synchronize Input and Output Polarity bit(1)
    // SYNCOEN(8): Primary Time Base Sync Enable bit(1)
    // SYNCEN(7): External Time Base Synchronization Enable bit
    // SYNCSRC<(6-4): Synchronous Source Selection bits(1)
    // SEVTPS(3-0): PWMx Special Event Trigger Output Postscaler Select bits
    PTCON = 0;               // 0b0000 0000 0000 0000(PWM Module is disabled)
    PTCONbits.EIPU = 0;      // 0 = Active Period register updates occur
                             // on PWMx cycle boundaries

    // PTCON2: PWMx PRIMARY MASTER CLOCK DIVIDER SELECT REGISTER
    // Unimplemented(15-3): Read as ?0?
    // PCLKDIV(2-0): PWMx Input Clock Prescaler (Divider) Select bits
    PTCON2 = 0;              // 0b0000 0000 0000 0000(PWM Module is disabled)
    PTCON2bits.PCLKDIV = 0;  // 0b000 = Divide-by-1, max PWM timing resolution

    InitPWM123Generators();

    ChargeBootstarpCapacitors();
    
    IOCON1 = 0xC301;
    IOCON2 = 0xC301;
    IOCON3 = 0xC301;

    PTCONbits.PTEN = 1;      // Enable PWM module after initialising generators
    

    return;
}

// *****************************************************************************
/* Function:
    InitPWM123Generators()

  Summary:
    Routine to initialise PWM generators 1,2,3 

  Description:
    Function initialises PWM module for 3-phase inverter control in Complimentary
    mode ;initialises period,dead time;Configures PWM fault control logic

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPWM123Generators(void)
{
    // PWMCONx: PWMx CONTROL REGISTER
    // FLTSTAT(15): Fault Interrupt Status bit
    // CLSTAT(14): Current-Limit Interrupt Status bit
    // TRGSTAT(13): Trigger Interrupt Status bit
    // FLTIEN(12): Fault Interrupt Enable bit
    // CLIEN(11): Current-Limit Interrupt Enable bit
    // TRGIEN(10): Trigger Interrupt Enable bit
    // ITB(9): Independent Time Base Mode bit
    // MDCS(8): Master Duty Cycle Register Select bit
    // DTC(7-6): Dead-Time Control bits
    // DTCP(5): Dead-Time Compensation Polarity bit
    // Unimplemented(4-3): Read as ?0?
    // CAM(2): Centrer-Aligned Mode Enable bit
    // XPRES(1): External PWMx Reset Control bit
    // IUE(0): Immediate Update Enable bit
    PWMCON1 = 0;                // 0b0000 0000 0000 0000
    PWMCON2 = 0;                // 0b0000 0000 0000 0000
    PWMCON3 = 0;                // 0b0000 0000 0000 0000

    PWMCON1bits.FLTIEN = 0;     // 0 = Fault interrupt is disabled
    PWMCON2bits.FLTIEN = 0;     // 0 = Fault interrupt is disabled
    PWMCON3bits.FLTIEN = 0;     // 0 = Fault interrupt is disabled

    PWMCON1bits.CLIEN = 0;      // 0 = Current-limit interrupt is disabled
    PWMCON2bits.CLIEN = 0;      // 0 = Current-limit interrupt is disabled
    PWMCON3bits.CLIEN = 0;      // 0 = Current-limit interrupt is disabled

    PWMCON1bits.TRGIEN = 0;     // 0 = Trigger event interrupts are disabled
    PWMCON2bits.TRGIEN = 0;     // 0 = Trigger event interrupts are disabled
    PWMCON3bits.TRGIEN = 0;     // 0 = Trigger event interrupts are disabled

    PWMCON1bits.ITB = 1;        // 1 = PHASE1 register provides time base period
    PWMCON2bits.ITB = 1;        // 1 = PHASE2 register provides time base period
    PWMCON3bits.ITB = 1;        // 1 = PHASE3 register provides time base period

    PWMCON1bits.MDCS = 0;       // 0 = PDC1 register provides duty cycle
    PWMCON2bits.MDCS = 0;       // 0 = PDC2 register provides duty cycle
    PWMCON3bits.MDCS = 0;       // 0 = PDC3 register provides duty cycle

    PWMCON1bits.DTC = 0;        // 00 = Positive dead time is actively applied
    PWMCON2bits.DTC = 0;        // 00 = Positive dead time is actively applied
    PWMCON3bits.DTC = 0;        // 00 = Positive dead time is actively applied

    PWMCON1bits.CAM = 1;        // 1 = Centre-Aligned mode is enabled
    PWMCON2bits.CAM = 1;        // 1 = Centre-Aligned mode is enabled
    PWMCON3bits.CAM = 1;        // 1 = Centre-Aligned mode is enabled

    PWMCON1bits.IUE = 0;        // 0 = Updates to the active
                                // MDC/PDC1/DTR1/ALTDTR1/PHASE1 registers are
                                // synchronized to the PWM1 period boundary
    PWMCON1bits.IUE = 0;        // 0 = Updates to the active
                                // MDC/PDC2/DTR2/ALTDTR2/PHASE2 registers are
                                // synchronized to the PWM2 period boundary
    PWMCON1bits.IUE = 0;        // 0 = Updates to the active
                                // MDC/PDC3/DTR3/ALTDTR3/PHASE3 registers are
                                // synchronized to the PWM3 period boundary

    // PHASEx: PWMx PRIMARY PHASE-SHIFT REGISTER
    // PHASEx register provides time base period, if PWMCONxbits.ITB = 1
    PHASE1 = LOOPTIME_TCY;      // Setting PWM period for PWM1 generator
    PHASE2 = LOOPTIME_TCY;      // Setting PWM period for PWM2 generator
    PHASE3 = LOOPTIME_TCY;      // Setting PWM period for PWM3 generator
    
    // DTRx: PWMx DEAD-TIME REGISTER
    DTR1 = 0x0000;              // Not used in centre aligned mode
    DTR2 = 0x0000;              // Not used in centre aligned mode
    DTR3 = 0x0000;              // Not used in centre aligned mode

    // ALTDTRx: PWMx ALTERNATE DEAD-TIME REGISTER
    // Provides 14-Bit Dead-Time Value for PWMx generator if PWMCONxbits.CAM = 1
    ALTDTR1 = DDEADTIME;        // Setting dead-time for PWM1 generator
    ALTDTR2 = DDEADTIME;        // Setting dead-time for PWM2 generator
    ALTDTR3 = DDEADTIME;        // Setting dead-time for PWM3 generator

    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Initialise the PWM duty cycle register
    PDC1 = MIN_DUTY;
    PDC2 = MIN_DUTY;
    PDC3 = MIN_DUTY;

    // IOCONx: PWMx I/O CONTROL REGISTER
    // PENH(15): PWMxH Output Pin Ownership bit
    // PENL(14): PWMxL Output Pin Ownership bit
    // POLH(13): PWMxH Output Pin Polarity bit
    // POLL(12): PWMxL Output Pin Polarity bit
    // PMOD(11:10): PWMx I/O Pin Mode bits
    // OVRENH(9): Override Enable for PWMxH Pin bit
    // OVRENL(8): Override Enable for PWMxL Pin bit
    // OVRDAT(7:6): Data for PWMxH, PWMxL Pins if Override is Enabled bits
    // FLTDAT(5:4): Data for PWMxH and PWMxL Pins if FLTMOD is Enabled bits.
    // CLDAT(1:0): Data for PWMxH and PWMxL Pins if CLMOD is Enabled bits
    // SWAP(1): SWAP PWMxH and PWMxL Pins bit
    // OSYNC(1): Output Override Synchronization bit
    IOCON1 = 0x0000;        // 0b0000 0000 0000 0000(GPIO controls PWM1H,L pins)
    IOCON2 = 0x0000;        // 0b0000 0000 0000 0000(GPIO controls PWM2H,L pins)
    IOCON3 = 0x0000;        // 0b0000 0000 0000 0000(GPIO controls PWM3H,L pins)
    
    IOCON1bits.PENH = 1;    // 1 = PWM1H pin is active-high
    IOCON2bits.PENH = 1;    // 1 = PWM2H pin is active-high
    IOCON3bits.PENH = 1;    // 1 = PWM3H pin is active-high
    
    IOCON1bits.PENL = 1;    // 1 = PWM1L pin is active-high
    IOCON2bits.PENL = 1;    // 1 = PWM2L pin is active-high
    IOCON3bits.PENL = 1;    // 1 = PWM3L pin is active-high
    
    IOCON1bits.POLH = 0;    // 0 = PWM1H pin is active-high
    IOCON2bits.POLH = 0;    // 0 = PWM2H pin is active-high
    IOCON3bits.POLH = 0;    // 0 = PWM3H pin is active-high
    
    IOCON1bits.POLL = 0;    // 0 = PWM1L pin is active-high
    IOCON2bits.POLL = 0;    // 0 = PWM2L pin is active-high
    IOCON3bits.POLL = 0;    // 0 = PWM3L pin is active-high
   
    IOCON1bits.FLTDAT = 0;  // 0b00 = If Fault is active, PWM1H,L is driven LOW
    IOCON2bits.FLTDAT = 0;  // 0b00 = If Fault is active, PWM2H,L is driven LOW
    IOCON3bits.FLTDAT = 0;  // 0b00 = If Fault is active, PWM3H,L is driven LOW
    
    IOCON1bits.PMOD = 0;    // 0b00 = PWM1 pair is in the Complementary O/P mode
    IOCON1bits.PMOD = 0;    // 0b00 = PWM2 pair is in the Complementary O/P mode
    IOCON1bits.PMOD = 0;    // 0b00 = PWM3 pair is in the Complementary O/P mode

    // TRGCONx: PWMx TRIGGER CONTROL REGISTER
    // TRGDIV (15-12): Trigger Output Divider bits
    // Unimplemented(11-6): Read as ?0?
    // TRGSTRT(5:0): Trigger Postscaler Start Enable Select bits
    TRGCON1 = 0x0000;       // 0b0000 UUUU UU00 0000
    TRGCON2 = 0x0000;       // 0b0000 = Trigger output for every trigger event
    TRGCON3 = 0x0000;       // 0b0000 = Trigger output for every trigger event

    TRGCON1bits.TRGDIV = 0; // 0b0000 = Trigger output for every trigger event
    TRGCON2bits.TRGDIV = 0; // 0b0000 = Trigger output for every trigger event
    TRGCON3bits.TRGDIV = 0; // 0b0000 = Trigger output for every trigger event

    TRGCON1bits.TRGSTRT = 0;// 000000 = Waits 0 PWM cycles before generating
                            // the 1st trigger event after the module is enabled
    TRGCON2bits.TRGSTRT = 0;// 000000 = Waits 0 PWM cycles before generating
                            // the 1st trigger event after the module is enabled
    TRGCON3bits.TRGSTRT = 0;// 000000 = Waits 0 PWM cycles before generating
                            // the 1st trigger event after the module is enabled

    // TRIGx: PWMx PRIMARY TRIGGER COMPARE VALUE REGISTER
    // TRGCMP(15:0): Trigger Control Value bits
    // When the primary PWMx functions in local time base, 
    // this register contains the compare values that can trigger the ADC module.
    TRIG1 = PHASE1 - 1;         // Set to trigger ADC at the PWM edge
    TRIG2 = 0;                  // Set at centre of the PWM (not used)
    TRIG3 = 0;                  // Set at centre of the PWM (not used)
    
    // FCLCONx: PWMx FAULT CURRENT-LIMIT CONTROL REGISTER
    // Unimplemented(15): Read as '0'
    // CLSRC(14:10): Current-Limit Control Signal Source Select 
    // CLPOL(9): Current-Limit Polarity for PWM Generator bit
    // CLMOD (8): Current-Limit Mode Enable for PWM Generator bit
    // FLTSRC(7:3): Fault Control Signal Source Select for PWM Generator bits
    // FLTPOL (2): Fault Polarity for PWM Generator bit
    // FLTMOD (1:0): Fault Mode for PWM Generator bits 
    FCLCON1 = 0x0003;           // 0bU000 0000 0000 0011(Fault input is disabled) 
    FCLCON2 = 0x0003;           // 0bU000 0000 0000 0011(Fault input is disabled)
    FCLCON3 = 0x0003;           // 0bU000 0000 0000 0011(Fault input is disabled)
    
    FCLCON1bits.FLTSRC = 0x9;  // 0b01000 = Fault SRC is CMP2
    FCLCON2bits.FLTSRC = 0x9;  // 0b01000 = Fault SRC is CMP2
    FCLCON3bits.FLTSRC = 0x9;  // 0b01000 = Fault SRC is CMP2
    
    FCLCON1bits.FLTPOL = 1;     // 1 = The selected Fault source is active-low
    FCLCON2bits.FLTPOL = 1;     // 1 = The selected Fault source is active-low
    FCLCON3bits.FLTPOL = 1;     // 1 = The selected Fault source is active-low
    
    FCLCON1bits.FLTMOD = 0;     // 0b01 = forces PWM1H,L to FLTDAT values(cycle)
    FCLCON2bits.FLTMOD = 0;     // 0b01 = forces PWM2H,L to FLTDAT values(cycle)
    FCLCON3bits.FLTMOD = 0;     // 0b01 = forces PWM3H,L to FLTDAT values(cycle)

    // Configuring PWMx Generators Interrupts and Priority
    IPC23bits.PWM1IP = 4;       // PWM1 Interrupt Priority is 4
    IPC23bits.PWM2IP = 4;       // PWM2 Interrupt Priority is 4
    IPC24bits.PWM3IP = 4;       // PWM3 Interrupt Priority is 4

    IFS5bits.PWM1IF = 0;        // Clear PWM1 Interrupt flag
    IFS5bits.PWM2IF = 0;        // Clear PWM1 Interrupt flag
    IFS6bits.PWM3IF = 0;        // Clear PWM1 Interrupt flag

    IEC5bits.PWM1IE = 0;        // Disable PWM1 Interrupt
    IEC5bits.PWM2IE = 0;        // Disable PWM2 Interrupt
    IEC6bits.PWM3IE = 0;        // Disable PWM3 Interrupt

}
// *****************************************************************************
/* Function:
    ChargeBootstarpCapacitors()

  Summary:
    Routine to initialise PWM generators 1,2,3 to charge bootstrap capacitors

  Description:
    Function to charge bootstrap capacitors initially

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ChargeBootstarpCapacitors(void)
{
    uint16_t i = BOOTSTRAP_CHARGING_TIME;

    // Enable PWMs only on PWMxL ,to charge bootstrap capacitors initially
    // Hence PWMxH is over-ridden to "LOW"
    IOCON1bits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled
    IOCON2bits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled
    IOCON3bits.OVRDAT = 0;  // 0b00 = State for PWM1H,L, if Override is Enabled

    IOCON1bits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H
    IOCON2bits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H
    IOCON3bits.OVRENH = 1;  // 1 = OVRDAT<1> provides data for output on PWM1H

    IOCON1bits.OVRENL = 0;  // 0 = PWM generator provides data for PWM1L pin
    IOCON2bits.OVRENL = 0;  // 0 = PWM generator provides data for PWM2L pin
    IOCON3bits.OVRENL = 0;  // 0 = PWM generator provides data for PWM3L pin

    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Initialise the PWM duty cycle for charging
    PDC1 = PHASE1 - (MIN_DUTY + 3);
    PDC2 = PHASE1 - (MIN_DUTY + 3);
    PDC3 = PHASE1 - (MIN_DUTY + 3);

    PTCONbits.PTEN = 1;      // Enable PWM module for charging bootstrap CAPs

    while(i)
    {
        __delay_us(1);
        i--;
    }
    
    PTCONbits.PTEN = 0;      // Disable PWM module after charging

    // Reset modified PWM configurations after bootstrap charging
    // PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
    // Re-initialise the PWM duty cycle register
    PDC1 = MIN_DUTY;
    PDC2 = MIN_DUTY;
    PDC3 = MIN_DUTY;

    IOCON1bits.OVRENH = 0;  // 1 = PWM generator provides data for PWM1H pin
    IOCON2bits.OVRENH = 0;  // 1 = PWM generator provides data for PWM2H pin
    IOCON3bits.OVRENH = 0;  // 1 = PWM generator provides data for PWM3H pin
}
// *****************************************************************************
/* Function:
    Init_ADC1

  Summary:
    Routine to initialise ADC 

  Description:
    Function initialises the ADC1 for simultaneously sampling four AD channels
    Measures offset on all channels .

  Precondition:
    None.

  Parameters:
    Pointer to ADC_OFFSET_T structure variable created to store ADC1 offset is
    passed to this function

  Returns:
    Structure will contain the averaged initial offset value of all 4 channels

  Remarks:
    None.
 */
void InitADCModule1(ADC_OFFSET_T *adc_offset)
{
    volatile uint16_t i = 0;

    // AD1CON1: ADC1 CONTROL REGISTER 1
    // ADON(15): ADC Operating Mode bit
    // Unimplemented(14,11): Read as '0'
    // ADSIDL(13): ADC Stop in Idle Mode bit
    // ADDMABM(12): DMA Buffer Build Mode bit
    // AD12B(10): 10-Bit or 12-Bit ADC Operation Mode bit
    // FORM(9-8): Data Output Format bits
    // SSRC(7-5>: Sample Clock Source Select bits
    // SSRCG(4): Sample Trigger Source Group bit
    // SIMSAM(3): Simultaneous Sample Select bit (only if CHPS<1:0> = 01 or 1x)
    // In 12-Bit Mode (AD12B = 1), SIMSAM is Unimplemented and is Read as '0'
    // ASAM(2): ADC Sample Auto-Start bit
    // SAMP(1): ADC Sample Enable bit
    // DONE(0): ADC Conversion Status bit
    AD1CON1 = 0;            // 0b0U00 U000 0000 0000 (0 = ADC is OFF)
    AD1CON1bits.AD12B = 0;   // 0 = 10-bit, 4-channel ADC operation possible
    AD1CON1bits.FORM = 3;   // 11 = Signed fractional(DOUT = sddd dddd dd00 0000
                            // Range is shown below for 10bit operation:
                            // 0b0111 1111 1100 0000 = 0.99804
                            // 0b0000 0000 0000 0000 = 0
                            // 0b1000 0000 0000 0000 = -1
    AD1CON1bits.SSRCG = 1;  // if SSRCG =  1,
    AD1CON1bits.SSRC = 0;   // 000 = PWM Generator 1 primary trigger compare
                            // ends sampling and starts conversion
    AD1CON1bits.SIMSAM = 1; // Samples CH0-CH3 simultaneously(if CHPS = 1x).
    AD1CON1bits.ASAM = 1;   // Sampling begins immediately after
                            // last conversion completes.SAMP bit is auto set.
    AD1CON1bits.DONE = 0;   // ADC conversion has not started

    // ADxCON2: ADCx CONTROL REGISTER 2
    // VCFG(15-13): Converter Voltage Reference Configuration bits
    // Unimplemented(12-11): Read as ?0?
    // CSCNA(10): Input Scan Select bit
    // CHPS(9-8): Channel Select bits
    // In 12-Bit Mode,CHPS<1:0> Bits are Unimplemented and are Read as ?0?
    // BUFS(7): Buffer Fill Status bit (only valid when BUFM = 1)
    // SMPI(6-2): Increment Rate bits
    // BUFM(1): Buffer Fill Mode Select bit
    // ALTS(0): Alternate Input Sample Mode Select bit
    AD1CON2 = 0;
    AD1CON2bits.VCFG = 0;   // ADC Voltage Reference VREFH = AVDD; VREFL = AVSS
    AD1CON2bits.CSCNA = 0;  // 0 = Does not scan inputs
    AD1CON2bits.CHPS = 2;   // 1x = Converts CH0, CH1, CH2 and CH3
    AD1CON2bits.SMPI = 0;   // x0000 = Generates interrupt after completion
                            // of every sample/conversion operation
    AD1CON2bits.BUFM = 0;   // 0 = Starts filling buffer from the Start address
    AD1CON2bits.ALTS = 0;   // 0 = Uses channel I/P selects for Sample MUXA

    // ADxCON3: ADCx CONTROL REGISTER 3
    // ADRC(15): ADC Conversion Clock Source bit
    // Unimplemented(14-13): Read as ?0?
    // SAMC(12-8): Auto-Sample Time bits
    // ADCS(7-0): ADC Conversion Clock Select bits
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0;       // 0 = Clock derived from system clock
    // ADC Conversion Clock Select bits
    // Tp*(ADCS<7:0> + 1) = Tp*(6+1) = 7 Tp = 1 Tad
    // 7 * (1/ Fp) = 7 * (1 / Fcy) = 100nSec
    AD1CON3bits.ADCS = ADC_MIN_ADCS_COUNTS;

    // ADxCON4: ADCx CONTROL REGISTER 4
    // Unimplemented(15-9,7-3): Read as ?0?
    // ADDMAEN(8): ADC DMA Enable bit
    // DMABL(2-0): Selects Number of DMA Buffer Locations per Analog Input bits
    AD1CON4 = 0;                   // Not used in the application

    // ADxCHS123: ADCx INPUT CHANNEL 1, 2, 3 SELECT REGISTER
    // Unimplemented(15-11,7-3): Read as '0'
    // CH123SB(8): Channels 1, 2, 3 Positive Input Select for Sample B bits
    // CH123NB(10:9): Channels 1, 2, 3 Negative Input Select for Sample B bits
    // CH123SA(0): Channels 1, 2, 3 Positive Input Select for Sample A bits
    // CH123NA(2-1): Channels 1, 2, 3 Negative Input Select for Sample A bits
    AD1CHS123 = 0x0000;            // 0bUUUU U000 UUUU U000
    AD1CHS123bits.CH123SB = 0;     // Not used int the application
    AD1CHS123bits.CH123NB = 0;     // 0x = CH1, CH2, CH3 negative input is VREFL

    // Sets the simultaneously sampled channels
    // Refer periph.h for details
    AD1CHS123bits.CH123SA = ADC1_ANx_CH123;
    AD1CHS123bits.CH123NA = 0;     // 0x = CH1, CH2, CH3 negative input is VREFL
    
    // AD1CHS0: ADC1 INPUT CHANNEL 0 SELECT REGISTER
    // CH0NB(15): Channel 0 Negative Input Select for Sample MUXB bit
    // CH0SB(13-8): Channel 0 Positive Input Select for Sample MUXB bits(1,4,5)
    // CH0NA(7): Channel 0 Negative Input Select for Sample MUXA bit
    // CH0SA(5:0): Channel 0 Positive Input Select for Sample MUXA bits(1,4,5)
    // Unimplemented(14,6): Read as '0'
    AD1CHS0 = 0;
    AD1CHS0bits.CH0NB = 0;  // 0 = Channel 0 negative input is VREFL
    AD1CHS0bits.CH0SB = 0;  // 0b000000 (Not used in this application)
    AD1CHS0bits.CH0NA = 0;  // 0 = Channel 0 negative input is VREFL
    // CH0 is used for measuring voltage set by speed reference potentiometer
    AD1CHS0bits.CH0SA = ADC1_ANx_CH0;
    
    // AD1CSSH: ADC1 INPUT SCAN SELECT REGISTER HIGH
    // CSS(31-16): ADC Input Scan Selection bits
    AD1CSSH = 0;

    // AD1CSSL: ADC1 INPUT SCAN SELECT REGISTER LOW
    // CSS(15-0): ADC Input Scan Selection bits
    AD1CSSL = 0;

    // Configuring ADCx Generators Interrupts and Priority
    IPC3bits.AD1IP = 7;         // 0b111 ADC1 Interrupt Priority is 7
    IFS0bits.AD1IF = 0;         // 0 = Clear ADC1 Interrupt flag
    IEC0bits.AD1IE = 0;         // 0 = Disable ADC1 Interrupt

    AD1CON1bits.ADON = 1;

    // Time to Stabilize Analog Stage from ADC Off to ADC On
    __delay_us(ADC_TON_DELAY_MICROSEC);
    
    // Taking multiple sample to measure voltage offset in all the channels
    for (i = 0; i < 128; i++)
    {
        // Wait for the conversion to complete
        while (!AD1CON1bits.DONE);
        // Sum up the converted results
        iADCOffsetCH0 += ADC1BUF0;
        iADCOffsetCH1 += ADC1BUF1;
        iADCOffsetCH2 += ADC1BUF2;
        iADCOffsetCH3 += ADC1BUF3;
    }
    /* Averaging CHO result to find voltage offset*/
    adc_offset->ch0 = (int16_t)(iADCOffsetCH0 >> 7);
    /* Averaging CH1 result to find voltage offset*/
    adc_offset->ch1 = (int16_t)(iADCOffsetCH1 >> 7);
    /* Averaging CH2 result to find voltage offset*/
    adc_offset->ch2 = (int16_t)(iADCOffsetCH2 >> 7);
    /* Averaging CH3 result to find voltage offset*/
    adc_offset->ch3 = (int16_t)(iADCOffsetCH3 >> 7);
    
}
// *****************************************************************************
/* Function:
    Init_Peripherals()

  Summary:
    Routine initializes controller peripherals

  Description:
    Routine to initialize Peripherals used for Inverter Control

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPeripherals(void)
{
    /* Initialize comparator and amplifiers */
    InitAmplifiersComparator();
    /* Initialize PWM generators */
    InitPWMGenerators();
    /* Initialize the ADC used for sensing Inverter A parameters */
    InitADCModule1(&adcOffset);
    
    __delay_us(50);
}
