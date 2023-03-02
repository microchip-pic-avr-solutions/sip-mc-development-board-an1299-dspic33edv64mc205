/**
  @File Name:
    uart1.h

  @Summary:
    This module provides peripheral drivers for UART1.

  @Description:
    This module provides peripheral drivers for UART1.
 */
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

#ifndef _UART1_H
#define _UART1_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

/**
  Section: Type defines
 */ 
    
/** UART receive interrupt mode */
typedef enum tagUART1_URXISEL_TYPE
{ 
    /** Interrupt is set when any character is received
     *  and transferred from the UxRSR */
    UART1_URXISEL_ONECHAR    = 1,
    /** Interrupt is set on UxRSR transfer, making the receive
     * buffer 3/4 full (i.e., has 3 data characters) */
    UART1_URXISEL_THREECHAR  = 2,
    /** Interrupt is set on UxRSR transfer, making the receive
     * buffer full (i.e., has 4 data characters) */
    UART1_URXISEL_FOURCHAR   = 3
}UART1_URXISEL_TYPE;

/** UART status */
typedef enum tagUART1_STATUS
{
    /** Indicates that Receive buffer has data, at least one more character can be read */
    UART1_RX_DATA_AVAILABLE = (1 << 0),
    
    /** Indicates that Receive buffer has overflowed */
    UART1_RX_OVERRUN_ERROR = (1 << 1),

    /** Indicates that Framing error has been detected for the current character */
    UART1_FRAMING_ERROR = (1 << 2),

    /** Indicates that Parity error has been detected for the current character */
    UART1_PARITY_ERROR = (1 << 3),

    /** Indicates that Receiver is Idle */
    UART1_RECEIVER_IDLE = (1 << 4),

    /** Indicates that the last transmission has completed */
    UART1_TX_COMPLETE = (1 << 8),

    /** Indicates that Transmit buffer is full */
    UART1_TX_FULL = (1 << 9)
}UART1_STATUS;

/**
  Section: Driver Interface
 */

/**
 * Clears UART1 transmit interrupt request flag.
 * Summary: Clears UART1 transmit interrupt request flag.
 * @example
 * <code>
 * UART1_InterruptTransmitFlagClear();
 * </code>
 */
inline static void UART1_InterruptTransmitFlagClear(void) {_U1TXIF = 0; }

/**
 * Clears UART1 receive interrupt request flag.
 * Summary: Clears UART1 receive interrupt request flag.
 * @example
 * <code>
 * UART1_InterruptReceiveFlagClear();
 * </code>
 */
inline static void UART1_InterruptReceiveFlagClear(void) {_U1RXIF = 0; }

/**
 * Enables UART1 transmit interrupt.
 * Summary: Enables UART1 transmit interrupt.
 * @example
 * <code>
 * UART1_InterruptTransmitEnable();
 * </code>
 */
inline static void UART1_InterruptTransmitEnable(void) {_U1TXIE = 1; }

/**
 * Disables UART1 transmit interrupt.
 * Summary: Disables UART1 transmit interrupt.
 * @example
 * <code>
 * UART1_InterruptTransmitDisable();
 * </code>
 */
inline static void UART1_InterruptTransmitDisable(void) {_U1TXIE = 0; }

/**
 * Enables UART1 receive interrupt.
 * Summary: Enables UART1 receive interrupt.
 * @example
 * <code>
 * UART1_InterruptReceiveEnable();
 * </code>
 */
inline static void UART1_InterruptReceiveEnable(void) {_U1RXIE = 1; }

/**
 * Disables UART1 receive interrupt.
 * Summary: Disables UART1 receive interrupt.
 * @example
 * <code>
 * UART1_InterruptReceiveDisable();
 * </code>
 */
inline static void UART1_InterruptReceiveDisable(void) {_U1RXIE = 0; }

/**
 * Enables and initializes UART1 with a default configuration:
 * - Continues module operation in Idle mode
 * - IrDA encoder and decoder are disabled
 * - UxRTS pin is in Flow Control mode
 * - UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLKx pins are
 *   controlled by PORT latches
 * - No wake-up is enabled
 * - Loopback mode is disabled
 * - Baud rate measurement is disabled
 * - UxRX Idle state is '1'
 * - BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
 * - 8-bit data, no parity
 * - One Stop bit.
 * This function does not configure the baud rate for UART1, however, it does
 * clear interrupt flags and disable interrupts for UART1 before initializing.
 * Summary: Enables and initializes UART1 with a default configuration.
 * @example
 * <code>
 * UART1_Initialize(void);
 * </code>
 */
void UART1_Initialize(void);

/**
 * Configures UART1 module to operate in standard baud rate mode (i.e. 16x baud clock).
 * Summary: Configures UART1 module to operate in standard baud rate mode (i.e. 16x baud clock).
 * @example
 * <code>
 * UART1_SpeedModeStandard();
 * </code>
 */
inline static void UART1_SpeedModeStandard(void) {U1MODEbits.BRGH = 0; }

/**
 * Configures UART1 module to operate in High-speed baud rate mode (i.e. 4x baud clock).
 * Summary: Configures UART1 module to operate in High-speed baud rate mode (i.e. 4x baud clock).
 * @example
 * <code>
 * UART1_SpeedModeHighSpeed();
 * </code>
 */
inline static void UART1_SpeedModeHighSpeed(void) {U1MODEbits.BRGH = 1; }

/**
 * Configures the baud rate divider for UART1 module.
 * Summary: Configures the baud rate divider for UART1 module.
 * @example
 * <code>
 * UART1_BaudRateDividerSet(500);
 * </code>
 */
inline static void UART1_BaudRateDividerSet(uint16_t baudRateDivider)
{
    U1BRG = baudRateDivider;
}

/**
 * Disables UART1 module.
 * Summary: Disables UART1 module.
 * @example
 * <code>
 * UART1_ModuleDisable();
 * </code>
 */
inline static void UART1_ModuleDisable(void) {U1MODEbits.UARTEN = 0; }

/**
 * Enables UART1 module.
 * Summary: Enables UART1 module.
 * @example
 * <code>
 * UART1_ModuleEnable();
 * </code>
 */
inline static void UART1_ModuleEnable(void) {U1MODEbits.UARTEN = 1; }

/**
 * Enables UART1 module transmit mode.
 * @example
 * <code>
 * UART1_TransmitModeEnable();
 * </code>
 */
inline static void UART1_TransmitModeEnable(void) {U1STAbits.UTXEN = 1; }

/**
 * Disables UART1 module transmit mode.
 * @example
 * <code>
 * UART1_TransmitModeDisable();
 * </code>
 */
inline static void UART1_TransmitModeDisable(void) {U1STAbits.UTXEN = 0; }

/**
 * Gets the status of UART1 Receive Buffer Data Available flag
 * @return status of UART1 Receive Buffer Data Available flag; 
 * 1 = Receive buffer has data, 0 = Receive buffer is empty 
 * @example
 * <code>
 * status = UART1_IsReceiveBufferDataReady();
 * </code>
 */

inline static bool UART1_IsReceiveBufferDataReady(void)
{
    return(U1STAbits.URXDA) ;
}

/**
 * Gets the status of UART1 Receive Buffer Overrun Error Status flag
 * @return status of UART1 Receive Buffer Overrun Error Status flag; 
 * 1 = Receive buffer has overflowed, 0 = Receive buffer has not overflowed
 * @example
 * <code>
 * status = UART1_IsReceiveBufferOverFlowDetected();
 * </code>
 */
inline static bool UART1_IsReceiveBufferOverFlowDetected(void)
{
    return(U1STAbits.OERR) ;
}

/**
 * Gets the status of UART1 Framing Error Status flag
 * @return status of UART1 Framing Error Status flag; 
 * 1 = Framing error has been detected for the character at the top of the receive FIFO
 * 0 = Framing error has not been detected
 * @example
 * <code>
 * status = UART1_IsFrameErrorDetected();
 * </code>
 */
inline static bool UART1_IsFrameErrorDetected(void)
{
    return(U1STAbits.FERR) ;
}
/**
 * Gets the status of UART1 Parity Error Status flag
 * @return status of UART1 Parity Error Status flag; 
 * 1 = Parity error has been detected for the character at the top of the receive FIFO
 * 0 = Parity error has not been detected
 * @example
 * <code>
 * status = UART1_IsParityErrorDetected();
 * </code>
 */
inline static bool UART1_IsParityErrorDetected(void)
{
    return(U1STAbits.PERR) ;
}
/**
 * Gets the status of UART1 Receiver is Idle Status Flag
 * @return status of UART1 Receiver is Idle Status flag; 
 * 1 = Receiver is Idle
 * 0 = Receiver is Active
 * @example
 * <code>
 * status = UART1_IsReceiverIdle();
 * </code>
 */
inline static bool UART1_IsReceiverIdle(void)
{
    return(U1STAbits.RIDLE) ;
}
/**
 * Gets the status of UART1 Transmit Shift Register Empty Status Flag
 * @return status of UART1 Transmit Shift Register Empty Status flag; 
 * 1 = Transmit Shift Register is empty and transmit buffer is empty
 * 0 = Transmit Shift Register is not empty, a transmission is in progress or queued
 * @example
 * <code>
 * status = UART1_IsTransmissionComplete();
 * </code>
 */
inline static bool UART1_IsTransmissionComplete(void)
{
    return(U1STAbits.TRMT) ;
}

/**
 * Gets the status of UART1 transmit buffer full flag
 * @return status of UART1 transmit buffer full flag; 1 = Transmit buffer is
 *  full, 0 = Transmit buffer is not full
 * @example
 * <code>
 * status = UART1_StatusBufferFullTransmitGet();
 * </code>
 */
inline static bool UART1_StatusBufferFullTransmitGet(void)
{
    return U1STAbits.UTXBF;
}

/**
 * Gets the transmitter and receiver status of UART1. The returned status may
 * contain a value with more than one of the bits specified in the
 * UART1_STATUS enumeration set. The caller should perform an "AND" with the
 * bit of interest and verify if the result is non-zero to verify the 
 * desired status bit.
 * @return UART1_STATUS value describing the current status of the transfer.
 * @example
 * <code>
 * while(!(UART1_StatusGet() & UART_TX_COMPLETE));
 * </code>
 */
inline static UART1_STATUS UART1_StatusGet(void)
{
    return U1STA;
}

/**
 * Clears the Receive Buffer Overrun Error Status bit for UART1. If this bit
 * was previously set, then calling this function will reset the receiver buffer
 * and the U1RSR to an empty state.
 * @example
 * <code>
 * UART1_ReceiveBufferOverrunErrorFlagClear();
 * </code>
 */
inline static void UART1_ReceiveBufferOverrunErrorFlagClear(void)
{
    U1STAbits.OERR = 0;
}

/**
 * Writes a 16-bit data word to UART1 transmit register.
 * @param data data to be transmitted on UART1
 * @example
 * <code>
 * UART1_DataWrite(txdata);
 * </code>
 */
inline static void UART1_DataWrite(uint16_t data)
{
    U1TXREG = data;
}

/**
 * Reads a 16-bit data word from the UART1 receive register.
 * @return data read from the UART1 receive register
 * @example
 * <code>
 * rxdata = UART1_DataRead();
 * </code>
 */
inline static uint16_t UART1_DataRead(void)
{
    return U1RXREG;
}

#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif //_UART1_H

/**
 End of File
 */