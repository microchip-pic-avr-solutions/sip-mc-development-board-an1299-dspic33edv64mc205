/** 
 * rtdm_hal_adapter.h
 * 
 * PLIB-compatible replacement of UART functions using the HAL
 * 
 * Component: diagnostics
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

#ifndef __RTDM_HAL_ADAPTER_H
#define	__RTDM_HAL_ADAPTER_H

#include <stdint.h>
#include <stdbool.h>
#include "hal/uart1.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * Writes a character to UART1
 * 
 * @param c character to write 
 */
inline static void WriteUART1(uint8_t c)
{
    U1TXREG = c;
}

/**
 * Returns true if data is available from UART1
 * 
 * @return true if data is available
 */
inline static bool DataRdyUART1(void)
{
    return U1STAbits.URXDA;
}

/**
 * Reads a character from UART1.
 * This function is non-blocking; use DataRdyUART1 to check whether
 * valid data is available
 * 
 * @return character
 */
inline static uint8_t ReadUART1(void)
{
    return U1RXREG;
}

/**
 * Returns whether the UART cannot accept additional characters
 * @return true if the UART cannot accept additional characters
 */
inline static bool BusyUART1(void)
{
    return U1STAbits.UTXBF;
}

/**
 * Disables the UART
 */
inline static void CloseUART1(void)
{
    UART1_ModuleDisable();
}

/**
 * Write a null-terminated character string to UART1.
 * 
 * @param pointer to buffer. MUST INCLUDE A '\0'
 */
inline static void putsUART1(const uint8_t *pbuf)
{
    while (*pbuf != '\0')
    {
        WriteUART1(*pbuf++);
        while (BusyUART1())
        {
            /* do nothing */
        }
    }
}

#ifdef	__cplusplus
}
#endif

#endif	/* __RTDM_HAL_ADAPTER_H */

