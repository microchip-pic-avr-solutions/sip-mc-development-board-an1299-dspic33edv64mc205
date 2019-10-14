/**
 * diagnostics.c
 * 
 * Diagnostics code
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

#include <stdint.h>
#include <stdbool.h>
#include "diagnostics.h"
#include "uart1.h"
#include "rtdmuser.h"
#include "rtdm_select_channels.h"

void DiagnosticsInit();

/* ******************** Variables to display data using DMCI *********************************/

#include "rtdm.h"

/**
 * addresses for logging
 */
static const volatile int16_t *RTDM_addresses[4];

/* Buffer to store the data samples for the DMCI data viewer graphs */
int RecorderBuffer1[DATA_BUFFER_SIZE]; 
int RecorderBuffer2[DATA_BUFFER_SIZE];
int RecorderBuffer3[DATA_BUFFER_SIZE];
int RecorderBuffer4[DATA_BUFFER_SIZE];

/* Tail pointer for the DMCI Graphs */
int * PtrRecBuffer1 = &RecorderBuffer1[0]; 
int * PtrRecBuffer2 = &RecorderBuffer2[0];
int * PtrRecBuffer3 = &RecorderBuffer3[0];
int * PtrRecBuffer4 = &RecorderBuffer4[0];
int * RecBuffUpperLimit = RecorderBuffer4 + DATA_BUFFER_SIZE - 1; /* Buffer Recorder Upper Limit */

typedef struct DMCIFlags
{
    /* Flag needs to be set to start buffering data */
    unsigned Recorder : 1; 
    unsigned StartStop : 1;
    unsigned unused : 14;
} DMCIFLAGS;
DMCIFLAGS DMCIFlags;
int SnapCount = 0;

int SnapShotDelayCnt = 0;
int SnapShotDelay = SNAPDELAY;
int SpeedReference = 0;

void DiagnosticsInit(void)
{
    RTDM_SelectChannels(RTDM_addresses);
    RTDM_Start(); 
    /* Configure the UART module used by RTDM
     * it also initializes the RTDM variables
     */
    DMCIFlags.StartStop = 0;
    DMCIFlags.Recorder = 0;
}

void DiagnosticsStepMain(void)
{
    RTDM_ProcessMsgs();
}

void DiagnosticsStepIsr(void)
{
    /* ******************** DMCI Dynamic Data Views  ***************************/
    /* ********************* RECORDING MOTOR PHASE VALUES ***************/
    if (DMCIFlags.Recorder)
    {
        SnapShotDelayCnt++;
        if (SnapShotDelayCnt == SnapShotDelay)
        {
            SnapShotDelayCnt = 0;
            const volatile int16_t **pdata = RTDM_addresses;
            *PtrRecBuffer1++ = **pdata++;
            *PtrRecBuffer2++ = **pdata++;
            *PtrRecBuffer3++ = **pdata++;
            *PtrRecBuffer4++ = **pdata++;

            if (PtrRecBuffer4 > RecBuffUpperLimit)
            {
                PtrRecBuffer1 = RecorderBuffer1;
                PtrRecBuffer2 = RecorderBuffer2;
                PtrRecBuffer3 = RecorderBuffer3;
                PtrRecBuffer4 = RecorderBuffer4;
                DMCIFlags.Recorder = 0;
            }
        }
    }
}
