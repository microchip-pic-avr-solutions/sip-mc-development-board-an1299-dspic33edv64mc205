;*******************************************************************************
; Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
;
; SOFTWARE LICENSE AGREEMENT:
;
; Microchip Technology Incorporated ("Microchip") retains all ownership and
; intellectual property rights in the code accompanying this message and in all
; derivatives hereto.  You may use this code, and any derivatives created by
; any person or entity by or on your behalf, exclusively with Microchip's
; proprietary products.  Your acceptance and/or use of this code constitutes
; agreement to the terms and conditions of this notice.
;
; CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
; WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
; TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
; PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
; PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
;
; YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
; WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
; STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
; FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
; LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
; HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
; THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
; MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
; SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
; HAVE THIS CODE DEVELOPED.
;
; You agree that you are solely responsible for testing the code and
; determining its suitability.  Microchip has no obligation to modify, test,
; certify, or support the code.
;
;******************************************************************************/

;*******************************************************************
; ReadADC0 and ReadSignedADC0
;
;Description:
;  Read Channel 0 of ADC, scale it using qK and put results in qADValue.
;  Do not call this routine until conversion is completed.
;
;  ReadADC0 range is qK*(0.0 ->0.9999).
;  ReadSignedADC0 range is qK*(-1.0 ->0.9999).
;
;  Scaling constant, qK, must be set elsewhere such that
;         iResult = 2 * qK * ADCBUF0
;  The factor of 2 is designed to allow qK to be given in 1.15.
;
;
;Function prototype:
; Calculates unsigned value 0 -> 2*qK
; void ReadADC0( int16_t ADC_Result,READ_ADC_PARM_T* pParm )
; Calculates signed value -2*qK -> 2*qK
; void ReadSignedADC0( int16_t ADC_Result,READ_ADC_PARM_T* pParm )
;
;On Entry:   ReadADCParm structure must contain qK. ADC channel 0
;            must contain signed fractional value.
;
;On Exit:    ReadADCParm will contain qADValue
;
;Parameters:
; Input arguments: None
;
; Return:
;   Void
;
; SFR Settings required:
;         CORCON.SATA  = 0
;     If there is any chance that Accumulator will overflow must set
;         CORCON.SATDW = 1
;
; Support routines required: None
;
; Local Stack usage: None
;
; Registers modified: w0,w4,w5
;
; Timing: 13 cycles
;
;*******************************************************************
;
.ifdecl __ASM30__
    .include "xc.inc"
.endif

; External references
.include "ReadADC.inc"

; Register usage
; ADC Result
.equ ADC_Result,w0
; Base of parameter structure
.equ ParmBaseW,w1
.equ Work0W,   w4
.equ Work1W,   w5

.section  .text
.global   _ReadADC0
.global   ReadADC0

; Inputs:
;  ADC_Value

; Outputs:
;  Read_ADC_Parm.qADvalue
_ReadADC0:
ReadADC0:

;; iResult = 2 * qK * ADCBUF0

    mov.w    [ParmBaseW+ADC_qK],Work0W
    mov.w    ADC_Result, Work1W

;; Change from signed fractional to fractional, i.e. convert
;; from -1->.9999 to 0 -> 0.9999
    btg       Work1W,#15
    lsr.w     Work1W,Work1W

    mpy       Work0W*Work1W,A
    sac       A,#-1,Work0W
    mov.w     Work0W,[ParmBaseW+ADC_qADValue]
    return


.global   _ReadSignedADC0
.global   ReadSignedADC0

; Inputs:
; ADC Value

; Outputs:
; Read_ADC_Parm.qADvalue
_ReadSignedADC0:
ReadSignedADC0:

;; iResult = 2 * qK * ADCBUF0

    mov.w     [ParmBaseW+ADC_qK],Work0W
    mov.w	  ADC_Result, Work1W

    mpy       Work0W*Work1W,A
    sac       A,#-1,Work0W
    mov.w     Work0W,[ParmBaseW+ADC_qADValue]

    return

.end
