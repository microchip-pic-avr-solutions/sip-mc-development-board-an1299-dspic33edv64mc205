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

; ******************************************************************************
; MeasCompCurr
;
;Description:
;  Read Channels 1 & 2 of ADC, scale them as signed fractional values
;  using qKa, qKb and put the results qIa and qIb of MeasCurrParm.
;
;  Specifically the offset is used to correct the raw ADC by
;         CorrADC = CurrentIn - Offset
;
;  Do not call this routine until conversion is completed.
;
;  Scaling constant, qKa and qKb, must be set elsewhere such that
;         qIa = qKa * CorrADC1
;         qIb = qKb * CorrADC2
;
;Functional prototypes:
; void MeasCompCurr(int,int,MEAS_CURR_PARM_T *MeasCurrParm);
; void InitMeasCompCurr(int Offset_a,int Offset_b,MEAS_CURR_PARM_T *MeasCurrParm);
;
;On Start:   Must call InitMeasCompCurr.
;
;On Entry:   MeasCurrParm structure must contain qKa & qKb. ADC conversion results
;            must contain signed fractional value.
;
;On Exit:    MeasCurrParm will contain qIa & qIb.
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
; Registers modified: w0,w1,w2,w4,w5
;
;
;*******************************************************************

.ifdecl __ASM30__
    .include "xc.inc"
.endif

; External references
.include "meascurr.inc"

; Register usage
.equ Current_Ia,w0      ; ADC Buffer Register
.equ Current_Ib,w1      ; ADC Buffer Register
.equ MeasCurrParm,w2    ; Base of MeasCurrParm structure

.global   _MeasCompCurr
.global   MeasCompCurr
; Inputs:
;  Ia
;  Ib
;  MeasCurrParm.Offseta
;  MeasCurrParm.Offsetb
;  MeasCurrParm.qKa
;  MeasCurrParm.qKb

; Outputs:
;  MeasCurrParm.qIa
;  MeasCurrParm.qIb
_MeasCompCurr:
MeasCompCurr:

    ;; CorrADC1 = Current_Ia - iOffsetHa
    ;; qIa = 2 * qKa * CorrADC1
    mov.w     [MeasCurrParm+ADC_Offseta],w5
    sub.w     w5,Current_Ia,w5                      ; w5 = Current_Ia - Offset
    mov.w     [MeasCurrParm+ADC_qKa],w4
    mpy       w4*w5,A
    sac       A,#-1,w4
    mov.w     w4,[MeasCurrParm+ADC_qIa]

    ;; CorrADC1 = Current_Ib - iOffsetHb
    ;; qIa = 2 * qKa * CorrADC1
    mov.w     [MeasCurrParm+ADC_Offsetb],w5
    sub.w     w5,Current_Ib,w5                      ; w5 = Current_Ib - Offset
    mov.w     [MeasCurrParm+ADC_qKb],w4
    mpy       w4*w5,A
    sac       A,#-1,w4
    mov.w     w4,[MeasCurrParm+ADC_qIb]

    return

; Inputs:
;  Offseta
;  Offsetb

; Outputs:
;  MeasCurrParm.Offseta
;  MeasCurrParm.Offsetb
.global   _InitMeasCompCurr
.global   InitMeasCompCurr

_InitMeasCompCurr:
InitMeasCompCurr:
    mov.w     w0,[MeasCurrParm+ADC_Offseta]
    mov.w     w1,[MeasCurrParm+ADC_Offsetb]
    return

.end
