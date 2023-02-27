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
#include <stdbool.h>

#include <libq.h>      
#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "estim.h"
#include "fdweak.h"

#include "readadc.h"   
#include "meascurr.h" 

#include "hal/periph.h"   
#include "hal/board_service.h"

#include "diagnostics.h"
#include "delay.h"

volatile UGF_T uGF;

CTRL_PARM_T ctrlParm;
MOTOR_STARTUP_DATA_T motorStartUpData;

MEAS_CURR_PARM_T measCurrParm;   
READ_ADC_PARM_T readADCParm;  

volatile int16_t thetaElectrical = 0,thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;

MC_ALPHABETA_T valphabeta,ialphabeta;
MC_SINCOS_T sincosTheta;
MC_DQ_T vdq,idq;
MC_DUTYCYCLEOUT_T pwmDutycycle;
MC_ABC_T   vabc,iabc;

MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;
   
volatile uint16_t measCurrOffsetFlag = 0;
/** Definitions */
/* Open loop angle scaling Constant - This corresponds to 1024(2^10)
   Scaling down motorStartUpData.startupRamp to thetaElectricalOpenLoop   */
#define STARTUPRAMP_THETA_OPENLOOP_SCALER       10 
/** Define max voltage vector */
#define MAX_VOLTAGE_VECTOR                      0.98

void InitControlParameters(void);
void DoControl(void);
void CalculateParkAngle(void);
void ResetParmeters(void);
void MeasCurrOffset(int16_t *,int16_t *);

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{
    InitOscillator();
    SetupGPIOPorts();

    /* Initialize Peripherals */
    InitPeripherals();
    MeasCurrOffset(&measCurrParm.Offseta,&measCurrParm.Offsetb);
    DiagnosticsInit();
    
    BoardServiceInit();
    CORCONbits.SATA = 0;
    LED1 = 0;
    while(1)
    {
        /* Initialize PI control parameters */
        InitControlParameters();        
        /* Initialize estimator parameters */
        InitEstimParm();
        /* Initialize flux weakening parameters */
        InitFWParams();
        /* Reset parameters used for running motor through Inverter A*/
        ResetParmeters();

        while(systemState != SYSTEM_READY)
        {
            BoardService();
        }
        HAL_Board_FaultClear();
        while(1)
        {
            DiagnosticsStepMain();
            BoardService();
            if (uGF.bits.RunMotor == 0)
            {
                LED1 = 1;
            }
            else
            {
                LED1 = 0;
            }
            if (IsPressed_Button1())
            {
                if (uGF.bits.RunMotor == 1)
                {
                    ResetParmeters();
                }
                else
                {
                    EnablePWMOutputsInverterA();
                    uGF.bits.RunMotor = 1;
                }

            }
            if (IsPressed_Button2())
            {
                HAL_Board_AutoBaudRequest();
                if ((uGF.bits.RunMotor == 1) && (uGF.bits.OpenLoop == 0))
                {
                    uGF.bits.ChangeSpeed = !uGF.bits.ChangeSpeed;
                }
            }
            
        }

    } // End of Main loop

    // should never get here
    while(1){}
}
// *****************************************************************************
/* Function:
    ResetParmsA()

  Summary:
    This routine resets all the parameters required for Motor through Inv-A

  Description:
    Reinitializes the duty cycle,resets all the counters when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmeters(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
    
    /* Re initialize the duty cycle to minimum value */
    INVERTERA_PWM_PDC1 = MIN_DUTY;
    INVERTERA_PWM_PDC2 = MIN_DUTY;
    INVERTERA_PWM_PDC3 = MIN_DUTY;
    DisablePWMOutputsInverterA();
    
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;        
    /* Set the reference speed value to 0 */
    ctrlParm.qVelRef = 0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    /* Change speed */
    uGF.bits.ChangeSpeed = 0;
    /* Change mode */
    uGF.bits.ChangeMode = 1;
    
    /* Initialize PI control parameters */
    InitControlParameters();        
    /* Initialize estimator parameters */
    InitEstimParm();
    /* Initialize flux weakening parameters */
    InitFWParams();

    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    EnableADCInterrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI iteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI iteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary variables for sqrt calculation of q reference */
    volatile int16_t temp_qref_pow_q15;
    
    if (uGF.bits.OpenLoop)
    {
        /* OPENLOOP:  force rotating angle,Vd and Vq */
        if (uGF.bits.ChangeMode)
        {
            /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;

            /* Synchronize angles */
            /* VqRef & VdRef not used */
            ctrlParm.qVqRef = 0;
            ctrlParm.qVdRef = 0;

            /* Reinitialize variables for initial speed ramp */
            motorStartUpData.startupLock = 0;
            motorStartUpData.startupRamp = 0;
            #ifdef TUNING
                /* Tuning speed ramp value  */
                motorStartUpData.tuningAddRampup = 0;
                /* tuning speed ramp increase delay */
                motorStartUpData.tuningDelayRampup = 0;   
            #endif
        }
        /* Speed reference */
        ctrlParm.qVelRef = Q_CURRENT_REF_OPENLOOP;
        /* q current reference is equal to the velocity reference 
         while d current reference is equal to 0
        for maximum startup torque, set the q current to maximum acceptable 
        value represents the maximum peak value */
        ctrlParm.qVqRef = ctrlParm.qVelRef;

        /* PI control for Q */
        piInputIq.inMeasure = idq.q;
        piInputIq.inReference = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d = piOutputId.out;

    }
    else
    /* Closed Loop Vector Control */
    {
        /* if change speed indication, double the speed */
        if (uGF.bits.ChangeSpeed)
        {
            
                /* read not signed ADC */
                ReadADC0(ADCBUF_SPEED_REF_A,&readADCParm);

            readADCParm.qAnRef = (__builtin_muluu(readADCParm.qADValue,
                    MAXIMUMSPEED_ELECTR-NOMINALSPEED_ELECTR)>>15)+
                    NOMINALSPEED_ELECTR;  
            if (readADCParm.qAnRef < ENDSPEED_ELECTR)
                {
                    readADCParm.qAnRef =  ENDSPEED_ELECTR;
                }
        }
        else
        {
                /* unsigned values */
                ReadADC0(ADCBUF_SPEED_REF_A,&readADCParm);
            readADCParm.qAnRef = (__builtin_muluu(readADCParm.qADValue,
                    NOMINALSPEED_ELECTR-ENDSPEED_ELECTR)>>15) +
                    ENDSPEED_ELECTR;  
            if (readADCParm.qAnRef < ENDSPEED_ELECTR)
            {
               readADCParm.qAnRef =  ENDSPEED_ELECTR;
            }
        }
        if (ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {

            /* Ramp generator to limit the change of the speed reference
              the rate of change is defined by CtrlParm.qRefRamp */
            ctrlParm.qDiff = ctrlParm.qVelRef - readADCParm.qAnRef;
            /* Speed Ref Ramp */
            if (ctrlParm.qDiff < 0)
            {
                /* Set this cycle reference as the sum of
                previously calculated one plus the reference ramp value */
                ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRamp;
            }
            else
            {
                /* Same as above for speed decrease */
                ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRamp;
            }
            /* If difference less than half of ref ramp, set reference
            directly from the pot */
            if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
            {
                ctrlParm.qVelRef = readADCParm.qAnRef;
            }
            ctrlParm.speedRampCount = 0;
        }    
        /* Tuning is generating a software ramp
        with sufficiently slow ramp defined by 
        TUNING_DELAY_RAMPUP constant */
        #ifdef TUNING
            /* if delay is not completed */
            if (motorStartUpData.tuningDelayRampup > TUNING_DELAY_RAMPUP)
            {
                motorStartUpData.tuningDelayRampup = 0;
            }
            /* While speed less than maximum and delay is complete */
            if ((motorStartUpData.tuningAddRampup < (MAXIMUMSPEED_ELECTR - ENDSPEED_ELECTR)) &&
                                                  (motorStartUpData.tuningDelayRampup == 0) )
            {
                /* Increment ramp add */
                motorStartUpData.tuningAddRampup++;
            }
            motorStartUpData.tuningDelayRampup++;
            /* The reference is continued from the open loop speed up ramp */
            ctrlParm.qVelRef = ENDSPEED_ELECTR +  motorStartUpData.tuningAddRampup;
        #endif

        if (uGF.bits.ChangeMode)
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0;
            piInputOmega.piState.integrator = (int32_t)ctrlParm.qVqRef << 13;
            ctrlParm.qVelRef = ENDSPEED_ELECTR;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the velocity control loop */
            piInputOmega.inMeasure = estimator.qVelEstim;
            piInputOmega.inReference = ctrlParm.qVelRef;
            MC_ControllerPIUpdate_Assembly(piInputOmega.inReference,
                                           piInputOmega.inMeasure,
                                           &piInputOmega.piState,
                                           &piOutputOmega.out);
            ctrlParm.qVqRef = piOutputOmega.out;
        #else
            ctrlParm.qVqRef = ctrlParm.qVelRef;
        #endif
        
        /* Flux weakening control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d current component 
        adapt the estimator parameters in concordance with the speed */
        ctrlParm.qVdRef=FieldWeakening(_Q15abs(ctrlParm.qVelRef));

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d    = piOutputId.out;

        /* Dynamic d-q adjustment
         with d component priority 
         vq=sqrt (vs^2 - vd^2) 
        limit vq maximum to the one resulting from the calculation above */
        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        temp_qref_pow_q15 = Q15(MAX_VOLTAGE_VECTOR) - temp_qref_pow_q15;
        piInputIq.piState.outMax = Q15SQRT (temp_qref_pow_q15);

        /* PI control for Q */
        piInputIq.inMeasure  = idq.q;
        piInputIq.inReference  = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;
    }
       
}
// *****************************************************************************
/* Function:
    ADCInterrupt()
  Summary:
    ADCInterrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void)
{

    if (uGF.bits.RunMotor)
    {
        /* Calculate qIa,qIb */
        MeasCompCurr(ADCBUF_INV_A_IPHASE1, ADCBUF_INV_A_IPHASE2,&measCurrParm);
        iabc.a = measCurrParm.qIa;
        iabc.b = measCurrParm.qIb;
        /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
        MC_TransformClarke_Assembly(&iabc,&ialphabeta);
        MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

        /* Speed and field angle estimation */
        Estim();
        /* Calculate control values */
        DoControl();
        /* Calculate qAngle */
        CalculateParkAngle();
        /* if open loop */
        if (uGF.bits.OpenLoop == 1)
        {
            /* the angle is given by park parameter */
            thetaElectrical = thetaElectricalOpenLoop;
        }
        else
        {
            /* if closed loop, angle generated by estimator */
            thetaElectrical = estimator.qRho;
        }
        MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);
        MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);

        MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);
        MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc,pwmPeriod,
                                                    &pwmDutycycle);

        if (pwmDutycycle.dutycycle1 < MIN_DUTY)
        {
            pwmDutycycle.dutycycle1 = MIN_DUTY;
        }
        if (pwmDutycycle.dutycycle2 < MIN_DUTY)
        {
            pwmDutycycle.dutycycle2 = MIN_DUTY;
        }
        if (pwmDutycycle.dutycycle3 < MIN_DUTY)
        {
            pwmDutycycle.dutycycle3 = MIN_DUTY;
        }
        INVERTERA_PWM_PDC1 = pwmDutycycle.dutycycle1;
        INVERTERA_PWM_PDC2 = pwmDutycycle.dutycycle2;
        INVERTERA_PWM_PDC3 = pwmDutycycle.dutycycle3;

    }
    else
    {
        INVERTERA_PWM_PDC3 = MIN_DUTY;
        INVERTERA_PWM_PDC2 = MIN_DUTY;
        INVERTERA_PWM_PDC1 = MIN_DUTY;
        measCurrOffsetFlag = 1;
    }
    DiagnosticsStepIsr();
    BoardServiceStepIsr();
    /* Clear Interrupt Flag */
    ClearADCIF();
}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calculates the angle for open loop control

  Description:
    Generate the start sine waves feeding the motor terminals
    Open loop control, forcing the motor to align and to start speeding up .
 
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{
    /* if open loop */
    if (uGF.bits.OpenLoop)
    {
        /* begin with the lock sequence, for field alignment */
        if (motorStartUpData.startupLock < LOCK_TIME)
        {
            motorStartUpData.startupLock += 1;
        }
        /* Then ramp up till the end speed */
        else if (motorStartUpData.startupRamp < END_SPEED)
        {
            motorStartUpData.startupRamp += OPENLOOP_RAMPSPEED_INCREASERATE;
        }
        /* Switch to closed loop */
        else 
        {
            #ifndef OPEN_LOOP_FUNCTIONING
                uGF.bits.ChangeMode = 1;
                uGF.bits.OpenLoop = 0;
            #endif
        }
        /* The angle set depends on startup ramp */
        thetaElectricalOpenLoop += (int16_t)(motorStartUpData.startupRamp >> 
                                            STARTUPRAMP_THETA_OPENLOOP_SCALER);

    }
    /* Switched to closed loop */
    else 
    {
        /* In closed loop slowly decrease the offset add to the estimated angle */
        if (estimator.qRhoOffset > 0)
        {
            estimator.qRhoOffset--;
        }
    }
}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initializes control parameters

  Description:
    Initialize control parameters: PI coefficients, scaling constants etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{
    /* ADC - Measure Current & Pot */
    /* Scaling constants: Determined by calibration or hardware design.*/
    readADCParm.qK = KPOT;
    measCurrParm.qKa = KCURRA;
    measCurrParm.qKb = KCURRB;
    
    ctrlParm.qRefRamp = SPEEDREFRAMP;
    ctrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    /* Set PWM period to Loop Time */
    pwmPeriod = LOOPTIME_TCY;
 
    /* PI - Id Current Control */
    piInputId.piState.kp = D_CURRCNTR_PTERM;
    piInputId.piState.ki = D_CURRCNTR_ITERM;
    piInputId.piState.kc = D_CURRCNTR_CTERM;
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    /* PI - Iq Current Control */
    piInputIq.piState.kp = Q_CURRCNTR_PTERM;
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = -piInputIq.piState.outMax;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    /* PI - Speed Control */
    piInputOmega.piState.kp = SPEEDCNTR_PTERM;
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;
    piInputOmega.piState.outMin = -piInputOmega.piState.outMax;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;
}
void __attribute__ ((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    if (BSP_LATCH_GATE_DRIVER_A_FAULT == false)
    {
        ResetParmeters();
        LED2 = 1;
        HAL_Board_FaultClear();
    }
    else
    {
        LED2 = 0;
        ResetParmeters();
    }
    ClearCNIF(); // Clear CN interrupt
}
void MeasCurrOffset(int16_t *pOffseta,int16_t *pOffsetb)
{    
    int32_t adcOffsetIa = 0, adcOffsetIb = 0;
    uint16_t i = 0;
                
    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    EnableADCInterrupt();
    
    /* Taking multiple sample to measure voltage offset in all the channels */
    for (i = 0; i < (1<<CURRENT_OFFSET_SAMPLE_SCALER); i++)
    {
        measCurrOffsetFlag = 0;
        /* Wait for the conversion to complete */
        while (measCurrOffsetFlag == 1);
        /* Sum up the converted results */
        adcOffsetIa += ADCBUF_INV_A_IPHASE1;
        adcOffsetIb += ADCBUF_INV_A_IPHASE2;
    }
    /* Averaging to find current Ia offset */
    *pOffseta = (int16_t)(adcOffsetIa >> CURRENT_OFFSET_SAMPLE_SCALER);
    /* Averaging to find current Ib offset*/
    *pOffsetb = (int16_t)(adcOffsetIb >> CURRENT_OFFSET_SAMPLE_SCALER);
    measCurrOffsetFlag = 0;
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    DisableADCInterrupt();
}
