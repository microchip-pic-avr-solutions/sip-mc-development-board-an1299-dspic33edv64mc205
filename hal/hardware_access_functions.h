/**
  @File Name:
    hardware_access_functions.h

  @Summary:
    This module provides hardware access function support.

  @Description:
    This module provides hardware access function support.
 */
/* ********************************************************************
 * (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(TM) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
* *****************************************************************************/
#ifndef __HAF_H
#define __HAF_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "uart2.h"
#include "bsp.h"
#include "delay.h"
#include "hardware_access_functions_types.h"
#include "hardware_access_functions_params.h"

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif


/**
  Section: Hardware Access Functions
 */     

/**
 * Configures the board and initializes any state depenendent data that is used
 * in the HAL. This function needs to be called once every 1ms.
 * Summary: Configures the board and initializes any state depenendent data that is used in the HAL.
 * @example
 * <code>
 * while (HAL_Board_Configure()==BOARD_NOT_READY);
 * <code>
 */
HAL_BOARD_STATUS HAL_Board_Configure(void);

/**
 * Runs the board service routines. This function needs to be called once every 1ms.
 * Summary: Runs the board service routines.
 * @example
 * <code>
 * if (HAL_Board_Service()!=BOARD_READY)
 * {
 *     // do something
 * }
 * <code>
 */
HAL_BOARD_STATUS HAL_Board_Service(void);

/**
 * This function can  be called to clear the fault.
 * Summary: Clears Gate Driver Fault Status
 * @example
 * <code>
 * HAL_Board_Board_FaultClear();
 * <code>
 */
void HAL_Board_Board_FaultClear(void);

extern  GATE_DRIVER_OBJ inverterGateDriver[BSP_GATE_DRIVER_INSTANCE_COUNT];
#ifdef __cplusplus
}
#endif

#endif /* __HAF_H */


/**
 End of File
 */