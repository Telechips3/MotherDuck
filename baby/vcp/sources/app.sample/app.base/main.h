// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : main.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/
/* Ultrasonic */
#define ULTRA_TRIG_GPIO       GPIO_GPA(23)
#define ULTRA_ECHO_GPIO       GPIO_GPA(24)
#define ULTRA_ECHO_TIMEOUT    0xFFFFFFFF

/* Buzzer */
#define BUZZER_GPIO           GPIO_GPA(19)

#ifndef MCU_BSP_MAIN_HEADER
#define MCU_BSP_MAIN_HEADER


#define MCU_BSP_SUPPORT_APP_BASE 1

#if ( MCU_BSP_SUPPORT_APP_BASE == 1 )

/*
***************************************************************************************************
*                                             INCLUDE FILES
***************************************************************************************************
*/
#include <sal_internal.h>

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
#define MAIN_UINT_MAX_NUM               (4294967295U)



/*
***************************************************************************************************
*                                             GLOBAL VARIABLES
***************************************************************************************************
*/
extern uint32                           gALiveMsgOnOff;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
extern void cmain
(
    void
);

/* Team 2
*
*
*
*/

#include "speed.h"
#include "interrupt_example.h"
#include "encoder.h"
#include "spi.h"

#include "ultrasonic.h"
#include "buzzer.h"
#include "ipc.h"
#include "steer.h"


#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#endif  //  MCU_BSP_MAIN_HEADER

