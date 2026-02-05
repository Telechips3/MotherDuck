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


#define ENABLE_IPC_TEST             0
#define ENABLE_SPI_TEST             0          
#define ENABLE_IMU_TASK             0       
#define ENABLE_ENCODER_TASK         1   
#define ENABLE_ULTRASONIC_TASK      0 
#define ENABLE_BUZZER_TASK          0   
#define ENABLE_POSE_TASK            0
#define ENABLE_FOLLOW_STEER_TASK    0

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
 


#endif  // ( MCU_BSP_SUPPORT_APP_BASE == 1 )

#endif  //  MCU_BSP_MAIN_HEADER
