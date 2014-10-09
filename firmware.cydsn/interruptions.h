// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//=====================================================     function declaration
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);


void function_scheduler(void);

void analog_measurements(void);
void encoder_reading(void);
void motor_control(void);

void calibration(void);

void pwm_limit_search();

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */