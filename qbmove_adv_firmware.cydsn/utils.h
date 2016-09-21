// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file 		utils.h
*
* \brief 		Declaration of utility functions.
* \date 		Feb 16, 2014
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/


#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <globals.h>

//--------------------------------------------------------------     DEFINITIONS

#define ALPHA 3     // current filters
#define BETA  300   // velocity filters

//#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))

#define SIGN(A) (((A) >= 0) ? (1) : (-1))


//-------------------------------------------------------------     DECLARATIONS

int32 filter_i1(int32 value);
int32 filter_i2(int32 value);

int32 filter_vel_1(int32 value);
int32 filter_vel_2(int32 value);
int32 filter_vel_3(int32 value);

uint8 LCRChecksum(uint8 *data_array, uint8 data_length);

CYBIT check_enc_data(const uint32*);

int8 get_pwm_sign_1(int32);
int8 get_pwm_sign_2(int32);

void reset_counters();

#endif

/* [] END OF FILE */