
// Copyright (c) 2017, Mattia Poggiani.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \file        exosense_communications.h
 *
 * \brief       Library of functions for SERIAL PORT communication with ExoSense device.
 *              Function Prototypes.
 *
 * \details
 *
 *  This library contains all necessary functions for communicating with ExoSense when
 *  using a USB to RS485 connector that provides a Virtual COM interface.
**/

 /**
* \mainpage     qbAPI Libraries
*
* \brief        Those functions allows to use ExoSense device through a serial port
*
* \version      6.1.0
*
* \author       Mattia Poggiani
*
* \date         January 11, 2017
*
* \details      This is a set of functions that allows to use ExoSense device  
*               via a serial port.
*/

#ifndef EXOSENSE_SERIALPORT_H_INCLUDED
#define EXOSENSE_SERIALPORT_H_INCLUDED

#include "qbmove_communications.h"


//======================================================     commGetExoSenseMeasurements

/** This function gets position measurements from ExoSense connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  measurements        Measurements.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       measurements[6];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetExoSenseMeasurements(&comm_settings_t, DEVICE_ID, measurements))
        printf("Measurements: %d\t%d\t%d\t%d\t%d\t%d\n",measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5]);
    else
        puts("Couldn't retrieve measurements.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetExoSenseMeasurements( comm_settings *comm_settings_t, int id, short int measurements[6] );
			   


// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */
