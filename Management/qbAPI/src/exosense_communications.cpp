
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
 *  \file       exosense_communications.cpp
 *
 *  \brief      Library of functions for serial port communication with ExoSense
 *
 *  \details
 *
 *  Check the \ref exosense_communications.h "exosense_communications.h" file
 *  for a complete description of the public functions implemented in
 *  exosense_communications.cpp.
**/

//=================================================================     includes

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdint.h>
#include <ctype.h>
#include <time.h>

#if (defined(_WIN32) || defined(_WIN64))
    #include <windows.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64))
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
    #include <sys/ioctl.h>
    #include <dirent.h>
    #include <sys/time.h>
    #include <stdlib.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__))
    #include <linux/serial.h>
#endif

#if (defined(__APPLE__))
    #include <IOKit/IOKitLib.h>
    #include <IOKit/serial/IOSerialKeys.h>
    #include <IOKit/serial/ioss.h>
    #include <IOKit/IOBSD.h>
#endif

#include "exosense_communications.h"


#define BUFFER_SIZE 500    ///< Size of buffers that store communication packets

//===========================================     public fuctions implementation

/// @cond C_FILES


//==============================================================================
//                                                   commGetExoSenseMeasurements
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetExoSenseMeasurements(comm_settings *comm_settings_t, int id, short int measurements[]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_GET_MEASUREMENTS;             // command
    data_out[5] = CMD_GET_MEASUREMENTS;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size < 0)
        return package_in_size;

//==============================================================	 get packet

    switch ((package_in_size - 2) >> 1) {
        case 1:
            ((char *) &measurements[0])[0] = package_in[2];
            ((char *) &measurements[0])[1] = package_in[1];
        break;
        case 2:
            ((char *) &measurements[0])[0] = package_in[2];
            ((char *) &measurements[0])[1] = package_in[1];
            ((char *) &measurements[1])[0] = package_in[4];
            ((char *) &measurements[1])[1] = package_in[3];
        break;
        case 3:
            ((char *) &measurements[0])[0] = package_in[2];
            ((char *) &measurements[0])[1] = package_in[1];
            ((char *) &measurements[1])[0] = package_in[4];
            ((char *) &measurements[1])[1] = package_in[3];
            ((char *) &measurements[2])[0] = package_in[6];
            ((char *) &measurements[2])[1] = package_in[5];
        break;
        case 4:
            ((char *) &measurements[0])[0] = package_in[2];
            ((char *) &measurements[0])[1] = package_in[1];
            ((char *) &measurements[1])[0] = package_in[4];
            ((char *) &measurements[1])[1] = package_in[3];
            ((char *) &measurements[2])[0] = package_in[6];
            ((char *) &measurements[2])[1] = package_in[5];
            ((char *) &measurements[3])[0] = package_in[8];
            ((char *) &measurements[3])[1] = package_in[7];
        break;
		case 5:
            ((char *) &measurements[0])[0] = package_in[2];
            ((char *) &measurements[0])[1] = package_in[1];
            ((char *) &measurements[1])[0] = package_in[4];
            ((char *) &measurements[1])[1] = package_in[3];
            ((char *) &measurements[2])[0] = package_in[6];
            ((char *) &measurements[2])[1] = package_in[5];
            ((char *) &measurements[3])[0] = package_in[8];
            ((char *) &measurements[3])[1] = package_in[7];
			((char *) &measurements[4])[0] = package_in[10];
            ((char *) &measurements[4])[1] = package_in[9];
        break;
		case 6:
            ((char *) &measurements[0])[0] = package_in[2];
            ((char *) &measurements[0])[1] = package_in[1];
            ((char *) &measurements[1])[0] = package_in[4];
            ((char *) &measurements[1])[1] = package_in[3];
            ((char *) &measurements[2])[0] = package_in[6];
            ((char *) &measurements[2])[1] = package_in[5];
            ((char *) &measurements[3])[0] = package_in[8];
            ((char *) &measurements[3])[1] = package_in[7];
			((char *) &measurements[4])[0] = package_in[10];
            ((char *) &measurements[4])[1] = package_in[9];
			((char *) &measurements[5])[0] = package_in[12];
            ((char *) &measurements[5])[1] = package_in[11];
        break;
        default:
            printf("Number of sensors not supported.\n");
            return -1; 
        break;
    }

    return ((package_in_size - 2) >> 1);
}




/// @endcond

/* [] END OF FILE */
