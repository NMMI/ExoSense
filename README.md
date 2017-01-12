/*==========================================================================
* ExoSense project
* Author: Mattia Poggiani
* Centro di Ricerca "E. Piaggio" - University of Pisa
* http://www.centropiaggio.unipi.it
* http://www.naturalmachinemotioninitiative.com/
* For support, please refer to: mattia.poggiani at centropiaggio.unipi.it
*==========================================================================/

ExoSense
===============
The repository is organized as follows:

1) Firmware

Here you can find the PSoC firmware to be programmed in the ExoSense boards.
It handles the readings of 6 encoders connected to a single board.

Just open the project with PSoC Creator and upload it onto the board


-------------------------------------------------------------------------------------
2) Management

Here are useful libraries to manage the ExoSense device.
In particular:
- qbAPI contains the API to communicate with the device through FTDI and RS-485 protocol.
- qbadmin contains softwares to interface with the board using the API.
- exosense_simulink contains Simulink libraries.

Simulink usage:
Install exosense_simulink libraries in addition to standard qbmove_simulink libraries 
by running 'install.m' file in Matlab (newer than version 2015a).
Use the provided Simulink schemes as an example on how to use the library.