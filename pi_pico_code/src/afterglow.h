/***********************************************************************
 *   ___  ___  ___  ___  ___  ___   _    ___  _ _ _ 
 *  | . || __>|_ _|| __>| . \/  _> | |  | . || | | |
 *  |   || _>  | | | _> |   /| <_/\| |_ | | || | | |
 *  |_|_||_|   |_| |___>|_\_\`____/|___|`___'|__/_/ 
 *                                                  pico
 *      Copyright (c) 2021 bitfield labs
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/bitfieldlabs/afterglow_pico
 *
 *  afterglow is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  afterglow is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with afterglow.
 *  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************/

#include <stdio.h> 

//------------------------------------------------------------------------------
// Setup

#define AFTERGLOW_PICO_VERSION 101      // Afterglow pico version number
#define BOARD_REV                13     // Latest supported Afterglow Pico board revision. Currently v1.3
#define SINGLE_UPDATE_CONS        2     // Number of consistent data samples required for matrix update. Helps prevent ghosting.
#define TTAG_INT_A              250     // Matrix update time interval, config A [us]
#define TTAG_INT_B              500     // Matrix update time interval, config B [us]
#define PWM_STEPS_A               8     // Number of brightness steps, config A (only 4, 8 an 16 are currently supported)
#define PWM_STEPS_B               8     // Number of brightness steps, config B (only 4, 8 an 16 are currently supported)
#define PWM_STEPS                 8     // Number of brightness steps (only 4, 8 an 16 are currently supported)
#define ANTIGHOST_DURATION       20     // Duration of anti ghosting [us] (turning off all lamps briefly)
#define DEFAULT_GLOWDUR         140     // Default glow duration [ms]
#define DEFAULT_BRIGHTNESS        7     // Default maximum lamp brightness 0-7
#define DEBUG_SERIAL              1     // Turn debug output via serial on/off
#define PROJECT_BUTTER            1     // Smooth as butter brightness transitions
#define RGB_LED                   1     // Use the RGB LED as status display


// Initialize the afterglow engine
void ag_init();

// Update the afterglow
// This function is called every time interval
void ag_update();

// Afterglow serial communication
// This function must be called frequently in order to process pending
// serial communication events.
void ag_sercomm();
