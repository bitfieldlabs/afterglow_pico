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

//------------------------------------------------------------------------------
/* This code assumes following Raspberry pi pico pin layout:
 *
 *  +------------+-----------------+------------+-----------+
 *  | Name       | Function        | Pico Pin#  | Mode      |
 *  +------------+-----------------+------------+-----------+
 *  | DATA_IN    | Data In D0-D7   | GPIO 0-7   | INPUT     |
 *  | COL_IN     | Column Sig In   | GPIO 8     | INPUT     |
 *  | ROW_IN     | Row Sig IN      | GPIO 9     | INPUT     |
 *  | COMB_IN    | Combined Sig    | GPIO 10    | INPUT     |
 *  | BLNK_IN    | Blanking Sig In | GPIO 11    | INPUT     |
 *  | TEST_IN    | Test mode pin   | GPIO 12    | INPUT     |
 *  | DATA_OUT   | Data Out D0-D7  | GPIO 15-22 | OUTPUT    |
 *  | COL_OUT    | Column Sig Out  | GPIO 26    | OUTPUT    |
 *  | ROW_OUT    | Row Sig Out     | GPIO 27    | OUTPUT    |
 *  | PIC_OUT_EN | Pico Out Enable | GPIO 28    | OUTPUT    |
 *  +------------+-----------------+------------+-----------+
*/


// Input
#define AG_PICO_PIN_DATA_IN_D0 15    // First data in pin
#define AG_PICO_PIN_LCOL_IN 8        // Lamp matrix column input
#define AG_PICO_PIN_LROW_IN 9        // Lamp matrix row input
#define AG_PICO_PIN_COMB_SIG 10      // Combined signal input
#define AG_PICO_PIN_BLANKING_IN 11   // Blanking signal input pin
#define AG_PICO_PIN_TESTMODE_IN 12   // Test mode configuration pin

// Output
#define AG_PICO_PIN_OE 13            // Output enable pin
#define AG_PICO_PIN_BLANKING_OUT 14  // Blanking signal output pin
#define AG_PICO_PIN_DATA_OUT_D0 15   // First data out pin
#define AG_PICO_PIN_LCOL_OUT 26      // Lamp matrix column output
#define AG_PICO_PIN_LROW_OUT 27      // Lamp matrix row output
#define AG_PICO_PIN_DATAOE 28        // Data output enable
#define AG_PICO_PIN_WS2812 23        // WS2812 data pin
