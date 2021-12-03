/***********************************************************************
 *  afterglow pico:
 *      Copyright (c) 2021 bitfield labs
 *
 ***********************************************************************
 *  This file is part of the afterglow pinball LED project:
 *  https://github.com/smyp/afterglow
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

// Input
#define AG_PICO_PIN_LCOL_IN 8        // Lamp matrix column input
#define AG_PICO_PIN_LROW_IN 9        // Lamp matrix row input
#define AG_PICO_PIN_COMB_SIG 10      // Combined signal input
#define AG_PICO_PIN_BLANKING_IN 11   // Blanking signal input pin

// Output
#define AG_PICO_PIN_OE 13            // Output enable pin
#define AG_PICO_PIN_BLANKING_OUT 14  // Blanking signal output pin
#define AG_PICO_PIN_LCOL_OUT 26      // Lamp matrix column output
#define AG_PICO_PIN_LROW_OUT 27      // Lamp matrix row output
#define AG_PICO_PIN_DATAOE 28        // Data output enable
