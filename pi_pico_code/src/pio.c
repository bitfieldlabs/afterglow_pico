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
#include "pio.h"
#include "hardware/pio.h"
#include "read_col_row.pio.h"
#include "write_col_row.pio.h"
#include "blanking.pio.h"
#include "pindef.h"


//------------------------------------------------------------------------------
// Local data

// Column/row reading and blanking on PIO 0
static PIO sPioColRow = pio0;
static int sSmCol = -1;
static int sSmRow = -1;
static int sSmBlank = -1;
static int sSmDelay = -1;
static int sSmColRowOffset = -1;
static int sSmBlankOffset = -1;
static int sSmDelayOffset = -1;
static uint8_t sColData = 0;
static uint8_t sRowData = 0;
static uint8_t sRowDataPrel = 0; // preliminary row data

// Data output on PIO 1
static PIO sPioOutput = pio1;
static int sSmOut = -1;
static int sSmOutOffset = -1;


//------------------------------------------------------------------------------
bool pio_init()
{
    // Input/Blanking on PIO 0

    // Find a place for the PIO program in the instruction memory
    sSmColRowOffset = pio_add_program(sPioColRow, &read_col_row_program);
    // Claim an unused state machine for the column reading and run the program
    sSmCol = pio_claim_unused_sm(sPioColRow, true);
    read_col_row_program_init(sPioColRow, sSmCol, sSmColRowOffset, AG_PICO_PIN_LCOL_IN);
    // Claim an unused state machine for the row reading and run the program
    sSmRow = pio_claim_unused_sm(sPioColRow, true);
    read_col_row_program_init(sPioColRow, sSmRow, sSmColRowOffset, AG_PICO_PIN_LROW_IN);
    // Claim an unused state machine for the blanking signal handling program
    sSmBlankOffset = pio_add_program(sPioColRow, &blanking_program);
    sSmBlank = pio_claim_unused_sm(sPioColRow, true);
    blanking_program_init(sPioColRow, sSmBlank, sSmBlankOffset);

    // Lamp matrix output on PIO 1

    // Find a place for the PIO program in the instruction memory
    sSmOutOffset = pio_add_program(sPioOutput, &write_col_row_program);
    // Claim an unused state machine for the column reading and run the program
    sSmOut = pio_claim_unused_sm(sPioOutput, true);
    write_col_row_program_init(sPioOutput, sSmOut, sSmOutOffset);

    return ((sSmCol != -1) && (sSmRow != -1) && (sSmBlank != -1) && (sSmOut != -1));
}
//------------------------------------------------------------------------------
uint16_t pio_col_row_data()
{
    // Process all new data
    bool noColData = false;
    bool noRowData = false;
    while (!noColData || !noRowData)
    {
        // Check for new row data
        if (sSmRow != -1)
        {
            noRowData = pio_sm_is_rx_fifo_empty(sPioColRow, sSmRow);
            if (!noRowData)
            {
                uint32_t d = pio_sm_get(sPioColRow, sSmRow);
                sRowDataPrel = (uint8_t)((d >> 23) & 0xff);
            }
        }

        // Check for new column data
        if (sSmCol != -1)
        {
            noColData = pio_sm_is_rx_fifo_empty(sPioColRow, sSmCol);
            if (!noColData)
            {
                uint32_t d = pio_sm_get(sPioColRow, sSmCol);
                uint8_t colData = (uint8_t)(d >> 24);

                // When all columns are HIGH, we're in the middle of anti-ghosting.
                // Otherwise we can sample the new matrix entry.
                if ((colData != 0xff) && (colData != sColData))
                {
                    // adopt the new column/row data
                    sColData = colData;
                    sRowData = sRowDataPrel;
                }
            }
        }
    }

    return ((~sColData << 8) | sRowData);
}

//------------------------------------------------------------------------------
void pio_write_col_row_data(uint8_t colData, uint8_t rowData)
{
    if (sSmOut != -1)
    {
        // Anti-ghosting when all data is zero
        if ((colData == 0) && (rowData == 0))
        {
            // The PIO will check for 0xffffffff
            pio_sm_put(sPioOutput, sSmOut, 0xffffffff);
        }
        else
        {
            // The PIO will read the first 8 bits for the row, the next 8 bits for
            // the column. Data output is inverted (HIGH means off).
            // The unused bits must be set to 0xffff as they are used to reset the
            // data lines.
            uint32_t data = 0xffff0000;
            data |= ~((uint32_t)colData << 8) & 0x0000ff00;
            data |= ~((uint32_t)rowData) & 0x000000ff;
            pio_sm_put(sPioOutput, sSmOut, data);
        }
    }
}

//------------------------------------------------------------------------------
void pio_debug()
{
    uint8_t pcCol = pio_sm_get_pc(sPioColRow, sSmCol) - sSmColRowOffset;
    uint8_t pcRow = pio_sm_get_pc(sPioColRow, sSmRow) - sSmColRowOffset;
    uint8_t pcWrite = pio_sm_get_pc(sPioOutput, sSmOut) - sSmOutOffset;
    printf("PIO pc %d %d %d\n", pcCol, pcRow, pcWrite);
}
