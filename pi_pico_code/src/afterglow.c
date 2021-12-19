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
 *  https://github.com/smyp/afterglow_pico
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

#include <string.h>
#include "afterglow.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pio.h"
#include "pindef.h"


//------------------------------------------------------------------------------
// Setup

// Afterglow version number
#define AFTERGLOW_PICO_VERSION 100

// Afterglow configuration version
#define AFTERGLOW_CFG_VERSION 1

// Afterglow Pico board revision. Currently v1.0
#define BOARD_REV 10

// turn debug output via serial on/off
#define DEBUG_SERIAL 1

// Number of consistent data samples required for matrix update
#define SINGLE_UPDATE_CONS 2

// original matrix update interval [us]
#define ORIG_INT (2000)

// local time interval, config A [us]
#define TTAG_INT_A (250)

// cycles per original interval, config A
#define ORIG_CYCLES_A (ORIG_INT / TTAG_INT_A)

// local time interval, config B [us]
#define TTAG_INT_B (500)

// cycles per original interval, config B
#define ORIG_CYCLES_B (ORIG_INT / TTAG_INT_B)

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 8

// default glow duration [ms]
#define DEFAULT_GLOWDUR 140

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10

// default maximum lamp brightness 0-7
#define DEFAULT_BRIGHTNESS 7

// test mode setup
#define TEST_MODE_NUMMODES 7    // number of test modes
#define TEST_MODE_DUR 8         // test duration per mode [s]
#define TESTMODE_INT (500)      // test mode lamp switch interval [ms]
#define TESTMODE_CYCLES_A ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)TTAG_INT_A) // number of cycles per testmode interval, config A
#define TESTMODE_CYCLES_B ((uint32_t)TESTMODE_INT * 1000UL / (uint32_t)TTAG_INT_B) // number of cycles per testmode interval, config B


//------------------------------------------------------------------------------
// serial port protocol definition

// write buffer size [bytes]
#define AG_CMD_WRITE_BUF 32

// command terminator character
#define AG_CMD_TERMINATOR ':'

// version poll command string
#define AG_CMD_VERSION_POLL "AGV"

// configuration poll command string
#define AG_CMD_CFG_POLL "AGCP"

// configuration save command string
#define AG_CMD_CFG_SAVE "AGCS"

// configuration reset to default command string
#define AG_CMD_CFG_DEFAULT "AGCD"

// data ready string
#define AG_CMD_CFG_DATA_READY "AGDR"

// acknowledge string
#define AG_CMD_ACK "AGCACK"

// NOT acknowledge string
#define AG_CMD_NACK "AGCNACK"


//-------------#include <stdio.h> -----------------------------------------------------------------
// function prototypes

int loadCfg(int *pErr);
void setDefaultCfg();
void saveCfgToEEPROM();
void applyCfg();
void driveLampMatrixPassThrough();
void driveLampMatrix();
uint16_t testModeInput(void);
bool updateValid(uint8_t inColMask, uint8_t inRowMask);
void updateCol(uint32_t col, uint8_t rowMask);
uint32_t calculateCRC32(const uint8_t *data, uint16_t length);
void dataOutput(uint8_t colData, uint8_t rowData);
void updateMx(uint16_t *pMx, bool on, uint16_t step);
void debugInputs(uint8_t inColMask, uint8_t inRowMask);
void debugOutput(uint8_t outColMask, uint8_t outRowMask);


//------------------------------------------------------------------------------
// global variables

// Lamp matrix 'memory'
static uint16_t sMatrixState[NUM_COL][NUM_ROW];
    
// local time
static uint32_t sTtag = 0;

// interrupt runtime counters [cycles]
static int64_t sLastIntTime = 0;
static int64_t sMaxIntTime = 0;
static volatile uint16_t sOverflowCount = 0;

// remember the last column and row samples
static uint8_t sLastColMask = 0;
static uint8_t sLastRowMask = 0;

#if DEBUG_SERIAL
static uint8_t sLastOutColMask = 0;
static uint8_t sLastOutRowMask = 0;
static uint32_t sBadColCounter = 0;
static uint32_t sBadColOrderCounter = 0;
static uint8_t sLastBadCol = 0;
static uint8_t sLastGoodCol = 0;
static int sMaxCurr = 0;
static int sLastCurr = 0;
#endif

// afterglow configuration data definition
typedef struct AFTERGLOW_CFG_s
{
    uint16_t version;                         // afterglow version of the configuration
    uint16_t res;                             // reserved bytes
    uint8_t lampGlowDur[NUM_COL][NUM_ROW];    // Lamp matrix glow duration configuration [ms * GLOWDUR_CFG_SCALE]
    uint8_t lampBrightness[NUM_COL][NUM_ROW]; // Lamp matrix maximum brightness configuration (0-7)
    uint32_t crc;                             // data checksum
} AFTERGLOW_CFG_t;

// afterglow configuration
static AFTERGLOW_CFG_t sCfg;

// afterglow test mode configuration structure
typedef struct AFTERGLOW_TESTMODE_s
{
    bool testMode;
    bool passThrough;
    bool freq2kHz;      // 2kHz mode (instead of 4kHz)
} AFTERGLOW_TESTMODE_t;

// afterglow test mode configuration
static AFTERGLOW_TESTMODE_t sTstModeCfg;

// precalculated glow steps for each lamp
static uint16_t sGlowSteps[NUM_COL][NUM_ROW];

// precalculated maximum subcycle for lamp activation (brightness)
static uint8_t sMaxSubcycle[NUM_COL][NUM_ROW];


//------------------------------------------------------------------------------
void ag_init()
{
    // initialize the data
    memset(sMatrixState, 0, sizeof(sMatrixState));
    memset(&sTstModeCfg, 0, sizeof(sTstModeCfg));

    // load the configuration from EEPROM
    int err;
    bool cfgLoaded = loadCfg(&err);
    if (cfgLoaded == false)
    {
        // set default configuration
        setDefaultCfg();

        // store the configuration to EEPROM
        saveCfgToEEPROM();
    }

    // Apply the configuration
    // This will prepare all values for the interrupt handlers.
    applyCfg();

    // enable serial output at 115200 baudrate
    printf("afterglow pico v%d  (c) 2021 bitfield labs\n", AFTERGLOW_PICO_VERSION);

#if DEBUG_SERIAL
    printf("CFG from %s", cfgLoaded ? "EEPROM" : "DEFAULT");
    if (err)
    {
        printf(" err %d", err);
    }
    printf("\n");
#endif
}

//------------------------------------------------------------------------------
void start()
{

}

//------------------------------------------------------------------------------
void stop()
{

}

//------------------------------------------------------------------------------
// This is the realtime task update. All the afterglow magic happens here.
void ag_update()
{   
    // time is running
    absolute_time_t startCnt = get_absolute_time();
    sTtag++;

    // read the test mode configuration
    sTstModeCfg.testMode = !gpio_get(AG_PICO_PIN_TESTMODE_IN);

    // kick the dog
    // todo
    //wdt_reset();

    // Drive the lamp matrix
    // This is done before updating the matrix to avoid having an irregular update
    // frequency due to varying update calculation times.
    if (sTstModeCfg.passThrough)
    {
        // pass-through mode
        driveLampMatrixPassThrough();
    }
    else
    {
        // afterglow mode
        driveLampMatrix();
    }

    // get lamp matrix input from the PIO
    uint16_t inData = pio_col_row_data();
    bool validInput = true;

    // testmode input simulation (jumper J1 active)
    if (sTstModeCfg.testMode)
    {
        // test mode
        inData = testModeInput();
    }

    uint8_t inColMask = (inData >> 8);    // LSB is col 0, MSB is col 7
    uint8_t inRowMask = ~(uint8_t)inData;  // LSB is row 0, MSB is row 7, HIGH means off

    // evaluate the column reading
    // only one bit should be set as only one column can be active at a time
    uint32_t inCol = NUM_COL;
    switch (inColMask)
    {
        case 0x01: inCol = 0; break;
        case 0x02: inCol = 1; break;
        case 0x04: inCol = 2; break;
        case 0x08: inCol = 3; break;
        case 0x10: inCol = 4; break;
        case 0x20: inCol = 5; break;
        case 0x40: inCol = 6; break;
        case 0x80: inCol = 7; break;
        default:
        {
#if DEBUG_SERIAL
            sBadColCounter++;
            sLastBadCol = inColMask;
#endif
            validInput = false;
        }
        break;
    }

    // The matrix is updated only once per original column cycle. The code
    // waits for a number of consecutive consistent information before updating the matrix.
    validInput &= updateValid(inColMask, inRowMask);

    // Update only with a valid input. If the input is invalid the current
    // matrix state is left unchanged.
    if (validInput)
    {
        // update the current column
        updateCol(inCol, inRowMask);

#if DEBUG_SERIAL
        if ((inCol != (sLastGoodCol+1)) && (inCol!=(sLastGoodCol-7)))
        {
            sBadColOrderCounter++;
        }
        sLastGoodCol = inCol;
#endif
    }

    // remember the last column and row samples
    sLastColMask = inColMask;
    sLastRowMask = inRowMask;

    // how long did it take?
    sLastIntTime = absolute_time_diff_us(startCnt, get_absolute_time());
    if (sLastIntTime > sMaxIntTime)
    {
        sMaxIntTime = sLastIntTime;
    }
}

//------------------------------------------------------------------------------
void ag_sercomm()
{
    // count the loops (used for debug output below)
    static uint32_t loopCounter = 0;
    loopCounter++;
/*
    // check for serial data
    static String cmd = "";
    static bool complete = false;
    while (Serial.available() && (complete == false))
    {
        char character = Serial.read();
        if (character != AG_CMD_TERMINATOR)
        {
            // add the character and wait for the command terminator
            cmd.concat(character);
        }
        else
        {
            // command complete
            complete = true;
        }
    }

    // handle complete commands
    if (complete)
    {
        // version poll
        if (cmd == AG_CMD_VERSION_POLL)
        {
            // Output the version numbers
            Serial.print(AG_CMD_VERSION_POLL);
            Serial.print(" ");
            Serial.print(AFTERGLOW_VERSION);
            Serial.print(" ");
            Serial.println(AFTERGLOW_CFG_VERSION);
        }

        // configuration poll
        else if (cmd == AG_CMD_CFG_POLL)
        {
            // send the full confiuration
            sendCfg();
        }

        // configuration reset
        else if (cmd == AG_CMD_CFG_DEFAULT)
        {
            // reset the configuration to default
            defaultCfg();
        }

        // configuration write
        else if (cmd == AG_CMD_CFG_SAVE)
        {
            // stop the matrix updates
            stop();

            // receive a new configuration
            receiveCfg();

            // resume operation
            start();
        }

        cmd = "";
        complete = false;
    }

    // watch out for interval configuration changes
    if ((PINB & B00000100) != (sLastPINB & B00000100))
    {
        // reinitialize the timers
        noInterrupts();
        timerSetup();
        sTtag = 0;
        sLastPINB = PINB;
        interrupts();
#if DEBUG_SERIAL
        Serial.print("New TTAG_INT: ");
        Serial.println((PINB & B00000100) ? TTAG_INT_A : TTAG_INT_B);
#endif
    }
*/
#if DEBUG_SERIAL
    if ((loopCounter % 5) == 0)
    {
        // print the maximum interrupt runtime
        if (sTstModeCfg.testMode)
        {
            printf("TESTMODE!\n");
        }
        if (sTstModeCfg.passThrough)
        {
            printf("PASS THROUGH!\n");
        }
        printf("TTAG_INT %d\n", sTstModeCfg.freq2kHz ? TTAG_INT_B : TTAG_INT_A);
        printf("INT dt max %lldus last %lldus ovfl %d\n", sMaxIntTime, sLastIntTime, sOverflowCount);
        printf("Bad col: %ld col %d ord %ld last good: %d\n", sBadColCounter, sLastBadCol, sBadColOrderCounter, sLastGoodCol);
        // data debugging
        debugInputs(sLastColMask, sLastRowMask);
        debugOutput(sLastOutColMask, sLastOutRowMask);
        // dump the full matrix
        for (uint32_t c=0; c<NUM_COL; c++)
        {
            printf("C%lu + ", c);
            for (uint32_t r=0; r<NUM_ROW; r++)
            {
                printf("%d", sMatrixState[c][r]);
                printf(" ");
            }
            printf("\n");
        }
    }
#endif
}

//------------------------------------------------------------------------------
void updateMx(uint16_t *pMx, bool on, uint16_t step)
{
    if (on)
    {
        // increase the stored brightness value
        if (*pMx < (65535 - step))
        {
            *pMx += step;
        }
        else
        {
            *pMx = 0xffff;
        }
    }
    else
    {
        // decrease the stored brightness value
        if (*pMx > step)
        {
            *pMx -= step;
        }
        else
        {
            *pMx = 0;
        }
    }
}

//------------------------------------------------------------------------------
void updateCol(uint32_t col, uint8_t rowMask)
{
    // paranoia check
    if (col >= NUM_COL)
    {
        return;
    }
    
    // get a pointer to the matrix column
    uint16_t *pMx = &sMatrixState[col][0];
    const uint16_t *pkStep = &sGlowSteps[col][0];

    // update all row values
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // update the matrix value
        updateMx(pMx, (rowMask & 0x01), *pkStep);

        // next row
        pMx++;
        pkStep++;
        rowMask >>= 1;
    }
}

//------------------------------------------------------------------------------
void driveLampMatrixPassThrough()
{
    static uint8_t sLastPassThroughColMask = 0;
    static uint8_t sLastPassThroughRowMask = 0;

    // only update when changed
    if ((sLastColMask != sLastPassThroughColMask) ||
        (sLastRowMask != sLastPassThroughRowMask))
    {
        // update the output
        dataOutput(sLastColMask, sLastRowMask);

        // remember the new state
        sLastPassThroughColMask = sLastColMask;
        sLastPassThroughRowMask = sLastRowMask;
    }
}

//------------------------------------------------------------------------------
void driveLampMatrix()
{   
    // turn off everything briefly to avoid ghosting
    dataOutput(0x00, 0x00);

    // Wait around 34us. This is about the same time the original WPC anti
    // ghosting ROM does, so it's probably matching the output hardware quite well.
    busy_wait_us(34);

    // check which column we're currently updating
    uint32_t outCol = (sTtag % NUM_COL);

    // The original cycle is divided into ORIG_CYCLES column sub cycles.
    // These cycles are used to do PWM in order to adjust the lamp brightness.
    //
    // Illustration with ORIG_CYCLES==4 and four brightness steps B1-B4 and off (B0):
    //
    // * Lamp on
    //                      2ms 2ms ...
    // Orig col            1   2   3   4   5   6   7   8   1   2   3   4   5   6
    // afterglow col       12345678123456781234567812345678123456781234567812345
    // col cycle           1       2       3       4       1       2       3
    //
    // Brightness 1        *                               *
    // Brightness 2        *       *                       *       *
    // Brightness 3        *       *       *               *       *       *
    // Brightness 4        *       *       *       *       *       *       *
    uint32_t colCycle = (!sTstModeCfg.freq2kHz) ?
        ((sTtag / NUM_COL) % ORIG_CYCLES_A) :
        ((sTtag / NUM_COL) % ORIG_CYCLES_B);

    // prepare the data
    // LSB is row/col 0, MSB is row/col 7
    uint8_t colData = (1 << outCol);
    uint8_t rowData = 0;
    uint16_t *pMx = &sMatrixState[outCol][0];
    uint8_t *pMaxSubCycle = &sMaxSubcycle[outCol][0];
    for (uint32_t r=0; r<NUM_ROW; r++)
    {
        // make room for the next bit
        rowData >>= 1;
        
        // nothing to do if the matrix value is zero (off)
        if (*pMx)
        {
            uint16_t subCycle = (!sTstModeCfg.freq2kHz) ?
                (*pMx / (65536 / ORIG_CYCLES_A)) :
                (*pMx / (65536 / ORIG_CYCLES_B));

            // limit to the configured maximum brightness
            if (subCycle > *pMaxSubCycle)
            {
                subCycle = *pMaxSubCycle;
            }

            // Lamps are turned on when the value in the matrix is not zero
            // and when the value is high enough for the current sub cycle.
            if (subCycle >= colCycle)
            {
                rowData |= 0x80;
            }
        }
        pMx++;
        pMaxSubCycle++;
    }

    // output the data
    dataOutput(colData, rowData);
#if DEBUG_SERIAL
    sLastOutColMask = colData;
    sLastOutRowMask = rowData;
#endif
}

//------------------------------------------------------------------------------
void dataOutput(uint8_t colData, uint8_t rowData)
{
    // data output is handled by PIO
    pio_write_col_row_data(colData, rowData);
}

//------------------------------------------------------------------------------
uint16_t testModeInput(void)
{
    // simulate the original column cycle
    uint8_t col = (!sTstModeCfg.freq2kHz) ?
        ((sTtag / ORIG_CYCLES_A) % NUM_COL) :
        ((sTtag / ORIG_CYCLES_B) % NUM_COL);
    uint8_t colMask = (1 << col);

    // populate the row
    uint8_t rowMask = 0;
   
    // loop through all available modes
    uint8_t m = (!sTstModeCfg.freq2kHz) ?
        (sTtag / (TEST_MODE_DUR * 1000000UL / TTAG_INT_A)) :
        (sTtag / (TEST_MODE_DUR * 1000000UL / TTAG_INT_B));
    uint32_t tmp = (!sTstModeCfg.freq2kHz) ?
        (sTtag / TESTMODE_CYCLES_A) :
        (sTtag / TESTMODE_CYCLES_B);
    switch (m % TEST_MODE_NUMMODES)
    {
        case 0:
        // cycle all columns
        {
            uint8_t c = (tmp % NUM_COL);
            if (c == col)
            {
                rowMask = 0xff;
            }
        }
        break;
        case 1:
        // cycle all rows
        {
            uint8_t r = (tmp % NUM_ROW);
            rowMask |= (1 << r);
        }
        break;
        case 2:
        // cycle all columns (inverted)
        {
            uint8_t c = (tmp % NUM_COL);
            if (c != col)
            {
                rowMask = 0xff;
            }
        }
        break;
        case 3:
        // cycle all rows (inverted)
        {
            uint8_t r = (tmp % NUM_ROW);
            rowMask = ~(1 << r);
        }
        break;
        case 4:
        // blink all lamps
        {
            if (tmp % 2)
            {
                rowMask = 0xff;
            }
        }
        break;
        case 5:
        // switch between even and odd lamps
        // turn on every other column
        {
            if (col % 2 == (tmp % 2))
            {
                rowMask = 0x55;
                if (tmp % 3)
                {
                    rowMask <<= 1;
                }
            }
        }
        break;
        case 6:
        // cycle through all lamps individually with 4x speed
        {
            uint8_t l = (uint8_t)((tmp * 4) % (NUM_COL * NUM_ROW));
            uint8_t c = (l / NUM_ROW);
            uint8_t r = (l % NUM_COL);
            if (c == col)
            {
                rowMask = (1 << r);
            }
        }
        break;
        default:
        break;
    }

    // invert the row mask as in the original input HIGH means off
    rowMask = ~rowMask;

    return ((colMask << 8) | rowMask);
}

//------------------------------------------------------------------------------
bool updateValid(uint8_t inColMask, uint8_t inRowMask)
{
    static uint8_t sConsistentSamples = 0;
    static uint8_t sLastUpdColMask = 0x00;
    bool valid = false;

    // check if the current column has not been handled already
    if (inColMask != sLastUpdColMask)
    {
        // reset the counter when the data changes
        if ((inColMask != sLastColMask) || (inRowMask != sLastRowMask))
        {
            sConsistentSamples = 0;
        }
        // count number of consecutive samples with consistent data
        else if (sConsistentSamples < 255)
        {
            sConsistentSamples++;
        }

        // The matrix is updated only once per original column cycle.
        // The code waits for a number of consecutive consistent information
        // before updating the matrix.
        // This also avoids ghosting issues, see
        // https://emmytech.com/arcade/led_ghost_busting/index.html for details.
        if (sConsistentSamples >= (SINGLE_UPDATE_CONS-1))
        {
            sLastUpdColMask = inColMask;
            valid = true;
        }
    }
    return valid;
}

//------------------------------------------------------------------------------
void applyCfg()
{
    // calculate the glow steps and maximum subcycles
    uint16_t *pGS = &sGlowSteps[0][0];
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    uint8_t *pMaxSubCycle = &sMaxSubcycle[0][0];
    for (uint8_t c=0; c<NUM_COL; c++)
    {
        for (uint8_t r=0; r<NUM_COL; r++)
        {
            // brightness step per lamp matrix update (assumes one update per original matrix step)
            uint32_t glowDur = (*pGlowDur * GLOWDUR_CFG_SCALE);
            *pGS++ = (glowDur > 0) ?
                ((uint16_t)(65535 / ((glowDur * 1000) / ORIG_INT)) * NUM_COL) : 0xffff;

            // translate maximum brightness into maximum lamp driving subcycle
            *pMaxSubCycle++ = (!sTstModeCfg.freq2kHz) ?
                (*pBrightness >> (8/ORIG_CYCLES_A-1)) :
                (*pBrightness >> (8/ORIG_CYCLES_B-1));

            // next
            pGlowDur++;
            pBrightness++;
        }
    }
}

//------------------------------------------------------------------------------
void setDefaultCfg()
{
    // initialize configuration to default values
    memset(&sCfg, 0, sizeof(sCfg));
    sCfg.version = AFTERGLOW_CFG_VERSION;
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    for (uint8_t c=0; c<NUM_COL; c++)
    {
        for (uint8_t r=0; r<NUM_ROW; r++)
        {
            *pGlowDur++ = (DEFAULT_GLOWDUR / GLOWDUR_CFG_SCALE);
            *pBrightness++ = DEFAULT_BRIGHTNESS;
        }
    }

    // calculate the crc
    uint16_t cfgSize = sizeof(sCfg);
    sCfg.crc = calculateCRC32((uint8_t*)&sCfg, cfgSize-sizeof(sCfg.crc));
}

//------------------------------------------------------------------------------
void defaultCfg()
{
    // set the default configuration
    setDefaultCfg();

    // send the acknowledge
    printf(AG_CMD_ACK);
}

//------------------------------------------------------------------------------
int loadCfg(int *pErr)
{
    bool valid = false;
    *pErr = 0;

    // load the configuration from the EEPROM
    /*
    uint16_t cfgSize = sizeof(sCfg);
    uint8_t *pCfg = (uint8_t*)&sCfg;
    for (uint16_t i=0; i<cfgSize; i++)
    {
        *pCfg++ = EEPROM.read(i);
    }

    // check the version
    if (sCfg.version == AFTERGLOW_CFG_VERSION)
    {
        // check the CRC of the data
        uint32_t crc = calculateCRC32((uint8_t*)&sCfg, cfgSize-sizeof(sCfg.crc));
        if (crc == sCfg.crc)
        {
            valid = true;
        }
        else
        {
            *pErr = 2;
        }
    }
    else
    {
        *pErr = 1;
    }
    */

    return valid;
}

//------------------------------------------------------------------------------
uint32_t calculateCRC32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xffffffff;
    while (length--)
    {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1)
        {
            bool bit = crc & 0x80000000;
            if (c & i)
            {
                bit = !bit;
            }
            crc <<= 1;
            if (bit)
            {
                crc ^= 0x04c11db7;
            }
        }
    }
    return crc;
}

//------------------------------------------------------------------------------
void sendCfg()
{
    /*
    // send the whole configuration structure via serial port
    uint16_t cfgSize = sizeof(sCfg);
    const byte *pkCfg = (const byte*)&sCfg;
    Serial.write(pkCfg, cfgSize);
    */
}

//------------------------------------------------------------------------------
void receiveCfg()
{
    /*
    // wait for the full configuration data
    bool res = false;
    AFTERGLOW_CFG_t cfg;
    uint8_t *pCfg = (uint8_t*)&cfg;
    uint16_t cfgSize = sizeof(cfg);
    uint16_t size = 0;

    // read all data
    while (size < cfgSize)
    {
        // send data ready signal and wait for data
        Serial.print(AG_CMD_CFG_DATA_READY);
        delay(200);

        // read data
        uint32_t readBytes = 0;
        while ((Serial.available()) && (readBytes < AG_CMD_WRITE_BUF) && (size < cfgSize))
        {
            *pCfg++ = Serial.read();
            readBytes++;
            size++;
        }
    }

    if (size == sizeof(cfg))
    {
        // check the crc
        uint32_t crc = calculateCRC32((uint8_t*)&cfg, size-sizeof(cfg.crc));
        if (crc == cfg.crc)
        {
             // set the new configuration and apply it
            memcpy(&sCfg, &cfg, size);
            applyCfg();

            // store the configuration to EEPROM
            saveCfgToEEPROM();

            res = true;
        }
#if DEBUG_SERIAL
        else
        {
            Serial.print("CRC FAIL ");
            Serial.print(crc);
            Serial.print(" ");
            Serial.println(cfg.crc);
        }
#endif
    }
#if DEBUG_SERIAL
    else
    {
            Serial.print("SIZE MISMATCH: ");
            Serial.println(size);
    }
#endif

    // send ACK/NACK
    Serial.print(res ? AG_CMD_ACK : AG_CMD_NACK);
    */
}

//------------------------------------------------------------------------------
void saveCfgToEEPROM()
{
    /*
    const uint8_t *pkCfg = (const uint8_t*)&sCfg;
    for (uint16_t i=0; i<sizeof(sCfg); i++)
    {
        EEPROM.write(i, *pkCfg++);
    }
    Serial.print("EEPROM write ");
    Serial.println(sizeof(sCfg));
    */
}

#if DEBUG_SERIAL
//------------------------------------------------------------------------------
void debugInputs(uint8_t inColMask, uint8_t inRowMask)
{
    // output the data
    printf("IN C 0x%02X R 0x%02X\n", inColMask, inRowMask);
}

//------------------------------------------------------------------------------
void debugOutput(uint8_t outColMask, uint8_t outRowMask)
{
    // output the data
    printf("OUT C 0x%02X R 0x%02X\n", outColMask, outRowMask);
}
#endif
