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

#include <string.h>
#include <math.h>
#include "afterglow.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pio.h"
#include "pindef.h"


//------------------------------------------------------------------------------
// Some definitions


// Afterglow configuration version
#define AFTERGLOW_CFG_VERSION 3

// original matrix update interval [us]
#define ORIG_INT (2000)

// cycles per original interval, config A
#define ORIG_CYCLES_A (ORIG_INT / TTAG_INT_A)

// cycles per original interval, config B
#define ORIG_CYCLES_B (ORIG_INT / TTAG_INT_B)

// number of columns in the lamp matrix
#define NUM_COL 8

// number of rows in the lamp matrix
#define NUM_ROW 8

// WPC systems strobe the 8 columns
#define NUM_STROBE NUM_COL
#define NUM_NONSTROBE NUM_ROW

// glow duration scaling in the configuration
#define GLOWDUR_CFG_SCALE 10

// test mode setup
#define TEST_MODE_NUMMODES 7    // number of test modes
#define TEST_MODE_DUR 8         // test duration per mode [s]
#define TEST_MODE_DUR_CYCLES_A  ((uint32_t)TEST_MODE_DUR * 1000000UL / TTAG_INT_A) // number of cycles per testmode, config A
#define TEST_MODE_DUR_CYCLES_B  ((uint32_t)TEST_MODE_DUR * 1000000UL / TTAG_INT_B) // number of cycles per testmode, config B
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


//------------------------------------------------------------------------------
// Brightness maps

static const uint8_t skMap_256_4_log[256] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3
};

static const uint8_t skMap_256_8_log[256] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,
    5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7
};

static const uint8_t skMap_256_16_log[256] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,
    2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,
    3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,
    5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,
    7,7,7,8,8,8,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,
    10,11,11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,14,14,14,14,14,14,15
};


//-------------#include <stdio.h> -----------------------------------------------------------------
// function prototypes

int loadCfg(int *pErr);
void setDefaultCfg();
void saveCfgToEEPROM();
void applyCfg();
void driveLampMatrixPassThrough();
void driveLampMatrix(uint16_t outCol);
uint32_t testModeInput(void);
bool updateValid(uint16_t inColMask, uint16_t inRowMask);
void updateCol(uint8_t col, uint16_t rowMask);
uint32_t calculateCRC32(const uint8_t *data, uint16_t length);
void dataOutput(uint8_t colData, uint8_t rowData);
void updateMx(uint16_t *pMx, bool on, uint16_t step);
void debugInputs(uint8_t inColMask, uint8_t inRowMask);
void debugOutput(uint8_t outColMask, uint8_t outRowMask);
uint8_t findSubCycle(uint16_t v);
bool checkValidStrobeMask(uint16_t inColMask, uint16_t inRowMask, uint8_t *pStrobeLine);
void updateStrobe(uint8_t strobe, uint16_t colMask, uint16_t rowMask);
void statusUpdate();
void ws2812Update(uint32_t rgb);
#if PROJECT_BUTTER
void updateOutCol(uint16_t outCol);
#endif

// Lamp matrix 'memory'
static uint16_t sMatrixState[NUM_COL][NUM_ROW];

#if PROJECT_BUTTER
// Step per matrix update for all lamps
static uint16_t sMatrixSteps[NUM_COL][NUM_ROW];
static bool sMatrixStepsDir[NUM_COL][NUM_ROW];

// Matrix value to brightness map
static const uint8_t *sBrightnessMap;
#endif

// local time
static uint32_t sTtag = 0;

// interrupt runtime counters [cycles]
static uint16_t sLastIntTime = 0;
static uint16_t sMaxIntTime = 0;
static volatile uint16_t sOverflowCount = 0;

// remember the last column and row samples
static uint16_t sLastColMask = 0;
static uint16_t sLastRowMask = 0;
static uint32_t sConsBadStrobeCounter = 0;

#if DEBUG_SERIAL
static uint16_t sLastOutColMask = 0;
static uint16_t sLastOutRowMask = 0;
static uint32_t sBadStrobeCounter = 0;
static uint32_t sBadStrobeOrderCounter = 0;
static uint32_t sZeroStrobeCounter = 0;
static uint16_t sLastBadStrobeMask = 0;
static uint8_t sLastGoodStrobeLine = 0;
#if CURRENT_MONITOR
static int sMaxCurr = 0;
static int sLastCurr = 0;
#endif
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

// last state of PINB
static uint8_t sLastPINB = 0;

// status enumeration
typedef enum AFTERGLOW_STATUS_e
{
    AG_STATUS_INIT = 0,     // initialising
    AG_STATUS_OK,           // up and running
    AG_STATUS_PASSTHROUGH,  // ready in pass-through mode
    AG_STATUS_TESTMODE,     // ready in test mode
    AG_STATUS_REPLAY,       // ready in replay mode
    AG_STATUS_INVINPUT,     // invalid input
    AG_STATUS_OVERRUN       // interrupt overrun
} AFTERGLOW_STATUS_t;

// afterglow status
static AFTERGLOW_STATUS_t sStatus = AG_STATUS_INIT;
static AFTERGLOW_STATUS_t sLastStatus = AG_STATUS_INIT;


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

    // which column are we updating in this run?
    uint16_t outCol = (sTtag % NUM_COL);

    // kick the dog
    //wdt_reset();  // TODO

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
        driveLampMatrix(outCol);
    }

    // get lamp matrix input from the PIO
    uint16_t inData = pio_col_row_data();

    // testmode input simulation (jumper J1 active)
    if (sTstModeCfg.testMode)
    {
        // test mode
        inData = testModeInput();
    }

    uint16_t inColMask = (inData >> 8);    // LSB is col 0, MSB is col 7
    uint16_t inRowMask = ~(inData & 0x00ff);  // LSB is row 0, MSB is row 7, HIGH means off

    // evaluate the strobe line reading
    // only one bit should be set as only one strobe line can be active at a time
    uint8_t strobeLine;
    bool validInput = checkValidStrobeMask(inColMask, inRowMask, &strobeLine);

    // The input matrix values are updated only once per original strobe cycle. The code
    // waits for a number of consecutive consistent information before adopting the new data.
    validInput &= updateValid(inColMask, inRowMask);

    // Update the input state only with a valid input. If the input is invalid the current
    // input matrix state is left unchanged.
    if (validInput)
    {
        // update the input data with the current strobe line
        updateStrobe(strobeLine, inColMask, inRowMask);

#if DEBUG_SERIAL
        // monitor for bad strobe line input
        if ((strobeLine != (sLastGoodStrobeLine+1)) && (strobeLine!=(sLastGoodStrobeLine-NUM_STROBE+1)))
        {
            sBadStrobeOrderCounter++;
        }
        sLastGoodStrobeLine = strobeLine;
#endif
    }

#if PROJECT_BUTTER
    // update the current output column with the values from the step matrix
    updateOutCol(outCol);
#endif

    // remember the last column and row samples
    sLastColMask = inColMask;
    sLastRowMask = inRowMask;

    // status update
    statusUpdate();

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
        printf("INT dt max %dus last %dus ovfl %d\n", sMaxIntTime, sLastIntTime, sOverflowCount);
        printf("Bad col: %ld col %d ord %ld last good: %d\n", sBadStrobeCounter, sLastBadStrobeMask, sBadStrobeOrderCounter, sLastGoodStrobeLine);
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

#if PROJECT_BUTTER
//------------------------------------------------------------------------------
void updateOutCol(uint16_t outCol)
{
    // get a pointer to the matrix column
    uint16_t *pMx = &sMatrixState[outCol][0];
    uint16_t *pMxSt = &sMatrixSteps[outCol][0];
    bool *pMxStDir = &sMatrixStepsDir[outCol][0];

    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // update the matrix value
        updateMx(pMx, *pMxStDir, *pMxSt);

        // next row
        pMx++;
        pMxSt++;
        pMxStDir++;
    }
}
#endif

//------------------------------------------------------------------------------
void updateMx(uint16_t *pMx, bool on, uint16_t step)
{
    if (on)
    {
        // increase the stored brightness value
        uint16_t v = (*pMx + step);
        *pMx = (v < step) ? 0xffff : v;
    }
    else
    {
        // decrease the stored brightness value
        uint16_t v = (*pMx - step);
        *pMx = (v > step) ? 0 : v;
    }
}

//------------------------------------------------------------------------------
void updateCol(uint8_t col, uint16_t rowMask)
{
    // paranoia check
    if (col >= NUM_COL)
    {
        return;
    }

#if (PROJECT_BUTTER == 0)
    // get a pointer to the matrix column
    uint16_t *pMx = &sMatrixState[col][0];
    const uint16_t *pkStep = &sGlowSteps[col][0];

    // update all row values
    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // update the matrix value
        updateMx(pMx, (rowMask & 0x0001), *pkStep);

        // next row
        pMx++;
        pkStep++;
        rowMask >>= 1;
    }
#else
    // get a pointer to the matrix steps column
    uint16_t *pMxSt = &sMatrixSteps[col][0];
    bool *pMxStDir = &sMatrixStepsDir[col][0];
    const uint16_t *pkStep = &sGlowSteps[col][0];

    // update all row values
    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // set the matrix step value
        *pMxStDir = (rowMask & 0x0001);
        *pMxSt = *pkStep;

        // next row
        pMxSt++;
        pMxStDir++;
        pkStep++;
        rowMask >>= 1;
    }
#endif
}

//------------------------------------------------------------------------------
void updateStrobe(uint8_t strobe, uint16_t colMask, uint16_t rowMask)
{
    updateCol(strobe, rowMask);
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
void driveLampMatrix(uint16_t outCol)
{
    // The update interval is divided into UPD_CYCLES column sub cycles.
    // These cycles are used to do PWM in order to adjust the lamp brightness.
    //
    // Illustration with UPD_CYCLES==4 and four brightness steps B1-B4 and off (B0):
    //
    // * Lamp on
    // afterglow col       012345670123456701234567012345670123456701234567012345
    // col cycle           0       1       2       3       0       1       2
    //
    // Brightness 1        *                               *
    // Brightness 2        *       *                       *       *
    // Brightness 3        *       *       *               *       *       *
    // Brightness 4        *       *       *       *       *       *       *
    uint8_t colCycle = (!sTstModeCfg.freq2kHz) ?
        (uint8_t)((sTtag / NUM_COL) % PWM_STEPS_A) :
        (uint8_t)((sTtag / NUM_COL) % PWM_STEPS_B);

    // prepare the data
    // LSB is row/col 0, MSB is row/col 7
    uint16_t colData = (1 << outCol);
    uint16_t rowData = 0;
    uint16_t *pMx = &sMatrixState[outCol][0];
    uint8_t *pMaxSubCycle = &sMaxSubcycle[outCol][0];
    for (uint8_t r=0; r<NUM_ROW; r++)
    {
        // make room for the next bit
        rowData >>= 1;
        
        // nothing to do if the matrix value is zero (off)
        //if (*pMx)  // handling the case only down below as we want constant run time here
        {
            // Find the subcycle in the brightness map
            uint8_t subCycle = findSubCycle(*pMx);

            // limit to the configured maximum brightness
            if (subCycle > *pMaxSubCycle)
            {
                subCycle = *pMaxSubCycle;
            }

            // Lamps are turned on when the value in the matrix is not zero
            // and when the value is high enough for the current sub cycle.
            if (subCycle >= colCycle)
            {
                if (*pMx) // handling the OFF state here in order to have a more constant run time
                {
                    rowData |= ((uint16_t)1 << (NUM_ROW-1));
                }
            }
        }
        pMx++;
        pMaxSubCycle++;
    }

    // turn off everything briefly to avoid ghosting
    dataOutput(0x0000, 0x0000);
    busy_wait_us(ANTIGHOST_DURATION);

    // output the data
    dataOutput(colData, rowData);
#if DEBUG_SERIAL
    sLastOutColMask = colData;
    sLastOutRowMask = rowData;
#endif
}

//------------------------------------------------------------------------------
uint8_t findSubCycle(uint16_t v)
{
#if (PROJECT_BUTTER == 0)
    uint8_t subCycle = (PINB & B00000100) ?
        (uint8_t)(v / (65536 / PWM_STEPS_A)) :
        (uint8_t)(v / (65536 / PWM_STEPS_B));
#else
    // This is done in a non optimal way in order to
    // maintain a constant runtime.
    uint8_t subCycle = *(sBrightnessMap + (v >> 8));
#endif
    return subCycle;
}

//------------------------------------------------------------------------------
void dataOutput(uint8_t colData, uint8_t rowData)
{
    // data output is handled by PIO
    pio_write_col_row_data(colData, rowData);
}

//------------------------------------------------------------------------------
uint32_t testModeInput(void)
{
    // simulate the original column cycle
    static uint8_t sCycleCounter = 0;
    static uint8_t sStrobeLine = 0;
    bool modeA = (!sTstModeCfg.freq2kHz) ? true : false;
    if ((modeA && (sCycleCounter == ORIG_CYCLES_A)) ||
        (!modeA && (sCycleCounter == ORIG_CYCLES_B)))
    {
        sCycleCounter = 0;
        sStrobeLine++;
    }
    if (sStrobeLine == NUM_STROBE)
    {
        sStrobeLine = 0;
    }
    sCycleCounter++;
    uint16_t strobeMask = ((uint16_t)1 << (uint16_t)sStrobeLine);

    // populate the non strobed mask
    uint16_t nonStrobeMask = 0;

    // Start simulation if test switch 2 (replay mode) is inactive
    {       
        // loop through all available modes
        static uint32_t sModeCounter = 0;
        static uint32_t sMode = 0;
        static uint32_t sModeCycleCounter = 0;
        static uint32_t sModeCycle = 0;
        if ((modeA && (sModeCounter == TEST_MODE_DUR_CYCLES_A)) ||
            (!modeA && (sModeCounter == TEST_MODE_DUR_CYCLES_B)))
        {
            sModeCounter = 0;
            sModeCycleCounter = 0;
            sModeCycle = 0;
            sMode++;
        }
        if (sMode == TEST_MODE_NUMMODES)
        {
            sMode = 0;
        }
        sModeCounter++;
        if ((modeA && (sModeCycleCounter == TESTMODE_CYCLES_A)) ||
            (!modeA && (sModeCycleCounter == TESTMODE_CYCLES_B)))
        {
            sModeCycleCounter = 0;
            sModeCycle++;
        }
        sModeCycleCounter++;

        switch (sMode)
        {
            case 0:
            // cycle all strobe lines
            {
                uint8_t s = (sModeCycle % NUM_STROBE);
                if (s == sStrobeLine)
                {
                    nonStrobeMask = 0xffff;
                }
            }
            break;
            case 1:
            // cycle all non strobe lines
            {
                uint8_t ns = (sModeCycle % NUM_NONSTROBE);
                nonStrobeMask |= ((uint16_t)1 << ns);
            }
            break;
            case 2:
            // cycle all strobe lines (inverted)
            {
                uint8_t s = (sModeCycle % NUM_STROBE);
                if (s != sStrobeLine)
                {
                    nonStrobeMask = 0xffff;
                }
            }
            break;
            case 3:
            // cycle all non strobe lines (inverted)
            {
                uint8_t ns = (sModeCycle % NUM_NONSTROBE);
                nonStrobeMask = ~(1 << ns);
            }
            break;
            case 4:
            // blink all lamps
            {
                if (sModeCycle % 2)
                {
                    nonStrobeMask = 0xffff;
                }
            }
            break;
            case 5:
            // switch between even and odd lamps
            // turn on every other strobe line
            {
                if (sStrobeLine % 2 == (sModeCycle % 2))
                {
                    nonStrobeMask = 0xaaaa;
                    if (sModeCycle % 3)
                    {
                        nonStrobeMask <<= 1;
                    }
                }
            }
            break;
            case 6:
            // cycle through all lamps individually with 4x speed
            {
                uint8_t l = (uint8_t)((sModeCycle * 4) % (NUM_COL * NUM_ROW));
                uint8_t c = (l / NUM_ROW);
                uint8_t r = (l % NUM_COL);
                if (c == sStrobeLine)
                {
                    nonStrobeMask = (1 << r);
                }
            }
            break;
            default:
            break;
        }
    }

    // assign the column and row mask
    uint16_t colMask = strobeMask;
    uint16_t rowMask = nonStrobeMask;

    // invert the row mask as in the original input HIGH means off
    rowMask = ~rowMask;
    return (((uint32_t)colMask << 16) | (uint32_t)rowMask);
}

//------------------------------------------------------------------------------
bool checkValidStrobeMask(uint16_t inColMask, uint16_t inRowMask, uint8_t *pStrobeLine)
{
    bool validInput = true;
    *pStrobeLine = NUM_STROBE;

    uint16_t strobeMask = (inColMask & 0x00ff);
    switch (strobeMask)
    {
        case 0x0001: *pStrobeLine = 0; break;
        case 0x0002: *pStrobeLine = 1; break;
        case 0x0004: *pStrobeLine = 2; break;
        case 0x0008: *pStrobeLine = 3; break;
        case 0x0010: *pStrobeLine = 4; break;
        case 0x0020: *pStrobeLine = 5; break;
        case 0x0040: *pStrobeLine = 6; break;
        case 0x0080: *pStrobeLine = 7; break;
#if (NUM_STROBE > 8)
        case 0x0100: *pStrobeLine = 8; break;
        case 0x0200: *pStrobeLine = 9; break;
#endif
        default:
        {
            // This may happen if the sample is taken in between column transition.
            // Depending on the pinball ROM version the duration of this transition varies.
            // On a Whitewater with Home ROM LH6 (contains anti ghosting updates) this
            // gap was measured to be around 30us long.
            // Machines with anti-ghosting firmware will show a gap with no column enabled
            // for a while during the transition while older firmwares might have two
            // columns enabled at the same time due to slow transistor deactivation. Both
            // cases are caught here.
            // See also https://emmytech.com/arcade/led_ghost_busting/index.html for details.
            sConsBadStrobeCounter++;
            sBadStrobeCounter++;
            if (strobeMask == 0)
            {
                sZeroStrobeCounter++;
            }
#if DEBUG_SERIAL
            sLastBadStrobeMask = strobeMask;
#endif
            validInput = false;
        }
        break;
    }

    // restart the consecutive bad strobe counter
    if (validInput)
    {
        sConsBadStrobeCounter = 0;
    }
    return validInput;
}

//------------------------------------------------------------------------------
bool updateValid(uint16_t inColMask, uint16_t inRowMask)
{
    static uint8_t sConsistentSamples = 0;
    static uint16_t sLastUpdStrobeMask = 0x0000;
    bool valid = false;

    uint16_t strobeMask = inColMask;

    // check if the current strobe line has not been handled already
    if (strobeMask != sLastUpdStrobeMask)
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
            sLastUpdStrobeMask = strobeMask;
            valid = true;
        }
    }
    return valid;
}

//------------------------------------------------------------------------------
void applyCfg()
{
    uint8_t pwmSteps = (!sTstModeCfg.freq2kHz) ? PWM_STEPS_A : PWM_STEPS_B;

#if PROJECT_BUTTER
    // select the brightness map according to the current configuration
    sBrightnessMap = (pwmSteps == 4) ? skMap_256_4_log :
                     (pwmSteps == 8) ? skMap_256_8_log : skMap_256_16_log;
#endif
    
    // calculate the glow steps and maximum subcycles
    uint16_t *pGS = &sGlowSteps[0][0];
    uint8_t *pGlowDur = &sCfg.lampGlowDur[0][0];
    uint8_t *pBrightness = &sCfg.lampBrightness[0][0];
    uint8_t *pMaxSubCycle = &sMaxSubcycle[0][0];
    for (uint8_t c=0; c<NUM_COL; c++)
    {
        for (uint8_t r=0; r<NUM_ROW; r++)
        {
            uint32_t glowDur = (*pGlowDur * GLOWDUR_CFG_SCALE);

            // translate maximum brightness into maximum lamp driving subcycle
            *pMaxSubCycle = (pwmSteps <= 8) ? (*pBrightness >> (8/pwmSteps-1)) : (*pBrightness << (pwmSteps/8-1));

#if (PROJECT_BUTTER == 0)
            // brightness step per lamp matrix update (assumes one update per original matrix step)
            *pGS++ = (glowDur > 0) ?
                ((uint16_t)(65535 / ((glowDur * 1000) / ORIG_INT)) * NUM_STROBE) : 0xffff;
#else
            // brightness step per lamp matrix update (assumes one update per matrix step)
            uint16_t maxVal = (!sTstModeCfg.freq2kHz) ?
                (uint16_t)(log10((float)(*pMaxSubCycle)*10.0f/(float)(PWM_STEPS_A-1)) * 65535) :
                (uint16_t)(log10((float)(*pMaxSubCycle)*10.0f/(float)(PWM_STEPS_B-1)) * 65535);
            *pGS++ = (glowDur > 0) ?
                (!sTstModeCfg.freq2kHz) ?
                 ((uint16_t)(maxVal / ((glowDur * 1000) / TTAG_INT_A)) * NUM_COL) : 
                 ((uint16_t)(maxVal / ((glowDur * 1000) / TTAG_INT_B)) * NUM_COL) : 0xffff;
#endif

            // next
            pMaxSubCycle++;
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

//------------------------------------------------------------------------------
void statusUpdate()
{
    // update the current status value
    if (sTstModeCfg.testMode)
    {
        // testmode input simulation (jumper J1 active)
        sStatus = AG_STATUS_TESTMODE;
    }
    else
    {
        // normal operation
        sStatus = (sOverflowCount < 100) ? 
                   ((sConsBadStrobeCounter < 100) ?
                    ((sTstModeCfg.passThrough) ? AG_STATUS_PASSTHROUGH : AG_STATUS_OK) : AG_STATUS_INVINPUT) : AG_STATUS_OVERRUN;
    }

    // use the RGB LED to show the status
    if (sStatus != sLastStatus)
    {
        switch (sStatus)
        {
            case AG_STATUS_INIT: ws2812Update(0x00222222); break;
            case AG_STATUS_OK: ws2812Update(0x00220000); break;
            case AG_STATUS_TESTMODE: ws2812Update(0x00220011); break;
            case AG_STATUS_INVINPUT: ws2812Update(0x00003300); break;
            case AG_STATUS_OVERRUN: ws2812Update(0x00003333); break;
            case AG_STATUS_PASSTHROUGH: ws2812Update(0x00333333); break;
            case AG_STATUS_REPLAY: ws2812Update(0x00110033); break;
            default: ws2812Update(0x00444444); break;
        }
        sLastStatus = sStatus;
    }
}

//------------------------------------------------------------------------------
void ws2812Update(uint32_t rgb)
{
#if RGB_LED
    pio_set_ws2812(rgb);
#endif
}

