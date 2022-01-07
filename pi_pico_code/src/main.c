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
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pio.h"
#include "pindef.h"
#include "afterglow.h"


//------------------------------------------------------------------------------
// local variables

// The 250us timer is the realtime task of this software
static repeating_timer_t sHeartbeatTimer;


//------------------------------------------------------------------------------
bool heartbeat(struct repeating_timer *t)
{
    // afterglow update
    ag_update();
    return true;
}

//------------------------------------------------------------------------------
void panic_mode()
{
    // endless panic
    while (true)
    {
        // blinking alert
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(200);
    }
}

//------------------------------------------------------------------------------
int main(void)
{  
    stdio_init_all();

    printf("\n\nAfterglow Pico v0.1\n");

    // initialize the Pi Pico's LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // test mode pin
    gpio_init(AG_PICO_PIN_TESTMODE_IN);
    gpio_set_dir(AG_PICO_PIN_TESTMODE_IN, GPIO_IN);
    gpio_set_pulls(AG_PICO_PIN_TESTMODE_IN, true, false); // pull up

    // disable output for now
    gpio_init(AG_PICO_PIN_DATAOE);
    gpio_put(AG_PICO_PIN_DATAOE, false);
    gpio_set_dir(AG_PICO_PIN_DATAOE, GPIO_OUT);
    gpio_init(AG_PICO_PIN_OE);
    gpio_init(AG_PICO_PIN_BLANKING_OUT);
    for (uint32_t pin=AG_PICO_PIN_DATA_OUT_D0; pin<(AG_PICO_PIN_DATA_OUT_D0+8); pin++)
    {
        gpio_init(pin);
        gpio_put(pin, true);
        gpio_set_dir(pin, GPIO_OUT);
    }
    gpio_init(AG_PICO_PIN_LCOL_OUT);
    gpio_put(AG_PICO_PIN_LCOL_OUT, true);
    gpio_set_dir(AG_PICO_PIN_LCOL_OUT, GPIO_OUT);
    gpio_init(AG_PICO_PIN_LROW_OUT);
    gpio_put(AG_PICO_PIN_LROW_OUT, true);
    gpio_set_dir(AG_PICO_PIN_LROW_OUT, GPIO_OUT);

    // afterglow init
    ag_init();

    // PIO setup
    if (!pio_init())
    {
        printf("Failed to initialize the PIOs!\n");
        panic_mode();
    }

    // Heartbeat setup
    if (!add_repeating_timer_us(-250, heartbeat, NULL, &sHeartbeatTimer))
    {
        printf("Failed to start the heartbeat!\n");
        panic_mode();
    }

    // Enable output
    gpio_set_dir(AG_PICO_PIN_OE, GPIO_OUT);
    gpio_put(AG_PICO_PIN_OE, false);

    // Eternal loop
    while (true)
    {
        // afterglow serial communication
        ag_sercomm();

        // every device needs a blinking LED
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);

        // some PIO debugging
        //pio_debug();
    }
}
