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
 *  | LED        | LED             | GPIO 12    | OUTPUT    |
 *  | DATA_OUT   | Data Out D0-D7  | GPIO 15-22 | OUTPUT    |
 *  | COL_OUT    | Column Sig Out  | GPIO 26    | OUTPUT    |
 *  | ROW_OUT    | Row Sig Out     | GPIO 27    | OUTPUT    |
 *  | PIC_OUT_EN | Pico Out Enable | GPIO 28    | OUTPUT    |
 *  +------------+-----------------+------------+-----------+
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pio.h"
#include "pindef.h"
#include "afterglow.h"


//------------------------------------------------------------------------------
// local variables

static repeating_timer_t sHeartbeatTimer;


//------------------------------------------------------------------------------
bool heartbeat(struct repeating_timer *t)
{
    // afterglow update
    ag_update();
    return true;
}

//------------------------------------------------------------------------------
int main(void)
{  
    stdio_init_all();

    printf("\n\nAfterglow Pico v0.1\n");
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // disable output for now
    gpio_init(AG_PICO_PIN_DATAOE);
    gpio_put(AG_PICO_PIN_DATAOE, false);
    gpio_set_dir(AG_PICO_PIN_DATAOE, GPIO_OUT);
    gpio_init(AG_PICO_PIN_OE);
    gpio_init(AG_PICO_PIN_BLANKING_OUT);

    // afterglow init
    ag_init();

    // PIO setup
    pio_init();

    // Heartbeat setup
    if (!add_repeating_timer_us(-250, heartbeat, NULL, &sHeartbeatTimer))
    {
        printf("Hach!\n");
        while (true)
        {
            // can't do much really
            // blinking alert
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(200);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(200);
        }
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