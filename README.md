# Afterglow Pico
The afterglow pico is the newest and tiniest addition to the afterglow family.

It operates differently from the other boards as it is hooked directly into the WPC bus. The underlying concept remains the same though. The original lamp matrix is resampled at 4kHz and PWM is used to add a nice afterglow to the LEDs in your pinball machine.

![afterglow_pico](https://github.com/bitfieldlabs/afterglow_pico/blob/master/pcb_v13/pictures/pcb_v13.jpg "Afterglow Pico")

## Disclaimer

| :boom: WARNING  :boom:            |
|:---------------------------|
| **Messing with your pinball machine's data bus is risky!** |
| *The AG pico is an experiment, for stable afterglow please<br>use the standard afterglow boards.* |

Afterglow is distributed in the hope that it will be useful but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

## Concept
The concept is quite simple:
* The lamp matrix status is sampled directly from the WPC bus by the Raspberry Pi Pico
* All non lamp related address lines (solenoids etc.) are logically combined (AND) to monitor bus activity
* All non lamp related bus events are directly routed through a multiplexer to the output. Only the lamp events are filtered out. This ensures that original bus events always take precedence over the Pi signals and that no original events are lost.
* The Pi Pico interleaves the upsampled 4kHz PWM lamp matrix events into the bus through the multiplexer

![Afterglow Pico Schematic](https://github.com/bitfieldlabs/afterglow_pico/blob/master/pcb_v11/afterglow_pico.pdf "Afterglow Pico Schematic")

## Compiling the Raspberry Pi Pico code

The [pi_pico_code folder](https://github.com/bitfieldlabs/afterglow_pico/tree/master/pi_pico_code) contains a [platformio](https://platformio.org/) project which can be compiled using [Visual Studio Code](https://code.visualstudio.com/). It requires the [wizio-pico](https://github.com/Wiz-IO/wizio-pico) platform extension.
