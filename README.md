# Afterglow Pico
The afterglow pico is the newest and tiniest addition to the afterglow family.

It operates differently from the other boards as it is hooked directly into the WPC bus. The underlying concept remains the same though. The original lamp matrix is resampled at 4kHz and PWM is used to add a nice afterglow to the LEDs in your pinball machine.

![afterglow_pico](https://github.com/bitfieldlabs/afterglow_pico/blob/master/docs/pcb_v11_render.jpg "Afterglow Pico")

The concept is quite simple:
* The lamp matrix status is sampled directly from the WPC bus by the Raspberry Pi Pico
* All non lamp related address lines (solenoids etc.) are logically combined (AND) to monitor bus activity
* All non lamp related bus events are directly routed through a multiplexer to the output. Only the lamp events are filtered out.
* The Pi Pico interleaves the upsampled 4kHz PWM lamp matrix events into the bus through the multiplexer

![Afterglow Pico Schematic](https://github.com/bitfieldlabs/afterglow_pico/blob/master/pcb_v11/afterglow_pico.pdf "Afterglow Pico Schematic")
