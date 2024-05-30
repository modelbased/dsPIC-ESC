# dsPIC33 based Field Oriented Controller for Brushless Motor ESCs

### Highlights

This is a Field Oriented Control (FOC) motor controller implementation for dsPIC33 based ESCs.

It was originally developed for use in torque sensing robot joints, but may be used in any motor control application that requires current control.

Commercial licence for both hardware & software available.

Project features:
* Accurate current sensing & control at 100Amps+
* Memory safety (no malloc)
* Short and easy to understand, verify, codebase
* Based on low cost components

### Prerequisites

* PCB design available under commercial licence
* CCS PIC compiler (https://www.ccsinfo.com)


### Software features & details

* Hard realtime leveraging vector interrupt with prioritisation
* 20kHz center aligned PWM with deadtime
* Controls two 3-phase brushless motors
* Current sensing on all 2x 3 phases
* Biquad filtering of current sense data
* Motor and PCB temperature sensing
* Multiple motor angle sensing:
    * Digital hall sensors
    * Analogue hall sensors (for compact/low cost applications)
    * Rotary encoders over SPI
* Low latency control comms uses DMA
* UART debuggin output
* Fast dsPIC DSP fixed point maths engine leveraging inline assembly
* Motor calibration data stored to dsPIC flash
* Control multiple ESCs over SPI (up to 12 tested at 1kHz each)
* Reliable control comms using checksumms