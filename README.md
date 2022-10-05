# elegoo-car-ppm
Integration of a PPM radio control receiver for the Elegoo Smart Car

This does require the PPM library, https://www.arduino.cc/reference/en/libraries/ppm/, available via the Libraries tab in the Arduino IDE. 

Two different approaches are included here, slipstreamed and stripped; slipstreamed adds the radio control mode in place of line tracing in the Elegoo base code, so that all other functions remain; stripped takes out all unnecessary code and leaves only radio control and battery monitoring. 

Stripped code _does run better_ than slipstreamed, since the Elegoo base code is doing a _lot_ on the interrupt loop, so you have more glitches in operation. 

*Controller buttons as programmed*
* Rocker under your right finger controls the drive mode: center or down gives you RC car type control; up gives you skid-steer type control
* In RC car mode, the left stick controls forward/backward, and the right stick controls left/right. The dial control on the left finger gives control over maximum speed. 
* In Skid-steer mode, the left stick controls fore/back of the left motors, the right stick controls fore/back of the right motors

 