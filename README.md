# elegoo-car-ppm
Integration of a PPM radio control receiver for the Elegoo Smart Car

## Building this code

This requires the following libraries, all available via the Libraries tab in the Arduino IDE: 
* PPM, https://www.arduino.cc/reference/en/libraries/ppm/
* FastLED, https://github.com/FastLED/FastLED
* Servo, https://www.arduino.cc/reference/en/libraries/servo/
* ArduinoQueue (only if building with automated mode enabled), https://github.com/EinarArnason/ArduinoQueue

Note that there are two versions of the code here; "stripped" was the original version hacked together in the fall of 2022, and can be considered deprecated, but is known to work with the V4 and V3 bots, and is the only code available here that works with the V3, if needed for older bots.

The "base" version is where you will find the primary code, and if you're making custom builds of your own, gives you a structure to work with.

There are two primary build options in the code, see the top of ApplicationFunctionSet_xxx0.cpp.
* AUTOENABLED - uncommenting this will build functionality for autonomous control, more detail on that at a later date (note that there's a bad obstacle avoidance implementation in there, but it should work in a neat box; we may remove that default implementation) 
* SERVOENABLED - uncommenting this will give you control of the servo on your left stick X axis, but if you don't need it, don't waste the battery/cycles, as the servo operation will add some latency to your motor control


## Operating the Robot

### If Automatic mode is enabled
Pressing the momentary switch *on the robot control board* will cycle through three modes:
* Violet (almost white, solid) gives you **Operator** control for testing/play, this is default
* Red (pulsing) is **Standby** mode for the Red team waiting for activation of automatic control
* Blue (pulsing) is **Standby** mode for the Blue team waiting for activation of automatic control

When in **Standby**, the momentary button under your right finger will start automatic mode, or, if held during automatic operation, stop the robot until released. 

After activating automatic operation, it will continue until the automatic game time (defined in code at 30 seconds, currently) has elapsed, after which the bot will switch to operator control, indicated by a **solid** red or blue light on the board. 

### If Automatic mode is NOT enabled
Pressing the momentary switch *on the robot control board* will cycle through three modes:
* Violet (almost white, solid) gives you **Operator** control for testing/play, this is default
* Red (solid) is **Operator** mode for the Red team 
* Blue (solid) is **Operator** mode for the Blue team 

### Operator mode controller button functions
* Rocker under your right finger controls the drive mode: center or down gives you Arcade type control; up gives you Tank type control
* The dial under your left finger controls the maximum speed available in operator mode
* In Arcade mode, the left stick controls forward/backward, and the right stick controls left/right
* In Tank mode, the left stick controls fore/back of the left motors, the right stick controls fore/back of the right motors
* IF servo usage is enabled, the left stick X axis controls the angle of SERVO1 on the board
