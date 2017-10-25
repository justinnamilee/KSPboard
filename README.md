# KSPboard
Arduino code and Python script for interacting with Kerbal Space Program using [kRPC](https://krpc.github.io/krpc/index.html).

* ## Arduino Side

  * Configured for controlling the switches, joystick, etc. on the physical side.  Serves serial to Python client which parses and handles the rest on the PC side.
  * Enable pin that controls when the Python client exits. __Pin 2__.
  * Joystick is the four switch variety (P/Y or P/R), they are pulled high and should be connected to ground unless activation is desired. Additionally yaw or roll (whichever is not on the joystick) can then be put on two pedals which brings the total to six switches for full helm control. __Pins 7-12__.
  * Will eventually use an I/O expansion board, but for now it has support for four chosen actions using the remaining digital I/O pins.  These are configured with the opPin array and constants defined at the top of the file. These are also pulled high and should be connected to ground unless activation is desired.  Additionally each operation has an independent digital debounce counter. __Pins 3-6__.
  
-- --

* ## Python Side

  * Accepts serial data, uses that to control Kerbal Space Program with [kRPC](https://krpc.github.io/krpc/index.html).
  * Make sure kRPC mod for KSP is installed, then you will need the kRPC python library as well.
  * To use the script:
    * Flash Arduino.
    * Launch KSP.
    * Load a craft at any launch site.
    * Start kRPC server in KSP.
    * Start Python script making sure Enable Pin is pulled HIGH on Arduino.

-- --
-- --

# Parts Ordered: 2017-09-19
 * __40x__ SPDT Microswitch:  For the 10 custom actions, core action groups, etc.
 * ~~__2x__ Linear Slide Potentiometer:  One for trottle, one for pitch trim.~~ *Got In Mail 2017-10-05*
 * ~~__1x__ 4/8-way Arcade Joystick: For pitch and roll control.~~ *Got in Mail 2017-10-23*
 * ~~__3x__ Rotary Encorder: Thrust limiters, trim, etc.~~ *Got in Mail 2017-10-16*
 * __2x__ Sewing Machine Foot Pedal: For yaw control.
 * ~~__2x__ 16-port I/O Expander: To help connect it all up.~~ *Got in Mail 2017-10-16*
