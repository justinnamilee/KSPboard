//////////////////////////////////////////
/// Justin Lee <justin at kitten dot pink>
//
// Arduino driver for the KSPboard game
// controller.  See LICENSE and README
// for more information.


/////////////
/// libraries
//


#include <Wire.h>



////////////////////////////////////
/// pin configurations and constants
//


#define ENABLE true
#define DISABLE false
#define NEUTRAL 2

// serial
#define SERIAL_SPEED 115200

// state pins
#define PIN_LED 13 // status
#define PIN_ENABLE A1 // this should be high to run

// helm pins
#define PIN_PITCH_U 12
#define PIN_PITCH_D 11
#define PIN_PITCH_S 5 // axis stick flag
#define PIN_ROLL_R 9
#define PIN_ROLL_L 10
#define PIN_ROLL_S 4 // axis stick flag
#define PIN_YAW_R 7
#define PIN_YAW_L 8
#define PIN_YAW_S 3 // axis stick flag

#define PIN_THROTTLE A0 // analog read

// rotary pins
#define PIN_ROT_CTRL_CLK 2
#define PIN_ROT_CTRL_DATA A2
#define PIN_ROT_CTRL_SW A3 // no pin yet

// i2c addresses / stuff
#define I2C_SUCCESS 0
#define IO_ADDR_BASE 0x20
#define IO_DATA_LENGTH 2

#define IO_DEVICE_OPS 0

// delays
#define DELAY_LOOP 10
#define DELAY_START 30
#define DELAY_OP 100
#define DELAY_CTRL 100

// misc
#define OPS 16 // Action 1-10, stage, gear, light, rcs, brake, abort

// helm
#define RAMP_MAX 50
#define RAMP_MIN -RAMP_MAX
#define DIR_MAX 2000
#define DIR_MIN -DIR_MAX

// helper stuff for helm / shorthand
#define PITCH_STICK digitalRead(PIN_PITCH_S)
#define ROLL_STICK digitalRead(PIN_ROLL_S)
#define YAW_STICK digitalRead(PIN_YAW_S)

#define PITCH_PINS PIN_PITCH_U, PIN_PITCH_D
#define ROLL_PINS PIN_ROLL_R, PIN_ROLL_L
#define YAW_PINS PIN_YAW_R, PIN_YAW_L



///////////
/// globals
//


// helm control variables
int16_t pitch, yaw, roll, throttle;
int8_t pitchAdjust = 0, yawAdjust = 0, rollAdjust = 0; // for ramping the direction input

// sas control variable
volatile uint8_t _vol_rotaryControl = 0; // 0-255 for conversion to sas states
uint8_t controlState = 0, controlLocked = 0; // 0-7 for all sas states
uint8_t controlDebounce = 0;

// op control variables
// state == 0 to DELAY_OP-1 -> off / debounce, state == DELAY_OP -> on
// Basically if this is not equal to DELAY_OP then it is in debounce state
// or off (zero).  When it is equal to DELAY_OP then it fires the op code out
// over serial. This allows multi switch debounce in a convenient package.
uint8_t opState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// this controls whether the python script closes or not, 1 == run
boolean state = 1;



/////////////////////
//// helper functions
//


// interrupt for the SAS rotary dial & helper
void rotaryInterruptHandler()
{
  if (digitalRead(PIN_ROT_CTRL_CLK))
    _vol_rotaryControl += digitalRead(PIN_ROT_CTRL_DATA) ? 1 : -1; // cw
  else
    _vol_rotaryControl -= digitalRead(PIN_ROT_CTRL_DATA) ? 1 : -1; // ccw
}

uint8_t rotaryControl2State(uint8_t c) // this should be moved to python client
{
  return ((uint8_t)((6.0 * (float)c) / (float)UINT8_MAX));
}

// i/o expander helpers
uint8_t requestDevice(uint8_t address) // ping the device / send start command
{
  Wire.beginTransmission(IO_ADDR_BASE | address);
  return (Wire.endTransmission()); // returns success fail
}

boolean requestData(uint8_t address) // returns success fail
{
  return (Wire.requestFrom(IO_ADDR_BASE | address, IO_DATA_LENGTH) == IO_DATA_LENGTH);
}

uint16_t readData() // get the data bytes and stuff them into a 16-bit unsigned value
{
  return (~(Wire.read() | (Wire.read() << 8)));
}

// direction helpers
int16_t getAdjustment(uint8_t pinH, uint8_t pinL, int8_t adj) // get control input
{
  switch (digitalRead(pinH) ? HIGH : (digitalRead(pinL) ? LOW : NEUTRAL))
  {
    case HIGH: // pitch up / roll right / yaw right
      if (adj++ > RAMP_MAX)
        adj = RAMP_MAX;
      break;

    case LOW: // pitch down / roll left / yaw left
      if (adj-- < RAMP_MIN)
        adj = RAMP_MIN;
      break;

    case NEUTRAL: // stuck in the middle with you
    default:
      adj = 0;
  }

  return (adj);
}

int16_t getDirection(int16_t dir, int8_t adj, boolean stick)
{
  // reset current if no adj&stick flag or if adj and current are opposite signs
  if (!(adj || stick) || (dir > 0 && adj < 0) || (dir < 0 && adj > 0))
  {
    dir = 0; // reset current direction to neutral
  }
  else
  {
    dir += adj; // adjust current direction

    if (dir > DIR_MAX) // keep the current direction in bounds
      dir = DIR_MAX;
    else if (dir < DIR_MIN)
      dir = DIR_MIN;
  }

  return (dir);
}

uint16_t getThrottle(uint8_t pin) // get throttle input
{
  return (analogRead(pin)); // this used to do more
}



////////////////////
//// setup fucntions
//


void setupSerial()
{
  Wire.begin(); // start up I/O expanders

  Serial.begin(SERIAL_SPEED);
  Serial.println();
  Serial.flush();

  Serial.print(1); // enable code, super secure
  Serial.print(1);
  Serial.print(1);
}

void setupControl()
{
  // the rotary encoder uses two wire differential signalling
  attachInterrupt(
    digitalPinToInterrupt(PIN_ROT_CTRL_CLK),
    rotaryInterruptHandler,
    FALLING);
}

void setupState()
{
  pinMode(PIN_LED, OUTPUT); // debug led

  pinMode(PIN_ENABLE, INPUT_PULLUP);
}

void setupHelm()
{
  pinMode(PIN_PITCH_U, INPUT_PULLUP);
  pinMode(PIN_PITCH_D, INPUT_PULLUP);
  pinMode(PIN_PITCH_S, INPUT_PULLUP);
  pinMode(PIN_ROLL_L, INPUT_PULLUP);
  pinMode(PIN_ROLL_R, INPUT_PULLUP);
  pinMode(PIN_ROLL_S, INPUT_PULLUP);
  pinMode(PIN_YAW_L, INPUT_PULLUP);
  pinMode(PIN_YAW_R, INPUT_PULLUP);
  pinMode(PIN_YAW_S, INPUT_PULLUP);
}

void setupOps()
{
  // put check here for i/o expander??
}



/////////////////////
//// update fucntions
//


void updateControl()
{
  uint8_t roter = 0; // setup space

  noInterrupts(); // shutdown interrupts for this volatile operation
  roter = _vol_rotaryControl; // copy the volatile control variable
  interrupts(); // reenable interrupts immediately

  uint8_t state = rotaryControl2State(roter);

  if (state != controlState)
  {
    controlState = state; // update the hilighted state

    // do hiligting stuff //
  }

  if (controlDebounce > 0)
  {
    controlDebounce--; // do dat sweet digi debounce
  }
  else
  {
    if (digitalRead(PIN_ROT_CTRL_SW)) // if you press dis switch lock the sas control state
    {
      controlLocked = controlState; // update locked variable

      Serial.println("this is where the good stuff goes, fuck yeah");

      controlDebounce = DELAY_CTRL; // reset delay
    }
  }

  Serial.println(DISABLE); // skip or end transmission
}

void updateState()
{
  // tells the python client when to close
  state = digitalRead(PIN_ENABLE);
  digitalWrite(PIN_LED, state);
  Serial.println(state);
}

void updateHelm()
{
  // update control surface inputs
  pitchAdjust = getAdjustment(PITCH_PINS, pitchAdjust);
  pitch = getDirection(pitch, pitchAdjust, PITCH_STICK);
  Serial.println(pitch);

  rollAdjust = getAdjustment(ROLL_PINS, rollAdjust);
  roll = getDirection(roll, rollAdjust, ROLL_STICK);
  Serial.println(roll);

  yawAdjust = getAdjustment(YAW_PINS, yawAdjust);
  yaw = getDirection(yaw, yawAdjust, YAW_STICK);
  Serial.println(yaw);

  // update our current throttle input level
  throttle = getThrottle(PIN_THROTTLE);
  Serial.println(throttle);
}

void updateOps()
{
  uint8_t ops = 0;
  uint8_t index = 0;

  if (requestDevice(IO_DEVICE_OPS) == I2C_SUCCESS) // send start, skip if failure
  {
    if (requestData(IO_DEVICE_OPS)) // ask for data, skip if failure
    {
      uint16_t data = readData(); // finally get the 2 byte data packet

      for (index = 0; index < OPS; index++) // scan through ops 1-16 (everything but sas)
      {
        if (opState[index] > 0) // check for debounce on current pin
        {
          opState[index]--; // decrement debounce counter for current state
        }
        else // if not we can check to see if it's triggered
        {
          // scan through 0b0000000000000001 to 0b1000000000000000 masks
          // if that switch is 1 then set it's state to DELAY_OP to flag
          // it to be sent out over serial, otherwise set it to zero
          opState[index] =  ((1 << index) & data) ? DELAY_OP : 0;
          ops++;
        }
      }
    }
  }

  if (ops > 0) // if we have at least one operation to send
  {
    Serial.println(ENABLE); // send ops enable because we have at least one op

    for (index = 0; index < OPS; index++)
    {
      // for each state that's flagged...
      if (opState[index] == DELAY_OP)
        Serial.println(index); // print it
    }
  }

  Serial.println(DISABLE); // send skip / end tramission flag
}



///////////////////////////////
//// built-in arduino functions
//


void setup()
{
  setupSerial();

  setupControl();
  setupState();
  setupHelm();
  setupOps();

  delay(DELAY_START);
}

void loop()
{
  updateState();

  if (state)
  {
    updateControl();
    updateHelm();
    updateOps();
  }

  delay(DELAY_LOOP);
  Serial.flush();
}

