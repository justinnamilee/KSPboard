/// Justin Lee <justin at kitten dot pink>
//
// Arduino driver for the KSPboard game
// controller.  See LICENSE and README
// for more information.

#include <Wire.h>


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
#define PIN_ROT_CLK 2
#define PIN_ROT_DATA A2
#define PIN_ROT_SW // no pin yet

// i2c addresses / stuff
#define I2C_SUCCESS 0
#define IO_ADDR_BASE 0x20
#define IO_DATA_LENGTH 2

#define IO_DEV_OPS 0

// delays
#define DELAY_LOOP 1000
#define DELAY_START 30
#define DELAY_OP 100

// misc
#define OPS 16

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


// helm control variables
int16_t pitch, yaw, roll, throttle;
int8_t pitchAdjust = 0, yawAdjust = 0, rollAdjust = 0; // for ramping the direction input

// op control variables
// state == 0 to DELAY_OP-1 -> off / debounce, state == DELAY_OP -> on
uint8_t opState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// this controls whether the python script closes or not, 1 == run
boolean state = 1;


//// helper functions
//

// i/o expander helpers
uint8_t requestDevice(uint8_t address) // ping the device / send start command
{
  Wire.beginTransmission(IO_ADDR_BASE | address);
  return (Wire.endTransmission());
}

boolean requestData(uint8_t address) // returns success fail
{
  return (Wire.requestFrom(IO_ADDR_BASE | address, IO_DATA_LENGTH) == IO_DATA_LENGTH);
}

uint16_t readData() // get the data and stuff it into a 16-bit unsigned value
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

int16_t getDirection(int16_t current, int8_t adj, boolean stick)
{
  // reset current if no stick flag or if adj and current are opposite signs
  if (!(adj || stick) || (current > 0 && adj < 0) || (current < 0 && adj > 0))
  {
    current = 0;
  }
  else
  {
    current += adj; // adjust current direction

    if (current > DIR_MAX) // keep the current direction in bounds
      current = DIR_MAX;
    else if (current < DIR_MIN)
      current = DIR_MIN;
  }

  return (current);
}

uint16_t getThrottle(uint8_t pin) // get throttle input
{
  return (analogRead(pin));
}


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


//// update fucntions
//

void updateState()
{
  // tells the python client when to close
  state = digitalRead(PIN_ENABLE);
  digitalWrite(PIN_LED, state);
  Serial.println(state);
}

void updateHelm()
{
  pitchAdjust = getAdjustment(PITCH_PINS, pitchAdjust);
  pitch = getDirection(pitch, pitchAdjust, PITCH_STICK);
  Serial.println(pitch);

  rollAdjust = getAdjustment(ROLL_PINS, rollAdjust);
  roll = getDirection(roll, rollAdjust, ROLL_STICK);
  Serial.println(roll);

  yawAdjust = getAdjustment(YAW_PINS, yawAdjust);
  yaw = getDirection(yaw, yawAdjust, YAW_STICK);
  Serial.println(yaw);

  throttle = getThrottle(PIN_THROTTLE);
  Serial.println(throttle);
}

void updateOps()
{
  uint8_t ops = 0;
  uint8_t index = 0;

  if (requestDevice(IO_DEV_OPS) == I2C_SUCCESS) // send start, skip if failure
  {
    if (requestData(IO_DEV_OPS)) // ask for data, skip if failure
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
          opState[index] =  ((1 << index) & data) ? DELAY_OP : 0;
          ops++;
        }
      }
    }
  }

  if (ops > 0) // if we have at least one operation to send
  {
    Serial.println(ENABLE);

    for (index = 0; index < OPS; index++)
    {
      if (opState[index] == DELAY_OP)
        Serial.println(index); // print it
    }
  }

  Serial.println(DISABLE);
}


//// built-in arduino functions
//

void setup()
{
  setupSerial();

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
    updateHelm();
    updateOps();
  }

  delay(DELAY_LOOP);
  Serial.flush();
}

