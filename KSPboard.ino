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

// ops pins
#define PIN_OP_LAUNCH 0
#define PIN_OP_STAGE 0
#define PIN_OP_ACG3 0
#define PIN_OP_ACG5 0

// delays
#define DELAY_LOOP 100
#define DELAY_START 30
#define DELAY_OP 100

// misc
#define OPS (sizeof(opPin) / sizeof(uint8_t))

// helm
#define RAMP_ADJUST 3
#define RAMP_MAX 111
#define RAMP_MIN -RAMP_MAX
#define DIR_MAX 1023
#define DIR_MIN -1023

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
uint8_t opState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// operation pins set to zero are skipped
uint8_t opPin[] =
{
  0,             // null
  0,             // action group 1
  0,             // '' 2
  PIN_OP_ACG3,   // '' 3
  0,             // '' 4
  PIN_OP_ACG5,   // '' 5
  0,             // '' 6
  0,             // '' 7
  0,             // '' 8
  0,             // '' 9
  0,             // '' 10
  PIN_OP_LAUNCH, //
  PIN_OP_STAGE   //
};

// this controls whether the python script closes or not, 1 == run
boolean state = 1;

// specialized control functions



//// helper functions
//

int8_t getAdjustment(uint8_t pinH, uint8_t pinL, int8_t adj) // get control input
{
  switch (digitalRead(pinH) ? HIGH : (digitalRead(pinL) ? LOW : NEUTRAL))
  {
    case HIGH:
      if ((adj += RAMP_ADJUST) > RAMP_MAX)
        adj = RAMP_MAX;
      break;

    case LOW:
      if ((adj -= RAMP_ADJUST) < RAMP_MIN)
        adj = RAMP_MIN;
      break;

    case NEUTRAL:
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
    current += adj;

    if (current > DIR_MAX)
      current = DIR_MAX;
    else 
      if (current < DIR_MIN)
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
{ // disabled while building the joystick control
  /*pinMode(PIN_OP_LAUNCH, INPUT_PULLUP);
    pinMode(PIN_OP_STAGE, INPUT_PULLUP);
    pinMode(PIN_OP_ACG3, INPUT_PULLUP);
    pinMode(PIN_OP_ACG5, INPUT_PULLUP);*/
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


  for (index = 0; index < OPS; index++)
  {
    if (!opPin[index]) // if the pin is zero, skip
      continue;

    if (opState[index] > 0) // check for debounce on current pin
    {
      opState[index]--; // decrement debounce counter for current state
    }
    else // if not we can check to see if it's triggered
    {
      opState[index] = digitalRead(opPin[index]) ? DELAY_OP : 0;
      ops++;
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
    //updateOps();
  }

  delay(DELAY_LOOP);

  Serial.println();
  Serial.flush();
}

