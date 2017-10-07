/// Justin Lee <justin at kitten dot pink>
//
// Arduino driver for the KSPboard game
// controller.  See LICENSE and README
// for more information.


/// pin configurations and constants
//

#define ENABLE 1
#define DISABLE 0

#define FULL 0xFF
#define HALF 0x7F
#define ZERO 0x00

// serial
#define SERIAL_SPEED 115200

// state pins
#define PIN_LED 13 // status
#define PIN_ENABLE 2 // this should be high to run

// helm pins
#define PIN_PITCH_U 12
#define PIN_PITCH_D 11
#define PIN_ROLL_L 10
#define PIN_ROLL_R 9
#define PIN_YAW_L 8
#define PIN_YAW_R 7
#define PIN_THROTTLE A0 // analog read

// ops pins
#define PIN_OP_LAUNCH 6
#define PIN_OP_STAGE 5
#define PIN_OP_ACG3 4
#define PIN_OP_ACG5 3

// delays
#define DELAY_LOOP 15
#define DELAY_START 30
#define DELAY_OP 100

// misc
#define OPS (sizeof(opPin) / sizeof(byte))

#define ANALOG_MAX (float)1023 // these two are for converting 0-1023 to 0-100
#define THOTTLE_MAX (float)255


// helm control variables
byte pitch, yaw, roll, throttle;

// op control variables
// state == 0 to DELAY_OP-1 -> off / debounce, state == DELAY_OP -> on
byte opState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// operation pins set to zero are skipped
byte opPin[] =
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
byte state = 1;


//// helper functions
//

byte readDirection(byte pinH, byte pinL) // get control input
{
  return (digitalRead(pinH) ? FULL : (digitalRead(pinL) ? ZERO : HALF));
}

byte readThrottle(byte pin) // get throttle input
{
  return ((byte) (((float)analogRead(pin) * THOTTLE_MAX) / ANALOG_MAX));
}


//// setup fucntions
//

void setupSerial()
{
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
  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_ENABLE, INPUT_PULLUP);
}

void setupHelm()
{
  pinMode(PIN_PITCH_U, INPUT_PULLUP);
  pinMode(PIN_PITCH_D, INPUT_PULLUP);
  pinMode(PIN_ROLL_L, INPUT_PULLUP);
  pinMode(PIN_ROLL_R, INPUT_PULLUP);
  pinMode(PIN_YAW_L, INPUT_PULLUP);
  pinMode(PIN_YAW_R, INPUT_PULLUP);
}

void setupOps()
{
  pinMode(PIN_OP_LAUNCH, INPUT_PULLUP);
  pinMode(PIN_OP_STAGE, INPUT_PULLUP);
  pinMode(PIN_OP_ACG3, INPUT_PULLUP);
  pinMode(PIN_OP_ACG5, INPUT_PULLUP);
}


//// update fucntions
//

void updateState()
{
  // tells the python client when to close
  state = digitalRead(PIN_ENABLE);
  digitalWrite(PIN_LED, state);

  if (state)
    Serial.println(ENABLE);
  else
    Serial.println(DISABLE);
}

void updateHelm()
{
  // get new values
  pitch = readDirection(PIN_PITCH_U, PIN_PITCH_D);
  roll = readDirection(PIN_ROLL_R, PIN_ROLL_L);
  yaw = readDirection(PIN_YAW_R, PIN_YAW_L);

  // helm are always sent without enable code
  Serial.println(pitch);
  Serial.println(yaw);
  Serial.println(roll);

  // read throttle
  throttle = readThrottle(PIN_THROTTLE);
  Serial.println(throttle);
}

void updateOps()
{
  byte ops = 0;
  byte index = 0;


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
    updateOps();
  }

  delay(DELAY_LOOP);
  Serial.flush();
}

