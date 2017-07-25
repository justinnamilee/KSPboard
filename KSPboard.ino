/// Justin Lee <justin at kitten dot pink>
//
// Arduino driver for the KSPboard game
// controller.  See LICENSE and README
// for more information.


/// pin configurations and constants
//

#define ENABLE 1
#define DISABLE 0

// serial
#define SERIAL_SPEED 115200

// pins
#define PIN_LED 13 // status
#define PIN_ENABLE 3 // this should be high to run
// helm pins
#define PIN_PITCH_U 12
#define PIN_PITCH_D 11
#define PIN_ROLL_L 10
#define PIN_ROLL_R 9
#define PIN_YAW_L 8
#define PIN_YAW_R 7

// delays
#define LOOP_DELAY 1
#define START_DELAY 5


byte pinSwitch = 2;

byte pitch, yaw, roll, throttle;

byte state = 1;


//// helper functions
//

byte readDirection(byte pinH, byte pinL)
{
  return (digitalRead(pinH) ? 0xFF : (digitalRead(pinL) ? 0x00 : 0x7F)); // high, low, or neutral
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


//// update fucntions
//

void updateState()
{
  // tells the python client when to close
  state = digitalRead(PIN_ENABLE);
  digitalWrite(PIN_LED, state);

  if (state)
    Serial.print(ENABLE);
  else
    Serial.print(DISABLE);
}

void updateHelm()
{
  // get new values
  pitch = readDirection(PIN_PITCH_U, PIN_PITCH_D);
  roll = readDirection(PIN_ROLL_R, PIN_ROLL_L);
  yaw = readDirection(PIN_YAW_R, PIN_YAW_L);

  // helm are always sent without enable code
  Serial.print(pitch);
  Serial.print(yaw);
  Serial.print(roll);

  Serial.print(throttle);
}

void updateOps()
{
  Serial.print(DISABLE);
}


//// built-in arduino functions
//

void setup()
{
  setupSerial();
  setupState();
  setupHelm();

  delay(START_DELAY);
}

void loop()
{
  updateState();

  if (state)
  {
    updateHelm();
    updateOps();
  }

  delay(LOOP_DELAY);
}

