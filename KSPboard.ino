/// Justin Lee <justin at kitten dot pink>
//
// Arduino driver for the KSPboard game
// controller.  See LICENSE and README
// for more information.


#define ENABLE 1
#define DISABLE 0

#define SERIAL_SPEED 115200

#define PIN_LED 13

#define LOOP_DELAY 1
#define START_DELAY 5


byte pinSwitch = 2;

volatile byte pitch, yaw, roll, throttle;

byte state = 1;


void setupSerial()
{
  Serial.begin(SERIAL_SPEED);
  Serial.println();
  Serial.flush();

  Serial.print(1); // enable code, super secure
  Serial.print(1);
  Serial.print(1);
}


void updateState()
{
  // tells the python client when to close
  state = digitalRead(pinSwitch);
  digitalWrite(PIN_LED, state);

  if (state)
    Serial.print(ENABLE);
  else
    Serial.print(DISABLE);
}

void updateHelm()
{
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


void setup()
{
  pinMode(PIN_LED, OUTPUT); // debug led
  digitalWrite(PIN_LED, HIGH);

  pinMode(pinSwitch, INPUT_PULLUP); // active-low switch

  setupSerial();

  delay(START_DELAY);
}

void loop()
{
  if (state)
  {
    updateState();
    updateHelm();
    updateOps();
  }

  delay(LOOP_DELAY);
}
