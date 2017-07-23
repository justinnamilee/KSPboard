/// Justin Lee <justin at kitten dot pink>
//
// Arduino driver for the KSPboard game
// controller.  See LICENSE and README.

byte pinLED = 6, pinSwitch = 2;
int delayTime = 10;

byte pitch, yaw, roll, throttle;

byte state = 1;


void setupSerial()
{
  Serial.begin(115200);
  Serial.println();
  Serial.flush();

  Serial.print(1);
  Serial.print(1);
  Serial.print(1);

  delay(5);
}


void updateState()
{
  state = digitalRead(pinSwitch);
  digitalWrite(pinLED, state);

  if (state)
    Serial.print(1);
  else
    Serial.print(0);
}

void updateHelm()
{
  Serial.print(pitch);
  Serial.print(yaw);
  Serial.print(roll);
  Serial.print(throttle);
}

void updateOps()
{
  Serial.print(0);
}


void setup()
{
  pinMode(pinLED, OUTPUT); // debug led
  pinMode(pinSwitch, INPUT_PULLUP); // active-low switch

  setupSerial();
}

void loop()
{
 updateState();
 updateHelm();
 updateOps();

 delay(1);
}
