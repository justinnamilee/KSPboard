byte pinLED = 6, pinSwitch = 2;
int delayTime = 10;


void setup()
{
  Serial.begin(115200);
  
  pinMode(pinLED, OUTPUT); // debug led
  pinMode(pinSwitch, INPUT_PULLUP); // active-low switch

  digitalWrite(pinLED, HIGH);
}

void loop()
{
  if (digitalRead(pinSwitch) == 1)
  {
    digitalWrite(pinLED, LOW);
    
    Serial.print(0);
    Serial.print(1);

    for (;;) {} // loop forever
  }
}
