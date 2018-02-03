
//Cyclone Robotics LED Control File

void updateLED(){
  switch(armLEDState) {
    
    case on:
      if(millis()-ledTimeStamp > 70) {
        digitalWrite(STAR, 0);
        digitalWrite(PORT, 0);
        ledTimeStamp = millis();
        armLEDState = off;
      }
      break;
      
    case off:
      if(millis()-ledTimeStamp > 1500) {
        digitalWrite(STAR, 1);
        digitalWrite(PORT, 1);
        ledTimeStamp = millis();
        armLEDState = on;
      }
      break;
  }
}

void initLED(){
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(STAR, OUTPUT);
  pinMode(PORT, OUTPUT);

  digitalWrite(STAR, 0);
  digitalWrite(PORT, 0);
}
