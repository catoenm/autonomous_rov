#include "IMU.h"

enum State {
  wait,
  sendMessage
};

IMU imu;
int roll, pitch, yaw;
State state;
long timeStamp;

void setup() {
    Wire.begin();
    Serial.begin(115200);
    if(imu.enable())
      Serial.println("Setup successful");
    else
      Serial.println("Setup failed");
    state = wait;
    timeStamp = millis();
} 

void loop() {
    imu.readIMU();
    
    switch(state){
      case wait:
        if(millis()-timeStamp > 100) {
          state = sendMessage;
          timeStamp = millis();
        }   
        break;
      case sendMessage:
        Serial.print(F("Orientation: "));
        Serial.print(imu.getPitch());
        Serial.print(F(" "));
        Serial.print(imu.getRoll());
        Serial.print(F(" "));
        Serial.print(imu.getYaw());
        Serial.println(F(""));
        state = wait;
        break;
    }
}


