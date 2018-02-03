#include "Config.h"

//Cyclone Robotics Main Flight File

bool toggle = 1;

void setup() {
  Serial.begin(9600);
  
  analogReference(EXTERNAL);
  
  pinMode(BUTTON, INPUT);
  
  initComm();
  initIMU();
  initLED();
  initPID();
  initMotors();
  
  armLEDState = off;
  motorState = idle;
  
  setThrottleAll(100);
  delay(3000);
  setThrottleAll(-1);

  imu.calibrateGyro();

  throttle = 0;

}

void loop() {
  
  //digitalWrite(SERVO,toggle);
  //toggle = !toggle;

  //Process User Commands
  updateComm();
  
  //Process R/P/Y
  updateOrientation();

  //Compute PID Outputs
  updatePID();

  //Set Motor Throttles
  updateMotors();

  //Run LED S/M
  updateLED();
  
}
