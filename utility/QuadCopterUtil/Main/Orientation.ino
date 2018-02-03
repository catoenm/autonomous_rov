
//Cyclone Robotics Orientation File

void updateOrientation(){
  imu.readIMU();
  
  rollInput = imu.getRoll();
  pitchInput = imu.getPitch();
  yawInput = imu.getYaw();

  dRollInput = imu.getGyro(X);
  dPitchInput = imu.getGyro(Y);
  dYawInput = imu.getGyro(Z);

/*

  if (printOrientation++ > 50) {
    printOrientation = 0;
    Serial.print(F("Orientation: "));
    Serial.print(pitchInput);
    Serial.print(F(" "));
    Serial.print(rollInput);
    Serial.print(F(" "));
    Serial.print(yawInput);
    Serial.println(F(""));
  }

  Serial.print(imu.getGyro(X));
  Serial.print(',');
  Serial.print(imu.getGyro(Y));
  Serial.print(',');
  Serial.println(imu.getGyro(Z));
  */

}
void initIMU(){
  imu.enable();
}
