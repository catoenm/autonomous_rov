// -------------------- SW110-IMULibrary-R06 -------------------- //

#include "imu.h"

IMU::IMU() {
    Wire.begin();
    accelerometerON = false;
    gyroscopeON = false;
    altimeterON = false;
    accelLowPassCoeff = 0.25;
    magLowPassCoeff = 0.5;
    gyroLowPassCoeff = 0.95;
    gyroPercentRoll = 1.0;
    gyroPercentPitch = 1.0;
    gyroPercentYaw = 1.0;
    gyroCorrectionTimeStamp = 0;
  
    for (int i = 0; i < 3; i++) {
        gyroIntegrator[i] = 0;
        gyroDrift[i] = 0;
    }
    prevTime = 0;

}

void IMU::writeToReg(byte deviceAddress, byte regAddress, byte value) const {
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(value);
    Wire.endTransmission();
}

bool IMU::testComms(byte address, byte whoAmI, byte whoAmIAnswer) const {
    Wire.beginTransmission(address);
    Wire.write(whoAmI);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    if (Wire.available() && Wire.read() == whoAmIAnswer) {
        return true;
    }
    return false;
}

bool IMU::enableAccelerometer() {
    bool comm = testComms(ACCEL_MAG_ADDRESS,ACCEL_MAG_WHO_AM_I,ACCEL_MAG_WHO_AM_I_ANSWER);
    writeToReg(ACCEL_MAG_ADDRESS,ACCEL_MAG_CTRL1,0x87); // CTRL_1 -> 1600 Hz accelerometer output, All axes enbabled
    writeToReg(ACCEL_MAG_ADDRESS,ACCEL_MAG_CTRL5,0x14); // CTRL_5 -> 100 Hz magnetometer output
    writeToReg(ACCEL_MAG_ADDRESS,ACCEL_MAG_CTRL6,0x00); // CTRL_6 -> 2 Gauss range
    writeToReg(ACCEL_MAG_ADDRESS,ACCEL_MAG_CTRL7,0x00); // CTRL_7 -> Magnetometer mode = continous-conversion
    if(comm) {
        accelerometerON = true;
        return true;
    }
    accelerometerON = false;
    return false;
}

bool IMU::enableGyroscope() {
    bool comm = testComms(GYRO_ADDRESS,GYRO_WHO_AM_I,GYRO_WHO_AM_I_ANSWER);
    writeToReg(GYRO_ADDRESS,GYRO_CTRL1,0x0F); // CTRL_1 -> TBC
    if(comm) {
        gyroscopeON = true;
        return true;
    }
    gyroscopeON = false;
    return false;
}

bool IMU::enableAltimeter() {
    bool comm = testComms(TEMP_ALT_ADDRESS,TEMP_ALT_WHO_AM_I,TEMP_ALT_WHO_AM_I_ANSWER);
    writeToReg(TEMP_ALT_ADDRESS,TEMP_ALT_CTRL_REG1,0xE0); // CTRL_1 -> TBC
    if(comm) {
        altimeterON = true;
        return true;
    }
    altimeterON = false;
    return false;
}

bool IMU::enable() {
    bool accel = enableAccelerometer();
    bool gyro = enableGyroscope(); 
    bool alt = enableAltimeter();
    return accel && gyro && alt;
}

void IMU::setAccelLowPassCoeff(float value) {
    accelLowPassCoeff = value;
}

void IMU::setMagLowPassCoeff(float value) {
    magLowPassCoeff = value;
}

void IMU::setGyroLowPassCoeff(float value) {
    gyroLowPassCoeff = value;
}

void IMU::setGyroPercentRoll(float value) {
    gyroPercentRoll = value;
}

void IMU::setGyroPercentPitch(float value) {
    gyroPercentPitch = value;
}

void IMU::setGyroPercentYaw(float value) {
    gyroPercentYaw = value;
}

/*
 bool IMU::disableAccelerometer() const {
 
 }
 
 bool IMU::disableGyroscope() const {
 
 }
 
 bool IMU::disableAltimeter() const {
 
 }
 */

void IMU::readAcceleration() {
    // Setup
    Wire.beginTransmission(ACCEL_MAG_ADDRESS);
    Wire.write(ACCEL_MAG_OUT_X_L_A | (1 << 7));  // starting with register 0x28 (OUT_X_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(ACCEL_MAG_ADDRESS,(byte)6);  // request a total of 6 registers
    while (Wire.available() < 6);
    for (int i = 0; i < 3; i++){
        newAccel[i] = (int16_t)(Wire.read() | Wire.read()<<8); // read x, y & z axes
    }
}

void IMU::readMagnetometer() {
    // Setup
    Wire.beginTransmission(ACCEL_MAG_ADDRESS);
    Wire.write(ACCEL_MAG_OUT_X_L_M | (1 << 7));  // starting with register 0x08 (OUT_X_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(ACCEL_MAG_ADDRESS,(byte)6);  // request a total of 6 registers
    while (Wire.available() < 6);
    for (int i = 0; i < 3; i++){
        newMag[i] = (int16_t)(Wire.read() | Wire.read()<<8); // read x, y & z axes
    }
}

void IMU::readGyroscope() {
    // Setup
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(GYRO_OUT_X_L | (1 << 7));  // starting with register 0x28 (OUT_X_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(GYRO_ADDRESS,(byte)6);  // request a total of 6 registers
    while (Wire.available() < 6);
    for (int i = 0; i < 3; i++){
        newGyro[i] = (int16_t)(Wire.read() | Wire.read()<<8)*-1; // read x, y & z axes
    }
}

int IMU::getRawPressure() {
    // Setup
    Wire.beginTransmission(TEMP_ALT_ADDRESS);
    Wire.write(TEMP_ALT_PRESS_POUT_XL_REH | (1 << 7));  // starting with register 0x28 (PRESS_OUT_XL), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(TEMP_ALT_ADDRESS,(byte)3);  // request a total of 5 registers
    while (Wire.available() < 3);
    uint8_t pxl = Wire.read();
    uint8_t pl = Wire.read();
    uint8_t ph = Wire.read();
    return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

int IMU::getRawTemperature() {
    // Setup
    Wire.beginTransmission(TEMP_ALT_ADDRESS);
    Wire.write(TEMP_ALT_TEMP_OUT_L | (1 << 7));  // starting with register 0x2B (TEMP_OUT_L), assert MSB to 1
    Wire.endTransmission();
    // Read
    Wire.requestFrom(TEMP_ALT_ADDRESS,(byte)2);  // request a total of 5 registers
    while (Wire.available() < 2);
    uint8_t tl = Wire.read();
    uint8_t th = Wire.read();
    return (int16_t)(th << 8 | tl);
}

float IMU::getAccel(int axis) {
    return accel[axis] * RAW_TO_G;
}

float IMU::getMag(int axis) {
    return mag[axis] * RAW_TO_GAUSS;
}

float IMU::getGyro(int axis) {
    return (gyro[axis] - gyroDrift[axis])* RAW_TO_DEG_PER_SEC;
}

void IMU::readIMU() {
    // Read Values
    readAcceleration();
    readMagnetometer();
    readGyroscope();
    
    // Update time;
    long int currTime = millis();
    
    // Low pass filter & gyro integrator
    for (int i = 0; i < 3; i++) {
        // Low pass filter: updatedValue = old*(1-coeff) + new*coeff
        accel[i] = accel[i] * (1.0 - accelLowPassCoeff) + newAccel[i] * accelLowPassCoeff;
        mag[i] = mag[i] * (1.0 - magLowPassCoeff) + newMag[i] * magLowPassCoeff;
        gyro[i] = gyro[i] * (1.0 - gyroLowPassCoeff) + (newGyro[i] - gyroDrift[i]) * gyroLowPassCoeff;
        // Update gyro integrator
        gyroIntegrator[i] += (gyro[i]) * RAW_TO_RAD_PER_SEC * (currTime - prevTime) / 1000.0;
        //gyroCorrection();
    }
    
    // Set gyro range to -180 to 180 deg (same as compass output)
    if (gyroIntegrator[Z] > M_PI)
        gyroIntegrator[Z] = -M_PI+0.00001; // add tolerance to avoid boucing back an forth
    else if (gyroIntegrator[Z] < -M_PI)
        gyroIntegrator[Z] = M_PI-0.00001;
    
    // Update time
    prevTime = currTime;
}

float IMU::pressure() {
    return double(getRawPressure()) / 4096; // measured in mPa
}

float IMU::temperature() {
    return double(getRawTemperature()) / 480 + 42.5; // measured in degreeC
}

void IMU::gyroCorrection() {
  if(millis()-gyroCorrectionTimeStamp > GYRO_CORRECTION_TIME_THRESH) {
    float accelRoll = atan2(getAccel(X), sqrt(getAccel(Y)*getAccel(Y) + getAccel(Z)*getAccel(Z)));
    float accelPitch = atan2(getAccel(X), sqrt(getAccel(Y)*getAccel(Y) + getAccel(Z)*getAccel(Z)));
    if (abs(accelRoll) < GYRO_CORRECTION_ANGLE_THRESH_RAD) {
      gyroIntegrator[X] = accelRoll;
      digitalWrite(GREEN,1);
    }
    if(abs(accelPitch) < GYRO_CORRECTION_ANGLE_THRESH_RAD) {
      gyroIntegrator[Y] = accelPitch;
      digitalWrite(RED,1);
    }
    gyroCorrectionTimeStamp = millis();
  }
  else if (millis()-gyroCorrectionTimeStamp > 300) {
    digitalWrite(GREEN,0);
    digitalWrite(RED,0);
  }
}

float IMU::getRoll() {
    return RAD_TO_DEG * (atan2((-1.0)*getAccel(Y), getAccel(Z)) * (1-gyroPercentRoll) + gyroIntegrator[X] * gyroPercentRoll);
}

float IMU::getPitch() {
    return RAD_TO_DEG * (atan2(getAccel(X), sqrt(getAccel(Y)*getAccel(Y) + getAccel(Z)*getAccel(Z))) * (1-gyroPercentPitch) + gyroIntegrator[Y] * gyroPercentPitch);
}

float IMU::getYaw()  {
    float roll = getRoll() / RAD_TO_DEG;
    float pitch = getPitch() / RAD_TO_DEG;
    return (atan2(  (getMag(Z) * sin(roll))
                  - (getMag(Y) * cos(roll)),
                    (getMag(X) * cos(pitch))
                  + (getMag(Y) * sin(pitch) * sin(roll))
                  + (getMag(Z) * sin(pitch) * cos(roll)))
            * (1.0 - gyroPercentYaw) + gyroIntegrator[Z] * gyroPercentYaw) * RAD_TO_DEG;
}

void IMU::calibrateGyro(){
  int timeElap = millis();
  float counter = 0;
  long tempDrift[3];
  long tempRoll = 0;
  long tempPitch = 0;
  tempDrift[0] = 0;
  tempDrift[1] = 0;
  tempDrift[2] = 0;
  
  digitalWrite(RED,1);
  
  gyroPercentRoll = 0;
  gyroPercentPitch = 0;
  
  while(millis() - timeElap <= 10000){
    readIMU();
    tempDrift[0] += newGyro[0];
    tempDrift[1] += newGyro[1];
    tempDrift[2] += newGyro[2];
    tempPitch += getPitch();
    tempRoll += getRoll();
    counter++;
  }
  
  gyroPercentRoll = 1.0;
  gyroPercentPitch = 1.0;
  
  gyroDrift[0] = tempDrift[0]/counter;
  gyroDrift[1] = tempDrift[1]/counter;
  gyroDrift[2] = tempDrift[2]/counter;
  
  gyro[X] = newGyro[X] - gyroDrift[0];
  gyro[Y] = newGyro[Y] - gyroDrift[1];
  gyro[Z] = newGyro[Z] - gyroDrift[2];
  
  gyroIntegrator[0] = (tempRoll/counter)/RAD_TO_DEG;
  gyroIntegrator[1] = (tempPitch/counter)/RAD_TO_DEG;
    
  digitalWrite(RED, 0);
  
  prevTime = millis();
}








