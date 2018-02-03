#include "config.h"
#include "quaternionFilters.h"
#include "MadgwickAHRS.h"

#define MS_PER_S (1000.0)
#define GYRO_ONLY_SETTLE_TIME_MS (60000)
#define GYRO_ONLY_INITIAL_CALIB_MS (5000)
#define GYRO_ONLY_CONTINUOUS_CALIB_INTERVAL_MS (300)
#define GYRO_ONLY_YAW_ROC_THRESH_DEG (0.5)

typedef struct GyroOnlyYawDriftState {
  float total_drifted_yaw;
  float corrected_yaw;
  unsigned long prev_sample_time_ms;
} YawDriftState;

typedef struct GyroOnlyCalibrationState {
  float prev_sample_yaw;
  float prev_sample_yaw_roc; // roc = rate of change, per second
  unsigned long prev_sample_time_ms;
  bool yaw_calibration_done;
} YawCalibrationState;

Madgwick madgwick_filter;
YawDriftState yaw_drift_state = {0};
YawCalibrationState yaw_calib_state = {0};

/*
  All imu related code, interface updates axes of freedom
 */
void imu_setup(void) {
  // TWBR = 12;  // 400 kbit/sec I2C speed

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    Serial.print("X-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis factory sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[2], 2);

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);

    Serial.println("Magnetometer:");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(myIMU.factoryMagCalibration[2], 2);

    Serial.println("Calibrating yaw using gyro drift. DO NOT MOVE ROV!");
    delay(2000);

    Serial.println("DO NOT MOVE ROV!");

    yaw_calib_state.yaw_calibration_done = false;
    unsigned long startTime = millis();
    while((millis() - startTime) < GYRO_ONLY_SETTLE_TIME_MS) {
      update_imu_data();
    }
    Serial.println("IMU settled");

    while(!update_imu_data());

    startTime = millis();
    float startYaw = madgwick_filter.getYaw();
    while((millis() - startTime) < GYRO_ONLY_INITIAL_CALIB_MS) {
      update_imu_data();
    }
    while(!update_imu_data());

    float endYaw = madgwick_filter.getYaw();
    unsigned long endTime = millis();
    float yaw_diff = endYaw - startYaw;
    Serial.print("Yaw diff: "); Serial.println(yaw_diff, 10);

    yaw_calib_state.prev_sample_yaw_roc = (endYaw - startYaw) / ((endTime - startTime) / MS_PER_S);
    Serial.print("Yaw roc /s "); Serial.println(yaw_calib_state.prev_sample_yaw_roc, 10);
    yaw_calib_state.prev_sample_time_ms = millis();
    yaw_drift_state.prev_sample_time_ms = yaw_calib_state.prev_sample_time_ms;

    yaw_calib_state.prev_sample_yaw = endYaw;
    yaw_calib_state.yaw_calibration_done = true;
    Serial.println("IMU calibrated");


  } else {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

// return: whether or not new data is available
bool update_imu_data(void) {
  bool new_data = false;
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

    myIMU.mx *= myIMU.magScale[0];
    myIMU.my *= myIMU.magScale[1];
    myIMU.mz *= myIMU.magScale[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();


  // Comment from another version of Kris' library regarding logic for coordinates in his original MPU orientation. We've changed
  // the MPU orientation has changed (accel +x points north, accel +y points east, and +z is down) but the below logic is the same:

  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
  // we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
  // positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
  // function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  // Pass gyro rate as rad/s
  // MahonyQuaternionUpdate(-myIMU.ay, -myIMU.ax, myIMU.az, myIMU.gy * DEG_TO_RAD,
  //                        myIMU.gx * DEG_TO_RAD, -myIMU.gz * DEG_TO_RAD, myIMU.mx,
  //                        myIMU.my, myIMU.mz, myIMU.deltat);
  myIMU.delt_t = millis() - myIMU.count;

  madgwick_filter.begin(1000.0 / abs(myIMU.delt_t));
  madgwick_filter.updateIMU(myIMU.gy, myIMU.gx, -myIMU.gz, -myIMU.ay, -myIMU.ax, myIMU.az);

  // Serial print and/or display at 0.5 s rate independent of data rates


  // update LCD once per half-second independent of read rate
  if (myIMU.delt_t > IMU_REFRESH_RATE_MS) {
    new_data = true;

    if(IMU_SERIAL_DEBUG) {
      //        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
      //        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
      //        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
      //        Serial.println(" mg");

      //        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
      //        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
      //        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
      //        Serial.println(" deg/s");

      //        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
      //        Serial.print(" my = "); Serial.print((int)myIMU.my);
      //        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
      //        Serial.println(" mG");

      //        Serial.print("q0 = ");  Serial.print(*getQ());
      //        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
      //        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
      //        Serial.print(" qz = "); Serial.println(*(getQ() + 3));

      Serial.print((int)myIMU.mx); Serial.print(" ");
      Serial.print((int)myIMU.my); Serial.print(" ");
      Serial.println((int)myIMU.mz);
    }

  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                  * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                  * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;

    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    //  8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw  -= 10;
    myIMU.roll *= RAD_TO_DEG;

    if(IMU_SERIAL_DEBUG) {

     Serial.print("rate = ");
     Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
     Serial.println(" Hz");
    }

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 500)

  if (new_data && yaw_calib_state.yaw_calibration_done) {
    yaw_drift_state.total_drifted_yaw += ((millis() - yaw_drift_state.prev_sample_time_ms) / MS_PER_S) * yaw_calib_state.prev_sample_yaw_roc;

    yaw_drift_state.prev_sample_time_ms = millis();

    float read_raw = madgwick_filter.getYaw();
    float actual_yaw = read_raw - yaw_drift_state.total_drifted_yaw;

    if (actual_yaw < 0) {
      yaw_drift_state.total_drifted_yaw = 0;
      actual_yaw = 360;
    } else if (actual_yaw > 360) {
      yaw_drift_state.total_drifted_yaw = 0;
      actual_yaw = 0;
    }
    if ((millis() - yaw_calib_state.prev_sample_time_ms) > GYRO_ONLY_CONTINUOUS_CALIB_INTERVAL_MS) {
      float new_yaw_roc = (read_raw - yaw_calib_state.prev_sample_yaw) / ((millis() - yaw_calib_state.prev_sample_time_ms) / MS_PER_S);
      if (abs(new_yaw_roc) < GYRO_ONLY_YAW_ROC_THRESH_DEG) {
        yaw_calib_state.prev_sample_yaw_roc = new_yaw_roc;
      }
      yaw_calib_state.prev_sample_time_ms = millis();
      yaw_calib_state.prev_sample_yaw = read_raw;
    }

    yaw_drift_state.corrected_yaw = actual_yaw;

    if (IMU_SERIAL_DEBUG) {
      Serial.print(actual_yaw, 2); Serial.print(" ");
      Serial.print(read_raw, 2); Serial.print(" ");
      Serial.print(yaw_drift_state.total_drifted_yaw, 2); Serial.print(" ");
      Serial.println(yaw_calib_state.prev_sample_yaw_roc, 6);
    }
  }
  return new_data;
}

float imu_get_yaw(void) {
  return yaw_drift_state.corrected_yaw;
}

float imu_get_pitch(void) {
  return madgwick_filter.getPitch();
}

float imu_get_roll(void) {
  return madgwick_filter.getRoll();
}
