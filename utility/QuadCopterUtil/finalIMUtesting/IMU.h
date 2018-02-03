// -------------------- SW110-IMULibrary-R06 -------------------- //

#ifndef IMU_h
#define IMU_h

#include <Wire.h>
#include <Arduino.h>

#define RAW_TO_G 0.00006103515625
#define RAW_TO_GAUSS 0.0001220703125
#define RAW_TO_RAD_PER_SEC 0.0001304948934
#define RAW_TO_DEG_PER_SEC 0.00747680663
#define RAD_TO_DEG 57.295779513
#define X 0
#define Y 1
#define Z 2

#define GYRO_CORRECTION_ANGLE_THRESH_RAD 0.0872
#define GYRO_CORRECTION_TIME_THRESH 2000

#define GYRO_ADDRESS 0x6B
#define GYRO_WHO_AM_I 0x0F
#define GYRO_CTRL1 0x20
#define GYRO_CTRL2 0x21
#define GYRO_CTRL3 0x22
#define GYRO_CTRL4 0x23
#define GYRO_CTRL5 0x24
#define GYRO_REFERENCE 0x25
#define GYRO_OUT_TEMP 0x26
#define GYRO_STATUS 0x27
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D
#define GYRO_FIFO_CTRL 0x2E
#define GYRO_FIFO_SRC 0x2F
#define GYRO_IG_CFG 0x30
#define GYRO_IG_SRC 0x31
#define GYRO_IG_THS_XH 0x32
#define GYRO_IG_THS_XL 0x33
#define GYRO_IG_THS_YH 0x34
#define GYRO_IG_THS_YL 0x35
#define GYRO_IG_THS_ZH 0x36
#define GYRO_IG_THS_ZL 0x37
#define GYRO_IG_DURATION 0x38
#define GYRO_LOW_ODR 0x39
#define GYRO_WHO_AM_I_ANSWER 0xD7

#define TEMP_ALT_ADDRESS 0x5D
#define TEMP_ALT_REF_P_XL 0x08
#define TEMP_ALT_REF_P_L  0x09
#define TEMP_ALT_REF_P_H  0x0A
#define TEMP_ALT_WHO_AM_I  0x0F
#define TEMP_ALT_RES_CONF  0x10
#define TEMP_ALT_CTRL_REG1  0x20
#define TEMP_ALT_CTRL_REG2  0x21
#define TEMP_ALT_CTRL_REG3  0x22
#define TEMP_ALT_INT_CFG_REG  0x23
#define TEMP_ALT_INT_SOURCE_REG  0x24
#define TEMP_ALT_THS_P_LOW_REG  0x25
#define TEMP_ALT_THS_P_HIGH_REG  0x26
#define TEMP_ALT_STATUS_REG  0x27
#define TEMP_ALT_PRESS_POUT_XL_REH  0x28
#define TEMP_ALT_PRESS_OUT_L  0x29
#define TEMP_ALT_PRESS_OUT_H  0x2A
#define TEMP_ALT_TEMP_OUT_L  0x2B
#define TEMP_ALT_TEMP_OUT_H  0x2C
#define TEMP_ALT_AMP_CTRL  0x30
#define TEMP_ALT_WHO_AM_I_ANSWER 0xBB

#define ACCEL_MAG_ADDRESS 0x1D
#define ACCEL_MAG_TEMP_OUT_L 0x05
#define ACCEL_MAG_TEMP_OUT_H 0x06
#define ACCEL_MAG_STATUS_M 0x07
#define ACCEL_MAG_OUT_X_L_M 0x08
#define ACCEL_MAG_OUT_X_H_M 0x09
#define ACCEL_MAG_OUT_Y_L_M 0x0A
#define ACCEL_MAG_OUT_Y_H_M 0x0B
#define ACCEL_MAG_OUT_Z_L_M 0x0C
#define ACCEL_MAG_OUT_Z_H_M 0x0D
#define ACCEL_MAG_WHO_AM_I 0x0F
#define ACCEL_MAG_INT_CTRL_M 0x12
#define ACCEL_MAG_INT_SRC_M 0x13
#define ACCEL_MAG_INT_THS_L_M 0x14
#define ACCEL_MAG_INT_THS_H_M 0x15
#define ACCEL_MAG_OFFSET_X_L_M 0x16
#define ACCEL_MAG_OFFSET_X_H_M 0x17
#define ACCEL_MAG_OFFSET_Y_L_M 0x18
#define ACCEL_MAG_OFFSET_Y_H_M 0x19
#define ACCEL_MAG_OFFSET_Z_L_M 0x1A
#define ACCEL_MAG_OFFSET_Z_H_M 0x1B
#define ACCEL_MAG_REFERENCE_X 0x1C
#define ACCEL_MAG_REFERENCE_Y 0x1D
#define ACCEL_MAG_REFERENCE_Z 0x1E
#define ACCEL_MAG_CTRL0 0x1F
#define ACCEL_MAG_CTRL1 0x20
#define ACCEL_MAG_CTRL2 0x21
#define ACCEL_MAG_CTRL3 0x22
#define ACCEL_MAG_CTRL4 0x23
#define ACCEL_MAG_CTRL5 0x24
#define ACCEL_MAG_CTRL6 0x25
#define ACCEL_MAG_CTRL7 0x26
#define ACCEL_MAG_STATUS_A 0x27
#define ACCEL_MAG_OUT_X_L_A 0x28
#define ACCEL_MAG_OUT_X_H_A 0x29
#define ACCEL_MAG_OUT_Y_L_A 0x2A
#define ACCEL_MAG_OUT_Y_H_A 0x2B
#define ACCEL_MAG_OUT_Z_L_A 0x2C
#define ACCEL_MAG_OUT_Z_H_A 0x2D
#define ACCEL_MAG_FIFO_CTRL 0x2E
#define ACCEL_MAG_FIFO_SRC 0x2F
#define ACCEL_MAG_IG_CFG1 0x30
#define ACCEL_MAG_IG_SRC1 0x31
#define ACCEL_MAG_IG_THS1 0x32
#define ACCEL_MAG_IG_DUR1 0x33
#define ACCEL_MAG_IG_CFG2 0x34
#define ACCEL_MAG_IG_SRC2 0x35
#define ACCEL_MAG_IG_THS2 0x36
#define ACCEL_MAG_IG_DUR2 0x37
#define ACCEL_MAG_CLICK_CFG 0x38
#define ACCEL_MAG_CLICK_SRC 0x39
#define ACCEL_MAG_CLICK_THS 0x3A
#define ACCEL_MAG_TIME_LIMIT 0x3B
#define ACCEL_MAG_TIME _LATENCY 0x3C
#define ACCEL_MAG_TIME_WINDOW 0x3D
#define ACCEL_MAG_Act_THS 0x3E
#define ACCEL_MAG_Act_DUR 0x3F
#define ACCEL_MAG_WHO_AM_I_ANSWER 0x49

class IMU {
    private:
        bool accelerometerON;
        bool gyroscopeON;
        bool altimeterON;
        int16_t accel[3];
        int16_t newAccel[3];
        int16_t mag[3];
        int16_t newMag[3];
        int16_t gyro[3];
        int16_t newGyro[3];
    
        float gyroIntegrator[3]; // angle in degrees from gyro perspective
        float accelLowPassCoeff; // proportion of new value (1 = no filter)
        float magLowPassCoeff;
        float gyroLowPassCoeff;
        float gyroPercentRoll; // percentage of gyro used
        float gyroPercentPitch;
        float gyroPercentYaw;
    
        long int prevTime;
        long int gyroCorrectionTimeStamp;
    
    public:
        // Setup Methods
        bool enableAccelerometer();
        bool enableGyroscope();
        bool enableAltimeter();
        bool enable();
        void calibrateGyro();
        void setAccelLowPassCoeff(float value);
        void setMagLowPassCoeff(float value);
        void setGyroLowPassCoeff(float value);
        void setGyroPercentRoll(float value);
        void setGyroPercentPitch(float value);
        void setGyroPercentYaw(float value);
        /*
        bool disableAccelerometer() const;
        bool disableGyroscope() const;
        bool disableAltimeter() const;
        */
        // Accessors
        void readIMU();
        float pressure();
        float temperature();
        float getRoll();
        float getPitch();
        float getYaw();
        float getAccel(int axis);
        float getMag(int axis);
        float getGyro(int axis);
        // Constructor
        IMU();
    
    private:
        // I2C Methods
        void writeToReg(byte deviceAddress, byte regAddress, byte value) const;
        bool testComms(byte address, byte whoAmI, byte whoAmIAnswer) const;
        void readAcceleration();
        void readMagnetometer();
        void readGyroscope();
        int getRawPressure();
        int getRawTemperature();
        void gyroCorrection();
};

#endif


