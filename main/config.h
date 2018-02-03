#ifndef __CONFIG_H__
#define __CONFIG_H__
/* Included libraries */
#include "MPU9250.h"
#include <Servo.h>
#include "SparkFun_MS5803_I2C.h"

/* Configuration Properties */
#define COMM_DEBUG_CRC (false)
#define IMU_SERIAL_DEBUG (false)    // Set to true to get Serial output for debugging
#define IMU_REFRESH_RATE_MS (10) // Used to determine time between polls on the IMU
#define MAX_FORWARD (1860)
#define MAX_REVERSE (1060)
#define NEUTRAL ((MAX_FORWARD + MAX_REVERSE) >> 1)
#define MESSAGE_SIZE (5)
#define SURFACE_PROBE_THRESH (600)
#define PROTOCOL_START_BYTE (0xFF)
#define PROTOCOL_SERIAL_TIMEOUT_MS (5)
#define CONTROLLER_DEAD_VALUE (127)
#define MOTOR_NEUTRAL (127)
#define UNATTENDED_TIMEOUT_MS (2000)
#define SWEEP_STEPS (10)

/* Pin definitions */
#define ESC1 (3)
#define ESC2 (5)
#define ESC3 (6)
#define ESC4 (9)
#define SURFACE_PROBE (A0)
#define STATUS_LED (13)
#define ELECTROMAG_P (51)
#define ELECTROMAG_N (53)
#define BASE_ALTITUDE (1655.0)

/* Typedefs */
#define ControllerComm (Serial1)
#define SURFACE_LED (50)
#define DANGER_LED (23)

/* Misc */
#define JOYSTICK_STATE_CRC_SIZE (sizeof(uint32_t))

/* Struct definitions */
enum SubState {
  ELECTROMAG_ATTACH,
  ELECTROMAG_DETACH,
  QUICK_DIVE,
  AUTO,
  JUST_BACK,
  RESTART_AUTO,
  SET_POTS
};

enum PotChooser {
  POT_TRIM,
  POT_L1,
  POT_L1L2,
  POT_L2,
  POT_L2L3,
  POT_L3
};

typedef struct JoystickState {
    uint8_t buttonEnum;
    uint8_t leftJoy;
    uint8_t rightJoy;
    uint8_t crc[JOYSTICK_STATE_CRC_SIZE];
} JoystickState;

typedef struct Log {
    uint32_t time;
    uint8_t buttonEnum;
    uint8_t leftJoy;
    uint8_t rightJoy;
    uint32_t pressure;
} SubLog;

#define JOYSTICK_STATE_DATA_SIZE (sizeof(JoystickState) - JOYSTICK_STATE_CRC_SIZE)
/* IMU object */
MPU9250 myIMU(NOT_SPI, 343.86, -175.99, -159.90, 1.31, 1.03, 0.79);
MS5803 depth_sensor(ADDRESS_HIGH);
Servo motors[4];
byte message [MESSAGE_SIZE];
int throttle = 0;
int angle = 0;
double pressure_abs, pressure_relative, pressure_baseline;
unsigned long last_log = 0;

unsigned long last_packet_receive_time;
/* States */
JoystickState joystick_state;
SubState sub_state;
unsigned long time_ref = 0;

#endif
