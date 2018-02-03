/*
  Main setup and flow control
*/

#include "config.h"
#include "auto_config.h"

#define DEBUG_COMM 0
#define DEBUG_STATE 1
#define DEBUG_IMU 0
#define DEBUG_PRESSURE 0
#define LOGGING 0
#define DEBUG_LOGGING 0
#define DEBUG_MOTORS 0
#define DEBUG_POTS 0

void setup() {
  // Start serial and I2C
  Wire.begin();
  Serial.begin(115200);
  ControllerComm.setTimeout(PROTOCOL_SERIAL_TIMEOUT_MS);
  ControllerComm.begin(115200);

  pinMode(ELECTROMAG_P, OUTPUT);
  pinMode(ELECTROMAG_N, OUTPUT);
  pinMode(SURFACE_PROBE, INPUT);
  pinMode(SURFACE_LED, OUTPUT);

  // // Assign PWM pins to each ESC
  attach_motors();
  // // Wait 5 seconds at max voltage
  calibrate_motors();
  // Setup procedure for IMU
  // imu_setup();

  // Setup prodecure for depth sensor
  depth_sensor_setup();
  // Setting the polarity of the elctromagnet
  digitalWrite(ELECTROMAG_P, LOW);

  // Setup procedure for IMU
  imu_setup();
  throttle = CONTROLLER_DEAD_VALUE;
  angle = CONTROLLER_DEAD_VALUE;
  last_packet_receive_time = 0;

  comm_flush();
}

void loop() {
  if ((millis() - last_packet_receive_time) > UNATTENDED_TIMEOUT_MS) {
    throttle = CONTROLLER_DEAD_VALUE;
    angle = CONTROLLER_DEAD_VALUE;
    sub_state = ELECTROMAG_DETACH;
  }

  if(get_message_if_available(&joystick_state)) {
    sub_state = joystick_state.buttonEnum;
    last_packet_receive_time = millis();
    if(sub_state != SET_POTS) {
      throttle = joystick_state.leftJoy;
      angle = joystick_state.rightJoy;
    } else {
      config_auto_param(joystick_state.leftJoy, joystick_state.rightJoy);
    }
    if(DEBUG_COMM) {
       Serial.print("EM State: ");
       Serial.print(sub_state);
       Serial.print(" Throttle: ");
       Serial.print(throttle);
       Serial.print(" Angle: ");
       Serial.println(angle);
    }
  }

  if (update_imu_data()) {
    if(DEBUG_IMU) {

      // Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(imu_get_yaw(), 2);
      Serial.print(" ");
      Serial.print(imu_get_pitch(), 2);
      Serial.print(" ");
      Serial.println(imu_get_roll(), 2);
    }
  }

  switch(sub_state) {
    case ELECTROMAG_ATTACH:
      electromag_attract();
      motors_drive(throttle, angle);
      if(DEBUG_STATE) {
        Serial.print("Motors Up Running at Throttle: ");
        Serial.print(throttle);
        Serial.print(" and Direction: ");
        Serial.println(angle);
      }
      break;
    case ELECTROMAG_DETACH:
      electromag_repel();
      motors_drive(throttle, angle);
      if(DEBUG_STATE) {
        Serial.print("Motors Down Running at Throttle: ");
        Serial.print(throttle);
        Serial.print(" and Direction: ");
        Serial.println(angle);
      }
      break;
      case QUICK_DIVE:
        electromag_attract();
        motors_front_only(-240);
        delay(300);
        motors_front_only(127);
        delay(300);
        motors_front_only(240);
        delay(500);
        // Passive State
        sub_state = ELECTROMAG_ATTACH;
        motors_drive(0, 0);
        comm_flush();
        if(DEBUG_STATE) {
          Serial.println("Deep Diving");
        }
        break;
      case AUTO:
        switch(auto_state) {
          case START:
            time_ref = millis();
            auto_state = L1;
            Serial.println("Auto Start");
            break;
          case L1:
            if(millis() - time_ref > auto_values[POT_L1]) {
              auto_state = L1L2;
              sweep_motors(STRAIGHT_SPEED, MOTOR_NEUTRAL + auto_values[POT_TRIM], MOTOR_NEUTRAL, RIGHT_TURN_VAL);
              time_ref = millis();
            } else {
              motors_drive(STRAIGHT_SPEED, MOTOR_NEUTRAL + auto_values[POT_TRIM]);
            }
            break;
          case L1L2:
            if(millis() - time_ref > auto_values[POT_L1L2]) {
              auto_state = L2;
              sweep_motors(MOTOR_NEUTRAL, RIGHT_TURN_VAL, STRAIGHT_SPEED, MOTOR_NEUTRAL + auto_values[POT_TRIM]);
              time_ref = millis();
            } else {
              motors_drive(MOTOR_NEUTRAL, RIGHT_TURN_VAL);
            }
            break;
          case L2:
            if(millis() - time_ref > auto_values[POT_L2]) {
              auto_state = L2L3;
              sweep_motors(STRAIGHT_SPEED, MOTOR_NEUTRAL + auto_values[POT_TRIM], MOTOR_NEUTRAL, RIGHT_TURN_VAL);
              time_ref = millis();
            } else {
              motors_drive(STRAIGHT_SPEED, MOTOR_NEUTRAL + auto_values[POT_TRIM]);
            }
            break;
          case L2L3:
            if(millis() - time_ref > auto_values[POT_L2L3]) {
              auto_state = L3;
              time_ref = millis();
            } else {
              motors_drive(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
            }
            break;
          case L3:
            Serial.println("Auto End");
            motors_drive(MOTOR_NEUTRAL, MOTOR_NEUTRAL);
            break;
        }

        if(DEBUG_STATE) {
          Serial.println("In Autonomous Mode");
        }
        break;
      case JUST_BACK:
        electromag_repel();
        motors_back_only(throttle);
        if(DEBUG_STATE) {
          Serial.print("Running back at: ");
          Serial.println(throttle);
        }
        break;
      case RESTART_AUTO:
        auto_state = START;
        break;

  }
  on_surface();
  update_pressure();
  danger_zone();

  if(DEBUG_PRESSURE) {
    Serial.print("Absolute Pressure: ");
    Serial.print(pressure_abs);
    Serial.print(" Relative Pressure: ");
    Serial.println(pressure_relative);
  }

  if(LOGGING) {
    if(millis() - last_log > 50) {
      last_log = millis();
      SubLog log = {
        .time = millis(),
        .buttonEnum = sub_state,
        .leftJoy = throttle,
        .rightJoy = angle,
        .pressure = 100
      };

      if(DEBUG_LOGGING) {
          Serial.print(" ");
          Serial.print(log.time);
          Serial.print(" ");
          Serial.print(log.buttonEnum);
          Serial.print(" ");
          Serial.print(log.leftJoy);
          Serial.print(" ");
          Serial.print(log.rightJoy);
          Serial.print(" ");
          Serial.println(log.pressure);
      }

      ControllerComm.write(PROTOCOL_START_BYTE);
      ControllerComm.write((uint8_t*)&log, sizeof(log));
    }
  }

  if(DEBUG_POTS) {
    Serial.print("trim: ");
    Serial.print(auto_values[POT_TRIM]);
    Serial.print(" l1: ");
    Serial.print(auto_values[POT_L1]);
    Serial.print(" l1l2: ");
    Serial.print(auto_values[POT_L1L2]);
    Serial.print(" l2: ");
    Serial.print(auto_values[POT_L2]);
    Serial.print(" l2l3: ");
    Serial.print(auto_values[POT_L2L3]);
    Serial.print(" l3: ");
    Serial.println(auto_values[POT_L3]);
  }
}
