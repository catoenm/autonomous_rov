/*
  Control and setup of all submarine BLDC motors
*/

typedef enum MotorNumbers {
  MOTOR_BACK_RIGHT = 0,
  MOTOR_FRONT_RIGHT,
  MOTOR_BACK_LEFT,
  MOTOR_FRONT_LEFT,
  MOTOR_NUM_MOTORS
} MotorNumbers;

void attach_motors() {
  motors[MOTOR_BACK_RIGHT].attach(ESC1); // BACK_RIGHT
  motors[MOTOR_BACK_LEFT].attach(ESC3); // BACK_LEFT
  motors[MOTOR_FRONT_RIGHT].attach(ESC2); // FRONT_RIGHT
  motors[MOTOR_FRONT_LEFT].attach(ESC4); // FRONT_LEFT
}

void set_throttle_all(double motorThrottle) {
  motorThrottle = map(motorThrottle, 0, 254, MAX_REVERSE, MAX_FORWARD);
  for(auto motor : motors) {
    motor.write(motorThrottle);
  }
}

void set_throttle(int motor_num, double motorThrottle) {
  motorThrottle = map(motorThrottle, -100, 100, MAX_REVERSE, MAX_FORWARD);
  motors[motor_num].write(motorThrottle);
}

void motors_drive(int throttle, int angle) {
  angle = map(angle, 0, 254, -40, 40);
  throttle = map(throttle, 0, 254, -100, 100);
  set_throttle(MOTOR_BACK_RIGHT, throttle + angle);
  set_throttle(MOTOR_BACK_LEFT, (throttle - angle) * -1);
  set_throttle(MOTOR_FRONT_RIGHT, (throttle + angle) * -1);
  set_throttle(MOTOR_FRONT_LEFT, throttle - angle);
  if(DEBUG_MOTORS) {
    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.print("% Angle: ");
    Serial.print(angle);
    Serial.println("%");
  }
}

void motors_back_only(int throttle) {
  throttle = map(throttle, 0, 254, -100, 100);
  set_throttle(MOTOR_BACK_RIGHT, 0);
  set_throttle(MOTOR_BACK_LEFT, 0);
  set_throttle(MOTOR_FRONT_RIGHT, throttle * -1);
  set_throttle(MOTOR_FRONT_LEFT, throttle);
}

void motors_front_only(int throttle) {
  throttle = map(throttle, 0, 254, -100, 100);
  set_throttle(MOTOR_BACK_RIGHT, 0);
  set_throttle(MOTOR_BACK_LEFT, 0);
  set_throttle(MOTOR_FRONT_RIGHT, throttle);
  set_throttle(MOTOR_FRONT_LEFT, throttle);
}

void sweep_motors(int throttle1, int angle1, int throttle2, int angle2) {
  int throttle_diff = throttle2 - throttle1;
  int angle_diff = angle2 - angle1;
  int throttle = throttle1;
  int angle = angle1;
  int d_throttle = throttle_diff/SWEEP_STEPS;
  int d_angle = angle_diff/SWEEP_STEPS;

  for(int i = 0; i < SWEEP_STEPS; i++) {
    if(abs(throttle2 - throttle) > abs(d_throttle)) {
      throttle += d_throttle;
    }
    if(abs(angle2 - angle) > abs(d_angle)) {
      angle += d_angle;
    }
    motors_drive(throttle, angle);
    delay(8);
  }
  comm_flush();
}

void config_auto_param(PotChooser potChooser, int val) {
  if(potChooser == POT_TRIM) {
    val = map(val, 0, 254, -100, 100);
  } else if(potChooser == L1L2) {
    val = map(val, 0, 254, -200, 200);
  } else {
    val = map(val, 0, 254, -1000, 1000);
  }
  auto_values[potChooser] = base_auto_values[potChooser] + val;
}

void calibrate_motors() {
  set_throttle_all(254);
  delay(5000);
  set_throttle_all(0);
  delay(1000);
  set_throttle_all(127.5);
  delay(1000);
}

void electromag_attract() {
  digitalWrite(ELECTROMAG_N, LOW);
}

void electromag_repel() {
  digitalWrite(ELECTROMAG_N, HIGH);
}
