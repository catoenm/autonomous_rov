
//Cyclone Robotics PID Control File

void initPID(){
  rollPid.SetMode(AUTOMATIC);
  pitchPid.SetMode(AUTOMATIC);
  yawPid.SetMode(AUTOMATIC);
}

void updatePID(){
  if(motorState == spinning){
    // Setpoint override
    /*
    rollSetpoint = 0;
    pitchSetpoint = 0;
    yawSetpoint = 0;
    */
    
    // Track time for integration
    deltaT = millis() - prevTime;
    prevTime = millis();
    
//    rollPid.Compute();
//    pitchPid.Compute();
//    yawPid.Compute();

    double rollErr = rollSetpoint - rollInput;
    double pitchErr = pitchSetpoint - pitchInput;
    double yawErr = yawSetpoint - yawInput;

//    if (rollIntErr < -10/ROLL_I)
//      rollIntErr = -10/ROLL_I;
//    if (rollIntErr >  10/ROLL_I)
//      rollIntErr =  10/ROLL_I;
//      
//    if (pitchIntErr < -10/PITCH_I)
//      pitchIntErr = -10/PITCH_I;
//    if (pitchIntErr > 10/PITCH_I)
//      pitchIntErr =  10/PITCH_I;

    rollOutput = rollErr * ROLL_P + rollIntErr * ROLL_I - dRollInput * ROLL_D;
    pitchOutput = pitchErr * PITCH_P + pitchIntErr * PITCH_I - dPitchInput * PITCH_D;
    //yawOutput = yawErr * YAW_P + yawIntErr * YAW_I;

    // Print PID Values
    /*
    if (printOrientation++ > 20) {
      printOrientation = 0;
      Serial.print(tempPIDConst*100.0);
      Serial.print(F(" "));
      Serial.print(rollErr * ROLL_P);
      Serial.print(F(" "));
      Serial.print(rollIntErr * ROLL_I);
      Serial.print(F(" "));
      Serial.print(dRollInput * ROLL_D);
      Serial.println(F(""));
    }
    */
     
    //rollPrevErr = rollErr;
    //pitchPrevErr = pitchErr;
    //yawPrevErr = yawErr;
            
    if (throttle == 0){
      motor0Out = 0;
      motor1Out = 0;
      motor2Out = 0;
      motor3Out = 0;
    }
    
    else {
      // Integrate Error
      rollIntErr += rollErr * deltaT;
      pitchIntErr += pitchErr * deltaT;
      //yawIntErr += yawErr * deltaT;

      // Set outputs 
      motor0Out = throttle - rollOutput - pitchOutput + yawOutput;
      motor1Out = throttle + rollOutput - pitchOutput - yawOutput;
      motor2Out = throttle + rollOutput + pitchOutput + yawOutput;
      motor3Out = throttle - rollOutput + pitchOutput - yawOutput;
    }
  }
}
