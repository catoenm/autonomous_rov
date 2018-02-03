
//Cyclone Robotics Communication File

void updateComm(){
  if (rf69.available())
  {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len))
    {
      throttleMsg = (buf[0]<<8|buf[1]);
      rollMsg = (buf[2]<<8|buf[3]);
      pitchMsg = (buf[4]<<8|buf[5]);
      yawMsg = (buf[6]<<8|buf[7]);
      
      throttle = throttleMsg/10.23; // Map to 0 - 100%
      rollSetpoint = ((int)rollMsg-512)/68.34; // Map to +/- 7.5deg
      pitchSetpoint = ((int)pitchMsg-512)/68.34; // Map to +/- 7.5deg
      yawSetpoint = ((int)yawMsg-512)/2.84; // Map to +/- 180deg
      tempPIDConst = yawMsg*(0.0001);
      
      // Print values read
      /*
      Serial.print("T: ");
      Serial.print(throttle);
      Serial.print(" R: ");
      Serial.print(rollSetpoint);
      Serial.print(" P: ");
      Serial.print(pitchSetpoint);
      Serial.print(" Y: ");
      Serial.println(yawSetpoint);
      */
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}

void initComm(){
  
  throttleMsg = 0;
  rollMsg = 0;
  pitchMsg = 0;
  yawMsg = 0;

  if (!rf69.init())
    Serial.println("init failed");
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
#if 0
  // For compat with RFM69 Struct_send
  rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
  rf69.setPreambleLength(3);
  uint8_t syncwords[] = { 0x2d, 0x64 };
  rf69.setSyncWords(syncwords, sizeof(syncwords));
  rf69.setEncryptionKey((uint8_t*)"thisIsEncryptKey");
#endif
}


