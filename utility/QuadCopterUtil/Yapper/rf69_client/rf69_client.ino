#include <SPI.h>
#include <RH_RF69.h>

#define VOLTAGE_MONITOR A0
#define THROTTLE A1
#define BUTTON A3
#define ROLL A4
#define PITCH A5
#define YAW A2

int prevThrottle = 0;
int prevRoll = 0;
int prevPitch = 0;
int prevYaw = 0;

// Singleton instance of the radio driver
RH_RF69 rf69(4);
//RH_RF69 rf69(15, 16); // For RF69 on PJRC breakout board with Teensy 3.1

void setup() 
{
  pinMode(BUTTON, INPUT);
  Serial.begin(9600);
  if (!rf69.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  // No encryption
  if (!rf69.setFrequency(433.0))
    Serial.println("setFrequency failed");

  // If you are using a high power RF69, you *must* set a Tx power in the
  // range 14 to 20 like this:
  rf69.setTxPower(20);

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
}


void loop()
{
  uint8_t data[8];
  bool sendData = true;
  int throttleValue = analogRead(THROTTLE);
  int rollValue = analogRead(ROLL);
  int pitchValue = analogRead(PITCH);
  int yawValue = analogRead(YAW);
  int buttonValue = digitalRead(BUTTON);
  
  // If values have changed, send new values
  if (abs(throttleValue - prevThrottle) > 3 || abs(rollValue - prevRoll) > 3
        || abs(pitchValue - prevPitch) > 3 || abs(yawValue - prevYaw) > 3 ) {
    
    Serial.println("Sending message to Quad:");
    Serial.print("T: ");
    Serial.print(throttleValue/10.23);
    Serial.print(" R: ");
    Serial.print(((int)rollValue-512)/68.34);
    Serial.print(" P: ");
    Serial.print(((int)pitchValue-512)/68.34);
    Serial.print(" Y: ");
    Serial.print(((int)yawValue-512)/2.84);
    Serial.print(" B: ");
    Serial.println(buttonValue);
    
    Serial.print("PID Value: ");
    Serial.print((double)yawValue/1.0);
    
    data[0] = (uint8_t)(throttleValue>>8);
    data[1] = (uint8_t)(throttleValue%(1<<8));
    data[2] = (uint8_t)(rollValue>>8);
    data[3] = (uint8_t)(rollValue%(1<<8));
    data[4] = (uint8_t)(pitchValue>>8);
    data[5] = (uint8_t)(pitchValue%(1<<8));
    data[6] = (uint8_t)(yawValue>>8);
    data[7] = (uint8_t)(yawValue%(1<<8));
    
    rf69.send(data, sizeof(data));
    
    rf69.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.waitAvailableTimeout(250))
    { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len))
      {
        Serial.print("got reply: ");
        Serial.println((char*)buf);
      }
      else
      {
        Serial.println("recv failed");
      }
    }
    else
    {
      //Serial.println("No reply, is rf69_server running?");
    }
    
    prevThrottle = throttleValue;
    prevRoll = rollValue;
    prevPitch = pitchValue;
    prevYaw = yawValue;
  }
}

