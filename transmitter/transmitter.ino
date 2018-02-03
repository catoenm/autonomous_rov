#define FORWARD_DOWN 0x0
#define FORWARD_UP 0x1

#define ELECTROMAGNET_BUTTON (PSB_L1)
#define QUICK_DIVE_BUTTON (PSB_CROSS)
#define AUTO_BUTTON (PSB_PAD_UP)
#define JUST_BACK_BUTTON (PSB_SQUARE)
#define RESTART_AUTO (PSB_PAD_LEFT)
#define UPDATE_POT (PSB_PAD_DOWN)
#define PROTOCOL_MID_VAL (127)
#define DEAD_ZONE 10

#define PROTOCOL_START_BYTE (0xFF)
#define PROTOCOL_MIN_VAL (0x00)
#define PROTOCOL_MAX_VAL (0xFE) // 254
#define PROTOCOL_MID_VAL (127)

// See http://www.billporter.info/2010/06/05/playstation-2-controller-arduino-library-v1-0/ for wire designations
#define PS2_DAT (6)
#define PS2_CMD (5)
#define PS2_SEL (7) // Probably the "attention" wire
#define PS2_CLK (4)
#define PS2_PRESSURES (false)
#define PS2_RUMBLE (false)

#define DEBUG_INPUT (true)
#define DEBUG_TRANSMITTER (false)
#define DEBUG_RAW_INPUT (false)
#define DEBUG_POTS (false)
#define LOG_ACTIONS (false)

#define RIGHT_JOY (A0)
#define LEFT_JOY (A1)
#define POT_TRIM (A11)
#define POT_L1 (A8)
#define POT_L1L2 (A9)
#define POT_L2 (A10)
#define LED_TRIM (A12)
#define LED_L1 (A15)
#define LED_L1L2 (A14)
#define LED_L2 (A13)

#include "PS2X_lib.h"
#include "crc32b.h"

enum Autonomous {
  WAITING,
  PRESSED,
  RELEASED,
  PRESSED_AGAIN
};

typedef struct Log {
    uint32_t time;
    uint8_t buttonEnum;
    uint8_t leftJoy;
    uint8_t rightJoy;
    uint32_t pressure;
} SubLog;

PS2X ps2x; // create PS2 Controller Class
Autonomous autonomous = WAITING;
uint8_t potChooser = 5;
uint8_t pots[4] = {POT_TRIM, POT_L1, POT_L1L2, POT_L2};

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PS2_PRESSURES, PS2_RUMBLE);

  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (PS2_PRESSURES)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (PS2_RUMBLE)
      Serial.println("true)");
    else
      Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  } else if(error == 1) {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  } else if(error == 2) {
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  } else if(error == 3) {
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }

  int type = ps2x.readType();
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }

   pinMode(INPUT, RIGHT_JOY);
   pinMode(INPUT, LEFT_JOY);
   pinMode(INPUT, POT_TRIM);
   pinMode(INPUT, POT_L1);
   pinMode(INPUT, POT_L1L2);
   pinMode(INPUT, POT_L1);
   pinMode(OUTPUT, LED_TRIM);
   pinMode(OUTPUT, LED_L1);
   pinMode(OUTPUT, LED_L1L2);
   pinMode(OUTPUT, LED_L2);

}

void loop() {
  ps2x.read_gamepad(); //read controller and set large motor to spin at 'vibrate' speed


  uint16_t joy_right = map(analogRead(RIGHT_JOY), 0, 700, 254, 0);
  uint8_t joy_left = map(analogRead(LEFT_JOY), 0, 700, 254, 0);
  uint8_t state = 0;
  switch(autonomous) {
    case WAITING:
      if(ps2x.Button(ELECTROMAGNET_BUTTON)) {
        state = 1;
      } else if(ps2x.Button(QUICK_DIVE_BUTTON)) {
        state = 2;
      } else if(ps2x.Button(AUTO_BUTTON)) {
        autonomous = PRESSED;
        state = 3;
      } else if(ps2x.Button(JUST_BACK_BUTTON)) {
        state = 4;
      } else if(ps2x.Button(RESTART_AUTO)) {
        state = 5;
      } else if(ps2x.ButtonReleased(PSB_PAD_DOWN)) {
        state = 6;
        joy_left = potChooser;
        joy_right = map(analogRead(pots[potChooser]), 0, 1024, 0, 254);
        Serial.println("Sent Val");
        potChooser = (potChooser + 1) % 4;
      }
      break;
    case PRESSED:
      if(!ps2x.Button(AUTO_BUTTON)) {
        autonomous = RELEASED;
      }
      state = 3;
      break;
    case RELEASED:
      if(ps2x.Button(AUTO_BUTTON)) {
        autonomous = PRESSED_AGAIN;
      }
      state = 3;
      break;
    case PRESSED_AGAIN:
      if(!ps2x.Button(AUTO_BUTTON)) {
        autonomous = WAITING;
      }
      break;
  }

  if(state != 6) {
    if(abs((int)joy_right - PROTOCOL_MID_VAL) < DEAD_ZONE) {
      joy_right = PROTOCOL_MID_VAL;
    } else if(joy_right > 245) {
      joy_right = PROTOCOL_MAX_VAL;
    } else if(joy_right < 20) {
      joy_right = PROTOCOL_MIN_VAL;
    }
    if(abs((int)joy_left - PROTOCOL_MID_VAL) < DEAD_ZONE) {
      joy_left = PROTOCOL_MID_VAL;
    } else if(joy_left > 245) {
      joy_left = PROTOCOL_MAX_VAL;
    } else if(joy_left < 20) {
      joy_left = PROTOCOL_MIN_VAL;
    }
  }

  uint8_t message[3] = {state, joy_left, joy_right};

  uint32_t crc = crc32b(message, sizeof(message));
  uint8_t crc_bytes[sizeof(crc)] = {
    (crc >> 24) & 0xFF,
    (crc >> 16) & 0xFF,
    (crc >> 8) & 0xFF,
    crc & 0xFF
  };

  if (DEBUG_INPUT) {
    Serial.print("Joy Right: ");
    Serial.print(joy_right);
    Serial.print(" Joy Left: ");
    Serial.print(joy_left);
    Serial.print("  Button State: ");
    Serial.println(state);
  }

  if (DEBUG_RAW_INPUT) {
    Serial.print("Joy Right: ");
    Serial.print(ps2x.Analog(PSS_RX));
    Serial.print(" Joy Left: ");
    Serial.print(ps2x.Analog(PSS_LY));
    Serial.print("  Button State: ");
    Serial.println(state);
  }

  if (DEBUG_TRANSMITTER) {
    Serial.println("====================");
    Serial.print(message[0], HEX); Serial.print(" ");
    Serial.print(message[1], HEX); Serial.print(" ");
    Serial.print(message[2], HEX); Serial.println(" ");
    Serial.println();
    Serial.println(crc, HEX);
    Serial.println();
    Serial.print(crc_bytes[0], HEX); Serial.print(" ");
    Serial.print(crc_bytes[1], HEX); Serial.print(" ");
    Serial.print(crc_bytes[2], HEX); Serial.print(" ");
    Serial.print(crc_bytes[3], HEX); Serial.println(" ");
    Serial.println("************************");
  }


  if(LOG_ACTIONS) {
    SubLog log;
    if(get_message_if_available(&log)) {
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
  }

  if(analogRead(POT_TRIM) > 512) {
    analogWrite(LED_TRIM, 255);
  } else {
    analogWrite(LED_TRIM, 0);
  }
  if(analogRead(POT_L1) > 512) {
    analogWrite(LED_L1, 255);
  } else {
    analogWrite(LED_L1, 0);
  }
  if(analogRead(POT_L1L2) > 512) {
    analogWrite(LED_L1L2, 255);
  } else {
    analogWrite(LED_L1L2, 0);
  }
  if(analogRead(POT_L2) > 512) {
    analogWrite(LED_L2, 255);
  } else {
    analogWrite(LED_L2, 0);
  }

  if(DEBUG_POTS) {
    Serial.print("Trim: ");
    Serial.print(analogRead(RIGHT_JOY));
    Serial.print(" L1: ");
    Serial.print(analogRead(LEFT_JOY));
    Serial.print(" L1L2: ");
    Serial.print(analogRead(POT_L1L2));
    Serial.print(" L2: ");
    Serial.println(analogRead(POT_L2));
  }

  Serial1.write(PROTOCOL_START_BYTE);
  Serial1.write(message, sizeof(message));
  Serial1.write(crc_bytes, sizeof(crc_bytes));
  delay(25);
}

bool get_message_if_available(SubLog* log) {
  if (Serial1.available() && (Serial1.read() == PROTOCOL_START_BYTE)) {
    return Serial1.readBytes((uint8_t *)log, sizeof(SubLog)) == sizeof(SubLog);
  }

  return false;
}
