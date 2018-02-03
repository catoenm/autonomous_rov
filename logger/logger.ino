
#define PROTOCOL_START_BYTE (0xFF)

typedef struct Log {
    uint32_t time;
    uint8_t buttonEnum;
    uint8_t leftJoy;
    uint8_t rightJoy;
    uint32_t pressure;
} SubLog;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  SubLog log;
  if(get_message_if_available(&log)) {
    Serial.print("Time: ");
    Serial.print(log.time);
    Serial.print(" Button: ");
    Serial.print(log.buttonEnum);
    Serial.print(" LeftJoy: ");
    Serial.print(log.leftJoy);
    Serial.print(" RightJoy: ");
    Serial.print(log.rightJoy);
    Serial.print(" Pressure: ");
    Serial.println(log.pressure);
    return true;
  }
}

bool get_message_if_available(SubLog* log) {
  if (Serial1.available() && (Serial1.read() == PROTOCOL_START_BYTE)) {
    return Serial1.readBytes((uint8_t *)log, sizeof(SubLog)) == sizeof(SubLog);
  }

  return false;
}
