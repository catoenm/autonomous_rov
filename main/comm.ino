#include "config.h"
#include "crc32b.h"

// converting to a uint32_t is messy due to 16-bit max upconversion on shifting.
#define CHECK_BYTES_MATCH(byte_buffer, uint_var) ((byte_buffer[0] == ((uint_var >> 24) & 0xFF)) && \
                          (byte_buffer[1] == ((uint_var >> 16) & 0xFF)) && \
                          (byte_buffer[2] == ((uint_var >> 8) & 0xFF)) && \
                          (byte_buffer[3] == (uint_var & 0xFF)))

bool read_packet(uint8_t *buffer, size_t num_bytes) {
  return (ControllerComm.available() && (ControllerComm.read() == PROTOCOL_START_BYTE)) &&
         (ControllerComm.readBytes(buffer, num_bytes) == num_bytes);
}

bool get_message_if_available(JoystickState *state) {
  if (!read_packet((uint8_t *)state, sizeof(JoystickState))) {
      return false;
  }
  uint32_t crc = crc32b((uint8_t *)state, JOYSTICK_STATE_DATA_SIZE);

  if (COMM_DEBUG_CRC) {
    uint32_t crc2 = crc32b((uint8_t *)state, JOYSTICK_STATE_DATA_SIZE);
    uint8_t *state_bytes = (uint8_t *)state;
    Serial.println("===========================");
    Serial.print("Packet (state bytes): ");
    Serial.print(state_bytes[0], HEX); Serial.print(" ");
    Serial.print(state_bytes[1], HEX); Serial.print(" ");
    Serial.print(state_bytes[2], HEX); Serial.println(" ");
    Serial.print("CRC: ");
    Serial.print(state_bytes[3], HEX); Serial.print(" ");
    Serial.print(state_bytes[4], HEX); Serial.print(" ");
    Serial.print(state_bytes[5], HEX); Serial.print(" ");
    Serial.print(state_bytes[6], HEX); Serial.print(" ");
    Serial.print(" == ");Serial.print(crc, HEX); Serial.print(" == "); Serial.println(crc2, HEX);
    Serial.println(JOYSTICK_STATE_DATA_SIZE);

    Serial.println("**************************");
  }
  return CHECK_BYTES_MATCH(state->crc, crc);
}

void comm_flush() {
  while(ControllerComm.available() > 0)
     ControllerComm.read();
}
