
//Cyclone Robotics Tone File

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(RED, HIGH);
    delayMicroseconds(tone);
    digitalWrite(RED, LOW);
    delayMicroseconds(tone);
  }
}
