void MoveFORWARD() {
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor3A, HIGH);
  digitalWrite(Motor4B, HIGH);
                                // digitalWrite(Motor3A, HIGH);
                                // digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);   // digitalWrite(Motor3A, LOW);
  digitalWrite(Motor2A, LOW);   // digitalWrite(Motor4A, LOW);
  digitalWrite(Motor3A, LOW);
  digitalWrite(Motor4B, LOW);
}
void MoveBACKWARD() {
  digitalWrite(Motor1B, HIGH);
  digitalWrite(Motor2B, HIGH);
  digitalWrite(Motor3B, HIGH);
  digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor3B, LOW);
  digitalWrite(Motor4A, LOW);
}
void MoveBACKWARDRIGHT() {
  digitalWrite(Motor1B, HIGH);
  digitalWrite(Motor3B, HIGH);
  delay(50);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor3B, LOW);
}
void MoveFORWARDRIGHT() {
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor3A, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor3A, LOW);
}
void MoveFORWARDLEFT() {//
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor3A, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor3A, LOW);
}
void MoveBACKWARDLEFT() {//
  digitalWrite(Motor2B, HIGH);
  digitalWrite(Motor4B, HIGH);
  delay(50);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor4B, LOW);
}
void MoveSIDEWAYSRIGHT() {//
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor4A, LOW);
  }
void MoveSIDEWAYSLEFT() {//
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor3A, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor3A, LOW);

}
void TurnLEFT() {//
  digitalWrite(Motor1B, HIGH);
  digitalWrite(Motor4B, HIGH);
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor3A, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor4B, LOW);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor3B, LOW);
}
void TurnRIGHT() {//
  digitalWrite(Motor2B, HIGH);
  digitalWrite(Motor3B, HIGH);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor3B, LOW);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor4A, LOW);
}
