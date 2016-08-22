#include <Arduino.h>

 void setup() {

pinMode(13, OUTPUT);

 }

void loop() {

  anaglogWrite(13, HIGH);
  delay(45);
  analogWrite(13, LOW);

}
