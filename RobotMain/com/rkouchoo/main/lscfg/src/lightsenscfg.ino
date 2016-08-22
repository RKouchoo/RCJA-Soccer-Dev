#include <Arduino.h>

int sensorPin = A0; // select the input pin for ldr
int sensorValue = 0; // variable to store the value coming from the sensor
int buttonIn = 6;
int buttonOut = 5;
int laser = 10;

void setup() {
  pinMode(buttonIn, INPUT);
    pinMode(buttonOut, OUTPUT);
    pinMode(laser, OUTPUT);
Serial.begin(9600); //sets serial port for communication
}
void loop() {
  digitalWrite(buttonOut, HIGH);
  digitalWrite(laser, HIGH);

sensorValue = analogRead(sensorPin); // read the value from the sensor
Serial.println(sensorValue); //prints the values coming from the sensor on the screen

//delay(1000);
}
