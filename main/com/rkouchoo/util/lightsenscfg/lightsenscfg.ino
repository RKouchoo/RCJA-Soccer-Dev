int sensorPin = A0; // select the input pin for ldr
int sensorValue = 0; // variable to store the value coming from the sensor
int buttonIn = 6;
int buttonOut = 5;

void setup() {
  pinMode(buttonIn, INPUT);
    pinMode(buttonOut, OUTPUT);
Serial.begin(9600); //sets serial port for communication
}
void loop() {
  digitalWrite(buttonOut, HIGH);
  
sensorValue = analogRed(sensorPin); // read the value from the sensor
Serial.println(sensorValue); //prints the values coming from the sensor on the screen

if (buttonIn = HIGH) 
{
  //grab sensor values and store it somehow to the arduino...
}

//delay(1000);
}
