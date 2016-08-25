/*
 * Opens soccer robot main code set,
 * Team circut breaker Dev @RKouchoo
 *
 * First created:  5 / 8 / 2015
 * Latest Update:  25 / 8 / 2016
 *
 * If you are wondering how this code works, send a message to r.kouchoo1@gmail.com
 *
 * anything with a " // " next to it means it has been modified or its a comment!
 *
 *
 *        ===============================================
 *        PLEASE READ THE DOCUMENTATION FOR MORE INFO!!!!
 *        ===============================================
 *
 *
 */


                            //IMPORTS
 //=======================================================================

 #include <Wire.h>
 #include <Arduino.h>
                            //ROBOT MAP
//=======================================================================


  #define CompassAddress 0x1E //compass I2C addresses
  #define CompassMode 0x02
  #define CompassReadMode 0x00
  #define CompassRegister 0x03
  #define CompassByteLength 6

  static int LoopDelay = 5;

  int CompassX; //compass headings
  int CompassY;
  int CompassZ; //current compass oreintation/direction
  int CompassOriginalDirection_Z; //the original direction that the compass starts in. stored at robot init, used to auto align (LATER OK :/ )

  //details about the compass can be found on this data sheet http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf

  const int LASER_OUTPUT_1 = 10;     //laser and led power up pins
  const int LASER_OUTPUT_2 = 11;
  const int LED_PIN = 22;

  int LightsensorValue = 0;      //where the lightsensor input is stored
  int RightLightsensorValue = 0;
  int LeftLightsensorValue = 0;
  const int LightsensorInput_1 = A0; //lightsensor analog read pin
  const int LightsensorInput_2 = A1;

  int COLOR_GREEN_VALUE = 23;     //colour values which need to be changed, depends on feild lighting !
  int COLOR_WHITE_VALUE = 245;

  boolean IsMovingLeft = false;  //true or false statements used for light sensor & movement
  boolean IsMovingRight = false;
  boolean IsMovingBackward = false;
  boolean IsMovingForward = false;

  int DefendCountdown = 100;  //to be continued, look at documentation

  const int MotorMED_POWER = 115;  //can be used for motor setup functions max 255, -- will use when pwm cables are connected
  const int MotorFULL_POWER = 255;
  const int MotorLOW_POWER = 25;
                                  //pwm speed and enable pins for the motor controllers
  const int Motor_EN_S_1_1 = 2;
  const int Motor_EN_S_1_2 = 3;
  const int Motor_EN_S_2_1 = 4;
  const int Motor_EN_S_2_2 = 5;


  const int Motor1A = 38; //motor input pins A is forward, B is reverse
  const int Motor1B = 40;

  const int Motor2A = 42;
  const int Motor2B = 44;

  const int Motor3A = 46;
  const int Motor3B = 48;

  const int Motor4A = 50;
  const int Motor4B = 52;

  const int LED_PWM_MAX_RATE = 255; //lighting stuff
  const int LED_PWM_MIN_RATE = 25;
  long LED_RAND;

  const int SOLENOID_PIN = 17; //future ref for transformer & kicker



//==================================================================

void RobotInitLights() {
  pinMode(LED_PIN, OUTPUT);
  LED_RAND = random(LED_PWM_MIN_RATE, LED_PWM_MAX_RATE);
  digitalWrite(LED_PIN, LED_RAND);
 }

void initlaser() {
 pinMode(LASER_OUTPUT_1, OUTPUT);
 pinMode(LASER_OUTPUT_2, OUTPUT);

  digitalWrite(LASER_OUTPUT_1, HIGH);
 digitalWrite(LASER_OUTPUT_2, HIGH);
 }

//                          SEEKER
//==================================================================

/*
  IRSeeker.ino - A class for the HiTechnic IRSeeker V2 infrared sensor.
  Created by B. Blechschmidt, August 1, 2013.
  Edited by R. Kouchoo - STRICT USE for SPX.
  Clean version was released into the public domain.
*/

struct InfraredResult
{
  byte Direction;
  byte Strength;
};

class InfraredSeeker
{
  public:
    static void Initialize();
    static boolean Test();
    static void ReadACRaw(byte* buffer);
    static void ReadDCRaw(byte* buffer);
    static InfraredResult ReadAC();
    static InfraredResult ReadDC();
    static int DirectionAngle(byte Direction);
  private:
    static InfraredResult PopulateValues(byte* buffer);
    static void ReadValues(byte OffsetAddress, byte* buffer);
    static const int Address = 0x10 / 2; //Divide by two as 8bit-I2C address is provided
};

void InfraredSeeker::Initialize()
{
  Wire.begin();
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(0x00);
  Wire.endTransmission();
  while(Wire.available() > 0)
    Wire.read();
}

boolean InfraredSeeker::Test()
{
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.requestFrom(InfraredSeeker::Address, 16);
  char Manufacturer_Model[16];
  while(Wire.available() < 16);
  for(byte i=0; i < 16; i++)
  {
    Manufacturer_Model[i] = Wire.read();
  }
  while(Wire.available() > 0)
    Wire.read();
  return strncmp(Manufacturer_Model, "HiTechncNewIRDir", 16)==0;
}

void InfraredSeeker::ReadValues(byte OffsetAddress, byte* buffer)
{
  Wire.beginTransmission(InfraredSeeker::Address);
  Wire.write(OffsetAddress);
  Wire.endTransmission();
  Wire.requestFrom(InfraredSeeker::Address, 6);
  while(Wire.available() < 6);
  for(byte i = 0; i < 6; i++)
  {
    buffer[i] = Wire.read();
  }
  while(Wire.available() > 0)
    Wire.read();
}

void InfraredSeeker::ReadACRaw(byte* buffer)
{
  ReadValues(0x49, buffer);
}

void InfraredSeeker::ReadDCRaw(byte* buffer)
{
  ReadValues(0x42, buffer);
}

InfraredResult InfraredSeeker::PopulateValues(byte* buffer)
{
  InfraredResult Data;
  Data.Direction = buffer[0];
  if(buffer[0] != 0)
  {
    if(buffer[0] % 2 == 0)
    {
      Data.Strength = (buffer[buffer[0] / 2] + buffer[buffer[0] / 2 + 1]) / 2;
    }
    else
    {
      Data.Strength = buffer[buffer[0] / 2 + 1];
    }
  }
  else
  {
    Data.Strength = 0;
  }
  return Data;
}

InfraredResult InfraredSeeker::ReadAC()
{
  byte buffer[6];
  ReadACRaw(buffer);
  return PopulateValues(buffer);
}

InfraredResult InfraredSeeker::ReadDC()
{
  byte buffer[6];
  ReadDCRaw(buffer);
  return PopulateValues(buffer);
}

int DirectionAngle(byte Direction)
{
  return Direction * 30 - 150;
}
//============================================================================================

void CompassInit() {

    //initialise compass
    Wire.begin();
    Wire.beginTransmission(CompassAddress); //open comms to the compass
    Wire.write(CompassMode); //select mode register
    Wire.write(CompassReadMode); //continuous measurement mode
    Wire.endTransmission();
    Wire.beginTransmission(CompassAddress);
    Wire.write(CompassRegister); //select register 3, X MSB register, these valuses were found on the data sheet.
    Wire.endTransmission();

    //request data from the comnpass
    Wire.requestFrom(CompassAddress, CompassByteLength);

    //request data and set original direction, will be used for auto correct in the future.
    if(CompassByteLength<=Wire.available()){
      CompassOriginalDirection_Z = Wire.read()<<8; //Z msb
      CompassOriginalDirection_Z |= Wire.read(); //Z lsb
    }


    //Tell the Compass where to begin reading  directional data
    Wire.beginTransmission(CompassAddress);
    Wire.write(CompassRegister); //select register 3, X MSB register
    Wire.endTransmission();

     Wire.requestFrom(CompassAddress, CompassByteLength);

    if(CompassByteLength<=Wire.available()){
      CompassX = Wire.read()<<8; //X msb
      CompassX |= Wire.read(); //X lsb

      CompassZ = Wire.read()<<8; //Z msb
      CompassZ |= Wire.read(); //Z lsb

      CompassY = Wire.read()<<8; //Y msb
      CompassY |= Wire.read(); //Y lsb
    }

}

void LightSensor() {

  //Serial.println("LightSensor value = " + RightLightsensorValue + ", " + LeftLightsensorValue);

  //LIGHT & Movement

  RightLightsensorValue = analogRead(LightsensorInput_1);
  LeftLightsensorValue = analogRead(LightsensorInput_2);
 //cond combo 1
  if (LightsensorValue == COLOR_WHITE_VALUE)
  {
   if (IsMovingLeft = true)
     {
     MoveFORWARDRIGHT();
     Serial.println("boolean - IsMovingLeft - MoveFORWARDRIGHT()");
     }
  }
 //cond combo 2
  if (LightsensorValue == COLOR_WHITE_VALUE)
  {
    if (IsMovingRight = true)
    {
      MoveFORWARDLEFT();
      Serial.println("boolean - IsMovingRight - MoveFORWARDLEFT()");
    }
  }
 //cond combo 3
  if (LightsensorValue == COLOR_WHITE_VALUE)
  {
    if (IsMovingForward = true)
    {
      MoveBACKWARD();
      Serial.println("boolean - IsMovingForward - MoveBACKWARD()");
    }
  }

}

//============================================================================================
//                                      Motor Functions

void Motor1FWD() {
  digitalWrite(Motor1A, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);
}
void Motor1BACKWARD() {
  digitalWrite(Motor1B, HIGH);
  delay(50);
  digitalWrite(Motor1B, LOW);

}

void Motor2FWD() {
  digitalWrite(Motor2A, HIGH);
  delay(50);
  digitalWrite(Motor2A, LOW);
}
void Motor2BACKWARD() {
  digitalWrite(Motor2B, HIGH);
  delay(50);
  digitalWrite(Motor2B, LOW);
}

void Motor3FWD() {
  digitalWrite(Motor3A, HIGH);
  delay(50);
  digitalWrite(Motor3A, LOW);
}
void Motor3BACKWARD() {
  digitalWrite(Motor3B, HIGH);
  delay(50);
  digitalWrite(Motor3B, LOW);
}

void Motor4FWD() {
  digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor4A, LOW);
}
void Motor4BACKWARD() {
  digitalWrite(Motor4B, HIGH);
  delay(50);
  digitalWrite(Motor4B, LOW);
}

//==================================================================

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
void MoveFORWARDLEFT() {
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor4B, HIGH);
  delay(50);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor4B, LOW);
}
void MoveBACKWARDLEFT() {
  digitalWrite(Motor2B, HIGH);
  digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor4A, LOW);
}
void MoveSIDEWAYSRIGHT() {
  digitalWrite(Motor2B, HIGH);
  digitalWrite(Motor3A, HIGH);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor4B, HIGH);
  delay(50);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor3A, LOW);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor4B, LOW);
  }
void MoveSIDEWAYSLEFT() {
  digitalWrite(Motor2B, HIGH);
  digitalWrite(Motor4A, HIGH);
  digitalWrite(Motor1B, HIGH);
  digitalWrite(Motor3A, HIGH);
  delay(50);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor4A, LOW);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor3A, LOW);

}
void TurnLEFT() {
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor4B, HIGH);
  digitalWrite(Motor2B, HIGH);//
  digitalWrite(Motor3B, HIGH);
  delay(50);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor4B, LOW);
  digitalWrite(Motor2B, LOW);
  digitalWrite(Motor3B, LOW);//
}
void TurnRIGHT() {
  digitalWrite(Motor2A, HIGH);
  digitalWrite(Motor3B, HIGH);
  digitalWrite(Motor1A, HIGH);
  digitalWrite(Motor4A, HIGH);
  delay(50);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor3B, LOW);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor4A, LOW);
}
//=============================================================================================
void SetMotorSpeedLOW() {
  analogWrite(Motor_EN_S_1_1, MotorLOW_POWER);
  analogWrite(Motor_EN_S_1_2, MotorLOW_POWER);
  analogWrite(Motor_EN_S_2_1, MotorLOW_POWER);
  analogWrite(Motor_EN_S_2_2, MotorLOW_POWER);

  delay(50);

  analogWrite(Motor_EN_S_1_1, LOW);
  analogWrite(Motor_EN_S_1_2, LOW);
  analogWrite(Motor_EN_S_2_1, LOW);
  analogWrite(Motor_EN_S_2_2, LOW);
}

void SetMotorSpeedMED() {
  analogWrite(Motor_EN_S_1_1, MotorMED_POWER);
  analogWrite(Motor_EN_S_1_2, MotorMED_POWER);
  analogWrite(Motor_EN_S_2_1, MotorMED_POWER);
  analogWrite(Motor_EN_S_2_2, MotorMED_POWER);

  delay(50);

  analogWrite(Motor_EN_S_1_1, LOW);
  analogWrite(Motor_EN_S_1_2, LOW);
  analogWrite(Motor_EN_S_2_1, LOW);
  analogWrite(Motor_EN_S_2_2, LOW);
}

void SetMotorSpeedHIGH() {
  analogWrite(Motor_EN_S_1_1, MotorFULL_POWER);
  analogWrite(Motor_EN_S_1_2, MotorFULL_POWER);
  analogWrite(Motor_EN_S_2_1, MotorFULL_POWER);
  analogWrite(Motor_EN_S_2_2, MotorFULL_POWER);

  delay(50);

  analogWrite(Motor_EN_S_1_1, LOW);
  analogWrite(Motor_EN_S_1_2, LOW);
  analogWrite(Motor_EN_S_2_1, LOW);
  analogWrite(Motor_EN_S_2_2, LOW);
}

void SetMotorSpeedOFF(){
  analogWrite(Motor_EN_S_1_1, LOW);
  analogWrite(Motor_EN_S_1_2, LOW);
  analogWrite(Motor_EN_S_2_1, LOW);
  analogWrite(Motor_EN_S_2_2, LOW);
}


 //==========================================================================


void setup() {
  //initialise serial broadcast
  Serial.begin(9600);

  //initialise classes
  RobotInitLights();
  initlaser();
  InfraredSeeker::Initialize();

//initialise motor outputs
  pinMode(Motor_EN_S_1_1, OUTPUT);
  pinMode(Motor_EN_S_1_2, OUTPUT);
  pinMode(Motor_EN_S_2_1, OUTPUT);
  pinMode(Motor_EN_S_2_2, OUTPUT);

  pinMode(Motor1A, OUTPUT);
  pinMode(Motor1B, OUTPUT);
  pinMode(Motor2A, OUTPUT);
  pinMode(Motor2B, OUTPUT);
  pinMode(Motor3A, OUTPUT);
  pinMode(Motor3B, OUTPUT);
  pinMode(Motor4A, OUTPUT);
  pinMode(Motor4B, OUTPUT);

  Serial.println("HiTechnic IRSeeker V2 Initialised, robot has been initialised, loop program will now commence !");
  Serial.println("QUICK DEBUG: Compass start rotation = " + CompassOriginalDirection_Z);
}

//==========================================================================
 //                            I===========I
 //                            I-MAIN LOOP-I
 //                            I===========I
//==========================================================================

  void loop() {

  InfraredResult InfraredBall = InfraredSeeker::ReadAC();
  byte Direction = InfraredBall.Direction;
  SetMotorSpeedOFF();

//===========================================================================
   //conditionals for LightSensor Movement var
   boolean IsMovingLeft = false;
   boolean IsMovingRight = false;
   boolean IsMovingBackward = false;
   boolean IsMovingForward = false;
//==========================================================================

  if (InfraredBall.Direction == 1)
  {
  SetMotorSpeedMED();
  TurnRIGHT();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 1, TurnRIGHT()");
  boolean IsMovingRight = true;
  }

  if (InfraredBall.Direction == 2)
  {
  SetMotorSpeedLOW();
  TurnRIGHT();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 2, TurnRIGHT()");
  boolean IsMovingRight = true;
  }

  if (InfraredBall.Direction == 3)
  {
  SetMotorSpeedLOW();
  TurnRIGHT();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 3, MoveFORWARDRIGHT()");
  boolean IsMovingRight = true;
  }

  if (InfraredBall.Direction == 4)
  {
  SetMotorSpeedMED();
  MoveFORWARD();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 4, MoveFORWARD()");
  boolean IsMovingForward = true;
  }

  if (InfraredBall.Direction == 5)
  {
  SetMotorSpeedHIGH();
  MoveFORWARD();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 5, MoveFORWARD()");
  boolean IsMovingForward = true;
  }

  if (InfraredBall.Direction == 6)
  {
  SetMotorSpeedMED();
  MoveFORWARD();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 6, MoveFORWARD()");
  boolean IsMovingForward = true;
  }

  if (InfraredBall.Direction == 7)
  {
  SetMotorSpeedLOW();
  TurnLEFT();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 7, MoveFORWARDLEFT()");
  boolean IsMovingLeft = true;
  }

  if (InfraredBall.Direction == 8)
  {
  SetMotorSpeedLOW();
  TurnLEFT();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 8, TurnLEFT()");
  boolean IsMovingLeft = true;
  }

  if (InfraredBall.Direction == 9)
  {
  SetMotorSpeedMED();
  TurnLEFT();
  SetMotorSpeedOFF();
  Serial.println("InfraredBall.Direction = 9, TurnLEFT()");
  boolean IsMovingLeft = true;
  }

//==================================================================

  if (InfraredBall.Direction == 0)
  {
  Serial.println("Wh-Wo-Jo-La Do ? - TurnRIGHT()");
  SetMotorSpeedLOW();
  TurnRIGHT();
  SetMotorSpeedOFF();
  boolean IsMovingRight = true;
  }
  delay(LoopDelay);
}
