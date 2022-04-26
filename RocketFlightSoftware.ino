//Servo control inclusion
#include <Servo.h>

//Comunication protocol inclusion
//#include "Wire.h" //Inclusion of wire.h is unnececarry because it's already included in MPU9250 library
#include <SPI.h>
#include <SoftwareSerial.h>

//IMU sensor inclusion
#include "quaternionFilters.h"
#include "MPU9250.h"

//SD card library inclusion
#include <SD.h>

//Macros for pins
#define LED_R 23
#define LED_G 22
#define LED_B 19

#define PYRO_1 3
#define PYRO_2 9
#define PYRO_3 21
#define PYRO_4 16

#define CS_FLASH 39
#define CS_IMU 10
#define CS_BME 38
#define CS_BREAKOUT 0

#define GPS_RX 29
#define GPS_TX 28

#define BL_RX 35
#define BL_TX 34

#define SERVO_X 36
#define SERVO_Z 33

//definiton of flight states
#define GroundIdle 1;
#define Countdown 2;
#define Launch 3;
#define Flight 4;
#define Return 5;
#define Landed 6;
#define LaunchAbort 7;
#define FlightAbort 8;

//FlightState global int
int flightState = GroundIdle;

//Base angle for servo
int targetAngleX = 90;
int targetAngleZ = 90;

//PID setup for servos
float ServoP = 10f, ServoI = 0.5f, ServoD = 0.5f;

float ServoXError, ServoZError;
float ServoXPID, ServoZPID;

//Time references for countdown
int countdownTimer = 120; //in seconds
int countdownEndTime;
int countdownTime;
int countdownHoldTime;
bool countdownPause = false;

//Macros and object for IMU I2C bus if external sensor is used, unactive when using onboard sensor!
#ifdef IMUI2C
  #define I2Cclock 400000
  #define I2Cport Wire
  #define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 //I2C addres AD1 can also be used if needed

  //inicialization of IMU I2C object
  MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
#else
//Macros and object for IMU SPI bus if onboard sensor is used
  //***Block for SPI IMU***
#endif

//Instances of servo objects
Servo servoX;
Servo servoZ;

//Instances of object using UART communication
SoftwareSerial bluetooth(BL_TX, BL_RX);
SoftwareSerial gps(GPS_TX, GPS_RX);


void setup() {
  //pin I/O setups
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(PYRO_1, OUTPUT);
  pinMode(PYRO_2, OUTPUT);
  pinMode(PYRO_3, OUTPUT);
  pinMode(PYRO_4, OUTPUT);

  //servo setups
  servoX.attach(SERVO_X); 
  servoZ.attach(SERVO_Z); 

  //If using external IMU sensor inicilize I2C library
  #ifdef IMUI2C
    Wire.begin();

    //Read the WHO_AM_I register, this is a good test of communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);
  #else
  //If using onboard IMU sensor inicilize SPI library
    //TODO: ***Block for SPI IMU***
  #endif

  Serial.begin(9600);
  bluetooth.begin();
  gps.begin();

  //Set servos at default angle
  servoX.write(targetAngleX);
  servoZ.write(targetAngleZ);
}

void loop()
  //Check for current state: GroundIdle, Countdown, Launch, Flight, Return, Landed, LaunchAbort, FlightAbort
  switch(flightState)
  {
    case 1: //Ground Idle - sensor logging directly to SD, wating for launch signal
      GroundIdleState();
      break;

    case 2: //Countdown - Countdown, SD card logging suspended, using FLASH instead
      CountdownState();
      break;

    case 3: //Launch - first few seconds of the flight checking for flight nominality, if launch is not detected, switch to LaunchAbort
      LaunchState();
      break;

    case 4: //Flight - stabilization online, complete loging, chechks for flight nominality or return state conditions, if flight is not nominal, switch to FlightAbort
      FlightState();
      break;

    case 5: //Return - MECO, deploy chutes at safe atitude, wait for landing
      ReturnState();
      break;

    case 6: //Landed - landing detected, transfer data from flash to SD
      LandedState();
      break;

    case 7: //LaunchAbort - error at countdown -> stop countdown, transfer data from flash to SD, add error log to SD
      LaunchAbortState();
      break;

    case 8: //FlightAbort - jiggle engine vector to bleed as much power as possible, extended logging to flash, special chute conditions, wait for landing
      FlightAbortState();
      break;  
  }
 
  //Pasive loops - IMU reading, reference reading and logging from GPS, Barometer and IMU
  #ifdef IMUI2C
    readIMUI2C();
  #else
    readIMUSPI();
  #endif
  
  //Servos PID
  ServoXError = targetAngleX - SERVO_X.read();
  ServoZError = targetAngleZ - SERVO_Z.read();

  targetAngleX = ServoXError;
  //Set servos ***I dont know where I should put it yet***
  writeServos(targetAngleX, targetAngleZ);
}

//Functions for each state
void GroundIdleState()
{
  //read bluetooth for launch signal
  byte DATA;
  if(bluetooth.available() > 0)
  {
    DATA = bluetooth.read();

    switch (DATA) 
    {
      case '1': //Start countdown
        flightState = Countdown;
        break;
    }
  
  }
}

void CountdownState()
{
  //time reference update
  countdownTime = countdownEndTime - millis();

  //TODO: send remaining time troughout bluetooth

  if(millis() => countdownEndTime) //countdown over, proceed to launch
  {
    flightState = Launch;
    //TODO: log this info to flash memory
  }

  //read bluetooth for launch abort signal
  //TODO: if bluetooth disconnects, we want to hold countdown
  byte DATA;
  if(bluetooth.available() > 0)
  {
    DATA = bluetooth.read();

    switch (DATA) 
    {
      case '0': //stop countdown, abort launch
        flightState = LaunchAbort;
        break;

      case '2': //skip countdown
        flightState = Launch;
        break;

      case '3': //pause countdown
        if(!countdownPause)
        {
          countdownHoldTime = countdownEndTime - millis();
          countdownPause = true;
        }
        countdownEndTime = millis() + 20000000; //Around 5 and half hour, should be enought :)
        break;

      case '4': //resume countdown
        if(countdownPause)
        {
          countdownEndTime = millis() + countdownHoldTime;
          countdownPause = false;
        }
        break;
    }
  
  }
}

void LaunchState()
{
  digitalWrite(PYRO_1, HIGH); //pyro_1 should always be first stage rocket motor

  //TODO: check if launch is nominal
  flightState = Flight;
}

void FlightState()
{
  digitalWrite(PYRO_1, LOW); //despite pyro charge should no longer condct, for safety reasons we want to switch pyro off

  if(false) //TODO: apogee condition
  {
    flightState = Return;
  }
}
 
void ReturnState()
{
  if(true) //TODO: altitude condition
  {
    digitalWrite(PYRO_3, HIGH);
  }

  if(false) //TODO: backup charge condition
  {
    digitalWrite(PYRO_4, HIGH);
  }
  
}

void LandedState()
{

}

void LaunchAbortState()
{

}

void FlightAbortState()
{

}

//Pasive loops functions
void readIMUI2C()
{

}

void readIMUSPI()
{

}

void writeServos(int angleX, int angleZ)
{
  oldAngleX = servoX.read();
  oldAngleZ = servoZ.read();

  if(oldAngleX != angleX)
  {
    servoX.write(angleX);
  }

  if(oldAngleZ != angleZ)
  {
    servoZ.write(angleZ);
  }
}