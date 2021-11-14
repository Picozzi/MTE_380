// Libraries
#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
//#include "Adafruit_TCS34725.h"  
#include <PID_v1.h>             // PID controller library
#include <SharpIR.h>            // IR proximity sensor library
#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)
#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PIN 9/10 FOR PWM*/
#include <Adafruit_ICM20X.h>    // IMU library
#include <Adafruit_ICM20948.h>  // IMU specific model library
#include <Adafruit_Sensor.h>    // Adafruit sensor library

/*
 * Models:
 * IMU: Adafruit TDK InvenSense ICM-20948
 * MUX: Adafruit TCA9548A 1-to-8 I2C Multiplexer
 * Colour: 
 */

/* MUX Defs */
#define muxAddress 0x70           // Multiplexer address for I2C
#define colourLeftAddress 0
#define colourRightAddress 1
#define imuAddress 2
// 5 V

/* --- Colour Sensor Defs --- */
/*NOTE: that the pin vals here are arbitrary - go with whatever electrical needs*/
//#define redPin 2                // Digital pin
//#define greenPin 3              // PWM pin (doesn't have to be PWM)
//#define bluePin 4               // Digital pin

//#define commonAnode false         // for using a common cathode LED

//byte gammaTable[256];             // RGB gamma colour

Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // NOTE: CAN PROBABLY CHANGE THE INPUT PARAMS
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// 3.3 V

/* --- IR Proximity Sensor Defs --- */
#define irPin A0                  // Analog input pin
#define irModel 1080              // sensor model library uses for GP2Y0A21YK
#define irLegoThreshold 5         // ***CHANGE THROUGH TESTING; distance threshold to find the Lego man
// 5 V

SharpIR SharpIR(irPin, irModel); // Unsure if I can/it's better to assign a var to this object

/* IMU Sensor Defs */
Adafruit_ICM20948 imu;

/* --- Motor Controller Defs --- */
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B
#define enablePinB 6               // PWM signal for controlling speed Motor B

#define lowestMotorSpeed 150        // Standard motor driving speed
#define standardMotorSpeed 255      // Standard motor driving speed
// motor A: left - bad (offset by 70)

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B);

/* Servo Motor Defs */
Servo servo;
#define servoPin 2

/* --- PID Controller Defs --- */
/* NOTE: MAY NEED TO CHANGE/REMOVE THESE PIN DEFS */
//#define inputPin_PID 0        // PID input
//#define outputPin_PID 3       // PID output

//double Setpoint, Input, Output;

// NOTE: Apparantly these constants are the ones to be tuned
//double Kp = 2;
//double Ki = 5;
//double Kd = 1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);   // [bits/s] Communication data rate between Arduino and Serial Monitor

  Wire.begin();

  Serial.println("HERE");
  // Setup colour sensors
  selectMuxPin(colourLeftAddress);
  if (colourLeft.begin()){
    // If the sensor starts up correctly
    Serial.println("Found left colour sensor!");
  } 
  else {
    Serial.println("Left colour sensor was not found.");
  }

  selectMuxPin(colourRightAddress);
  if (colourRight.begin()){
    // If the sensor starts up correctly
    Serial.println("Found right colour sensor!");
  }
  else {
    Serial.println("Right colour sensor was not found.");
  }

//  // setup IMU
//  selectMuxPin(imuAddress);
//  if (imu.begin_I2C()) {
//    // If the IMU sensor was connected correctly.
//    Serial.println("Found IMU sensor!");
//  }
//  else {
//    Serial.println("IMU sensor was not found.");
//  }

  // setup DC motors (front wheels)
  motors.setSpeed(standardMotorSpeed);        // Set initial speed for both motors

//  // setup servo motors
 servo.attach(servoPin);
 servo.write(10);

//  // PID controller
//  Input = analogRead(inputPin_PID);       // set-up PID
//  Setpoint = 100;
//
//  myPID.SetMode(AUTOMATIC);               // turn PID on
//  myPID.SetTunings(Kp, Ki, Kd);
}

void loop(void) {
  // --- Colour sensor ---
  // Read sensor 1
//  Serial.println("\nColour Sensor");
//  String currentColour1 = identifyColour(colourLeft);

  /* Set up colour sensor */
//  testingColourSensor(colourLeft, colourRight);
  
  // Testing
//  uint16_t leftMotorSpeed = standardMotorSpeed + 5;
  
//  followRedLine(colourLeft, colourRight, leftMotorSpeed);
  motors.setSpeed(standardMotorSpeed);
  motors.forward();
  
  // Sensor 1
  uint16_t r1, g1, b1, clear1;//, lux1;
  colourRight.getRawData(&r1, &g1, &b1, &clear1);

 // Sensor 2
  uint16_t r2, g2, b2, clear2;//, lux2;
  colourLeft.getRawData(&r2, &g2, &b2, &clear2);

  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight)){ // NOTE: DEFINE CONSTANTS FOR THESE COLOUR RANGES
    // Shift leftwards
       motors.stopA();
       Serial.println("Stop A");
  } 

  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft)){
    // Shift rightwards
       motors.stopB();
       Serial.println("Stop B");
  }

  if (getIRDist() < 15){ // WHY DO WE NEED DELAYS?
    servo.write(10);
    delay(500);
    servo.write(180);
    delay(500);
  }

  // blue statement - bool -- while poll IR 

//  motors.forward();
//  resetMotorsSpeed(leftMotorSpeed);
//  motors.forward();
//  testingColourSensor(colourLeft);
//  runServo();
//  constructionCheckMotors(colourLeft);

  // --- IMU sensor ---
//  readIMU();
  
  // --- IR sensor ---
//  Serial.println("\nIR Sensor");
//  unsigned long currIRDist = getIRDistance();
//  testingIR();

  // ALGORITHM - TBD
//  if (currentColour1 == "blue" && currIRDist >= irLegoThreshold){
      // Keep driving
//      runMotors();
//  }
//  else {
//    // Found target circle (Lego man)
//    // claws to pick up Lego man
//  }

  // Motors
//  Serial.println("Motors Testing");
//  runMotors();

// LATER
//  // PID controller
//  Input = analogRead(inputPin_PID);
//  myPID.Compute();                        // PID calculation
//  analogWrite(outputPin_PID, Output);

//  Serial.print(Input);
//  Serial.print(" ");
//  Serial.println(Output);
}
