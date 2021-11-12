// Libraries
#include <Wire.h>               // I2C library
#include "Adafruit_TCS34725.h"  // Colour sensor library
#include <PID_v1.h>             // PID controller library
#include <SharpIR.h>            // IR proximity sensor library
#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)
#include <Servo.h>              // Servo motor library

/* --- Colour Sensor Defs --- */
/*NOTE: that the pin vals here are arbitrary - go with whatever electrical needs*/
//#define redPin 2                // Digital pin
//#define greenPin 3              // PWM pin (doesn't have to be PWM)
//#define bluePin 4               // Digital pin

//#define commonAnode false         // for using a common cathode LED

//byte gammaTable[256];             // RGB gamma colour

/*NOTE: In the future, initialize 2 colourSensor vars (left and right side)*/
Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/* --- IR Proximity Sensor Defs --- */
#define irPin A0                  // Analog input pin
#define irModel 1080              // sensor model library uses for GP2Y0A21YK
#define irLegoThreshold 5         // ***CHANGE THROUGH TESTING; distance threshold to find the Lego man
// 5 V

SharpIR SharpIR(irPin, irModel); // Unsure if I can/it's better to assign a var to this object

/* --- Motor Controller Defs --- */
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define inPin1B 7                  // Digital input pin to control spin direction of Motor B
#define inPin2B 8                  // Digital input pin to control spin direction of Motor B
#define enablePinB 9               // PWM signal for controlling speed Motor B

#define standardMotorSpeed 50      // Standard motor driving speed
// motor A: left - bad (offset by 70)

// 
L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B);

// HERE***********

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
  Serial.begin(9600);                       // [bits/s] Communication data rate between Arduino and Serial Monitor

  if (colourSensor.begin()){
    // If the sensor starts up correctly
    Serial.println("Found colour sensor!");
  } else {
    Serial.println("The colour sensor was not found...");
    // while (1); // pause
  }

//  setupServo();
  setupMotors();
  
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
//  String currentColour1 = identifyColour(colourSensor);
  
  // Testing
//  followRedLine(colourSensor, colourSensor2);
//  testingColourSensor(colourSensor);
//  runServo();
  constructionCheckMotors(colourSensor);
  
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
