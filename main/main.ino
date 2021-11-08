// Libraries
#include <Wire.h>               // I2C library
#include "Adafruit_TCS34725.h"  // colour sensor library
#include <PID_v1.h>             // PID controller library
#include <SharpIR.h>            // IR proximity sensor library

/* --- Colour Sensor Defs --- */
/*NOTE: that the pin vals here are arbitrary - go with whatever electrical needs*/
#define redPin 3              // PWM output pin
#define greenPin 5            // PWM output pin
#define bluePin 6             // PWM output pin

#define commonAnode false     // for using a common cathode LED

byte gammaTable[256];         // RGB gamma colour

/*NOTE: In the future, initialize 2 colourSensor vars (left and right side)*/
Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/* --- IR Proximity Sensor Defs --- */
#define irPin A0            // Analog input pin
#define irModel 1080        // sensor model library uses for GP2Y0A21YK

SharpIR SharpIR(irPin, irModel);


//// --- PID Controller Defs ---
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

  // Colour sensor
  Serial.println("Colour Sensor Testing");
  setupColourSensor();

//  // PID controller
//  Input = analogRead(inputPin_PID);       // set-up PID
//  Setpoint = 100;
//
//  myPID.SetMode(AUTOMATIC);               // turn PID on
//  myPID.SetTunings(Kp, Ki, Kd);
}

void loop(void) {
  // Colour sensor
  Serial.println("\nRun colour sensor\t");
  runColourSensor();

  // IR sensor
  Serial.println("IR Sensor Testing");
  Serial.println("Run IR sensor");
  runIRSensor();

//  // PID controller
//  Input = analogRead(inputPin_PID);
//  myPID.Compute();                        // PID calculation
//  analogWrite(outputPin_PID, Output);

//  Serial.print(Input);
//  Serial.print(" ");
//  Serial.println(Output);
}
