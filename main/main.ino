// Libraries
#include <Wire.h>               // I2C library
#include "Adafruit_TCS34725.h"  // colour sensor library
#include <PID_v1.h>             // PID controller library

// --- Colour Sensor Defs ---
// Pins
/*NOTE: that the pin vals here are arbitrary - go with whatever electrical needs*/
#define redPin 3              // PWM output for red anode
#define greenPin 5            // PWM output for green anode
#define bluePin 6             // PWM output for blue anode

#define commonAnode false     // using a common cathode LED

byte gammaTable[256];         // RGB gamma colour

/*NOTE: In the future, initialize 2 colourSensor vars (left and right side)*/
Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//// --- PID Controller Defs ---
//double Setpoint, Input, Output;
//PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);


void setupColourSensor(){
  if (colourSensor.begin()){
    // If the sensor starts up correctly
    Serial.println("Found colour sensor!");
  } else {
    Serial.println("The colour sensor was not found...");
    // while (1); // pause
  }

  // Set pin outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  for (int i=0; i<256; i++){
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode){
      gammaTable[i] = 255 - x;  
    } else {
      gammaTable[i] = x;
    }
  }
 }

void runColourSensor(){
  uint16_t clear, red, green, blue, colourTemp, lux;

  colourSensor.setInterrupt(false);                       // turn on LED
  delay(1000);                                            // wait 1000 ms for reading input
  colourSensor.getRawData(&red, &green, &blue, &clear);   // read sensor input

  colourSensor.setInterrupt(true);                        // turn off LED

  // Print read colour sensor values
  Serial.print("C:\t");
  Serial.print(clear);
  Serial.print("\tR:\t");
  Serial.print(red);
  Serial.print("\tG:\t");
  Serial.print(green);
  Serial.print("\tB:\t");
  Serial.print(blue);
}

void setup() {
  Serial.begin(9600);                       // [bits/s] Communication data rate between Arduino and Serial Monitor

  // Colour sensor
  Serial.println("Colour Sensor Test");
  setupColourSensor();

//  // PID controller
//  //initialize the variables we're linked to
//  Input = analogRead(0);
//  Setpoint = 100;
//
//  myPID.SetMode(AUTOMATIC);   // turn PID on

}

void loop(void) {
  // Colour sensor
  Serial.println("\tRun colour sensor\t");
  runColourSensor();

//  // PID controller
//  Input = analogRead(0);
//  myPID.Compute();
//  analogWrite(3,Output);
}
