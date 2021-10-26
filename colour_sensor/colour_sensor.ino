// Libraries
#include <Wire.h>               // I2C library
#include "Adafruit_TCS34725.h"  // colour sensor library

// Pins
/*NOTE: that the pin vals here are arbitrary - go with whatever electrical needs*/
#define redPin 3              // PWM output for red anode
#define greenPin 5            // PWM output for green anode
#define bluePin 6             // PWM output for blue anode

#define commonAnode false     // using a common cathode LED

byte gammeTable[256];         // RGB gamma colour

/*NOTE: In the future, initialize 2 colourSensor vars (left and right side)*/
Adafruit_TCS34725 colourSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);         // [bits/s] Communication data rate between Arduino and Serial Monitor
  Serial.println("Colour Sensor Test");

  if (colourSensor.begin()){
    // If the sensor starts up correctly
    Serial.println("Found colour sensor!");
  } else {
    Serial.println("The colour sensor was not found...");
    while (1); // pause
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

void loop(void) {
  uint16_t clear, red, green, blue, colourTemp, lux;

  colourSensor.setInterrupt(false);                       // turn on LED
  delay(60);                                              // wait 60 ms for reading input
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
