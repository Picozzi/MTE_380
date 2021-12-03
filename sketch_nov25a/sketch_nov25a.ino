#include <Wire.h>               // I2C library
#include <Servo.h>
#include <Adafruit_TCS34725.h>  // Colour sensor library

#define muxAddress 0x70           // Multiplexer address for I2C
#define colourRightAddress 0
#define colourLeftAddress 1
#define imuAddress 2

Servo servo;

int enA = 3;
int in1 = 1;//2, 4 can't go high
int in2 = 6; 
int enB = 11;
int in3 = 12;
int in4 = 13;

uint16_t r, g, b, clear;
uint16_t rR, gR, bR;
uint16_t rL, gL, bL;

Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);

  Wire.begin();

  Serial.println("\nSTART");

  // Setup colour sensors
  selectMuxPin(colourLeftAddress);
  if (colourLeft.begin()) {
    Serial.println("Found left colour sensor!");
  }
  else {
    Serial.println("Left colour sensor was not found.");
  }

  selectMuxPin(colourRightAddress);
  if (colourRight.begin()) {
    Serial.println("Found right colour sensor!");
  }
  else {
    Serial.println("Right colour sensor was not found.");
  }

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  servo.attach(5);
  //servoOpen();
  servoClose();
}



void loop() {
  // put your main code here, to run repeatedly:
  //calibrateColour();
  
  //stopAtRed(60);
  
  //reverseTest(60);
  
  //zigZag(60, false, false, true); 
  rescueMan(60, false, false, true);
  Serial.println("Return");
  returnTrip(60, false, false, true);
  while(1){}
}
