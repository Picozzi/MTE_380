// Libraries
#include <Wire.h>               // I2C library
#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PINS 9 AND 10 FOR PWM*/
#include <SharpIR.h>            // IR proximity sensor library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <Adafruit_ICM20X.h>    // IMU library
#include <Adafruit_ICM20948.h>  // IMU specific model library
#include <Adafruit_Sensor.h>    // Adafruit sensor library
#include <math.h>               // Math library for M_PI constant

/*
   Models:
   MUX: Adafruit TCA9548A 1-to-8 I2C Multiplexer
   Colour: Adagruit TCS34725
   IR: Sharp IR
   IMU: Adafruit TDK InvenSense ICM-20948
   DC motor drive controller: L298NX2
   Servo motor: SG90 
*/

/* --- MUX Defs --- */
#define muxAddress 0x70         // Multiplexer address for I2C
#define colourRightAddress 0
#define colourLeftAddress 1
#define imuAddress 2
// 5 V

/* --- Colour Sensor Defs --- */
Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// 3.3 V

// RGB colour sensor readings
uint16_t r, g, b, clear;
uint16_t rR, gR, bR;             // Right colour sensor
uint16_t rL, gL, bL;             // Left colour sensor

/* --- IR Proximity Sensor Defs --- */
#define irPin A0                 // Analog input pin
#define irModel 1080             // sensor model library uses for GP2Y0A21YK
#define irThreshold 11           // Distance threshold to find the Lego man and stop at start location
// 5 V

SharpIR SharpIR(irPin, irModel);

/* --- IMU Sensor Defs --- */
Adafruit_ICM20948 imu;

/* --- Servo Motor Defs --- */
Servo servo;
#define servoPin 2
#define openAngle 135             // Normal state where robot drives with claws open
#define closeAngle 40             // Claws are closed around Lego man

/* --- Motor Controller Defs --- */
// Motor A: left DC motor
// Motor B: right DC motor
#define enA 3                     // PWM signal for controlling speed Motor A
#define in1 4                     // Digital input pin to control spin direction of Motor A
#define in2 5                     // Digital input pin to control other spin direction of Motor A

#define enB 11                    // PWM signal for controlling speed Motor B
#define in3 12                    // Digital input pin to control spin direction of Motor B
#define in4 13                    // Digital input pin to control other spin direction of Motor B

#define baseSpeed 60              // Base motor driving speed

/*
   --- Boolean Flags ---
   There will be 3 cases for the run
   1) Searching for Lego man, found Lego man, and turn around to go home
   2) Return home to the start location
*/
bool case1 = true;
bool case2 = false;

/*
   Since the MUX only allows 1 IC2 device to be read at one instance,
   use flag for when the right and left colour sensors read a desired
   colour.

   Red and blue colour sensor right and left flags
*/
bool redRight = false;
bool redLeft = false;
bool blueRight = false;
bool blueLeft = false;

/*
 * Setup function
 */
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

  // setup IMU
  selectMuxPin(imuAddress);
  if (imu.begin_I2C()) {
    Serial.println("Found IMU sensor!");
  }
  else {
    Serial.println("IMU sensor was not found.");
  }

  imu.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);

  // setup DC motors (front wheels)
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // setup servo motors
  servo.attach(servoPin);
  servo.write(openAngle);
}

/*
 * Main loop
 */
void loop() {
  //calibrateColour();    // Initial calibration of environment colours for colour sensor

  if (case1)
  {
    // Case 1: Find and pick-up Lego man
    findTarget(baseSpeed, false, false, true); 
    case1 = false;
    case2 = true;   
  }
  else if (case2)
  {
    // Case 2: Carry Lego man and return to start location. Drop off Lego man.
    returnTrip(baseSpeed, false, false, true);
    while(1){}  // DONE
  }
}
