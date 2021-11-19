// Libraries
#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <SharpIR.h>            // IR proximity sensor library
#include <L298NX2.h>            // DC Motor drive controller library (powers 2 motors)
#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PINS 9 AND 10 FOR PWM*/
#include <Adafruit_ICM20X.h>    // IMU library
#include <Adafruit_ICM20948.h>  // IMU specific model library
#include <Adafruit_Sensor.h>    // Adafruit sensor library
#include <PID_v1.h>             // PID controller library

bool testing = false; // TOGGLE THIS WHEN TESTING

/*
 * Models:
 * MUX: Adafruit TCA9548A 1-to-8 I2C Multiplexer
 * Colour: Adagruit TCS34725
 * IR: Sharp IR
 * IMU: Adafruit TDK InvenSense ICM-20948
 * DC motor drive controller: L298NX2
 * Servo motor
 */

/* --- MUX Defs --- */
#define muxAddress 0x70           // Multiplexer address for I2C
#define colourRightAddress 0
#define colourLeftAddress 1
#define imuAddress 2
// 5 V

/* --- Colour Sensor Defs --- */
Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // NOTE: CAN PROBABLY CHANGE THE INPUT PARAMS
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
// 3.3 V

/* --- IR Proximity Sensor Defs --- */
#define irPin A0                  // Analog input pin
#define irModel 1080              // sensor model library uses for GP2Y0A21YK
#define irLegoThreshold 10        // ***CHANGE THROUGH TESTING; distance threshold to find the Lego man
#define irStartWallThreshold 15   // CHANGE THROUGH TESTING
// 5 V

SharpIR SharpIR(irPin, irModel); // RENAME TO irSensor

/* --- IMU Sensor Defs --- */
Adafruit_ICM20948 imu;

/* --- Motor Controller Defs --- */
// Motor A: left
// Motor B: right
/* NOTE: May name motorB as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

// FINALIZE THESE ONCE TESTING IS COMPLETE
#define lowestMotorSpeed 150        // Lowest motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed
#define baseSpeedMotorA 220         // Base motor A driving speed
#define baseSpeedMotorB 150         // Base motor B driving speed
// motor A: left - bad (offset by 70)

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B); // RENAME TO dcMotors

/* --- Servo Motor Defs --- */
Servo servoMotor;
#define servoPin 2

// --- Boolean Flags ---
/*
 * There will be 3 cases for the run
 * 1) Searching for Lego man
 * 2) Found Lego man, drop him off at a safe zone
 * 3) Return home to the start location
 */
bool foundLego1 = false;
bool droppedOffLego2 = false;
bool returningHome3 = false;

/*
 * Since the MUX only allows 1 IC2 device to be read at one instance, 
 * use flage for when the right and left colour sensors read a desired 
 * colour.
 */
bool redRight = false;
bool redLeft = false;
bool blueRight = false;
bool blueLeft = false;
bool greenRight = false;
bool greenLeft = false;

/* --- PID Controller Defs --- */
/* NOTE: MAY NEED TO CHANGE/REMOVE THESE PIN DEFS */
/*
#define inputPin_PID 0        // PID input
#define outputPin_PID 3       // PID output

double setpoint, input, output;

// NOTE: Apparantly these constants are the ones to be tuned
double Kp = 2;
double Ki = 5;
double Kd = 1;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
*/

/*
 * Set up function
 */
void setup() {
  Serial.begin(9600);   // [bits/s] Communication data rate between Arduino and Serial Monitor

  Wire.begin();

  Serial.println("START");
  
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

//  imu.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
//  imu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

  // setup DC motors (front wheels)
  motors.setSpeed(standardMotorSpeed);        // Set initial speed for both motors
  motors.setSpeedA(baseSpeedMotorA);

//  // setup servo motors
//  servoMotor.attach(servoPin);
//  servoMotor.write(160);

//  // PID controller
//  input = analogRead(inputPin_PID);       // set-up PID
//  setpoint = 100;
//
//  pid.SetMode(AUTOMATIC);               // turn PID on
//  pid.SetTunings(Kp, Ki, Kd);
  //  pid.SetOutputLimits(0, 255);
}

void loop(void) {
  // --- Colour sensor ---

  /* Set up colour sensor */
//  testingColourSensor(colourLeft, colourRight);
  
  // Testing
//  uint16_t leftMotorSpeed = standardMotorSpeed + 5;
  
//  followRedLine(colourLeft, colourRight, leftMotorSpeed);

  // --- RUN ---
//  motors.setSpeed(standardMotorSpeed);
//  motors.forward();
//  
//  // Sensor 1
//  uint16_t r1, g1, b1, clear1;//, lux1;
//  colourRight.getRawData(&r1, &g1, &b1, &clear1);
//
// // Sensor 2
//  uint16_t r2, g2, b2, clear2;//, lux2;
//  colourLeft.getRawData(&r2, &g2, &b2, &clear2);
//
//  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight)){ // NOTE: DEFINE CONSTANTS FOR THESE COLOUR RANGES
//    // Shift leftwards
//       motors.stopA();
//       Serial.println("Stop A");
//  } 
//
//  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft)){
//    // Shift rightwards
//       motors.stopB();
//       Serial.println("Stop B");
//  }
//
//  if (getIRDist() < 15){ // WHY DO WE NEED DELAYS?
//    servoMotor.write(10);
//    delay(500);
//    servoMotor.write(180);
//    delay(500);
//  }

  // blue statement - bool -- while poll IR 

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

//  // --- PID controller ---
//  input = analogRead(inputPin_PID);
//  pid.Compute();                        // PID calculation
//  analogWrite(outputPin_PID, Output);
//
//  Serial.print(input);
//  Serial.print(" ");
//  Serial.println(output);

  // --- Testing line following ---
  lineFollowing();
}

void lineFollowing(){
  redRight = false;
  redLeft = false;
  blueRight = false;
  blueLeft = false;
  greenRight = false;
  greenLeft = false;

  /*
  // Colour sensor 1
  uint16_t rR, gR, bR, clearR;//, lux1;
  colourRight.getRawData(&rR, &gR, &bR, &clearR);

  // Colour sensor 2
  uint16_t rL, gL, bL, clearL;//, lux2;
  colourLeft.getRawData(&rL, &gL, &bL, &clearL);

  */
  
  /* 
   *  make Case switch or if statements to differentiate between:
   *  1) finding target (lego man)
   *  2) finding safe zone
   *  3) finding home (start)
   */

  if (testing){
    motors.setSpeedB(standardMotorSpeed);
    motors.setSpeedA(baseSpeedMotorA);
    motors.forward();

    selectMuxPin(colourRightAddress);
    if (foundRed(colourRight)){
      Serial.println("\nRight");
      motors.stop();
      while(1){}
    }    
    
    selectMuxPin(colourLeftAddress);
    if (foundRed(colourLeft)){
      Serial.println("\nRight");
      motors.stop();
      while(1){}
    }
  }

//  if (!foundLego){
//    // Case 1
//    // Chuck everything related in here    
//  }

  else {
    // --- RUN ---
    motors.setSpeedB(standardMotorSpeed);
    motors.setSpeedA(baseSpeedMotorA);
    motors.forward();
  
  //  // Check for target
  //  // find blue -- place here so we don't miss it
  //  selectMuxPin(colourLeftAddress);
  //  if (foundBlue(colourLeft)){ // NOTE: DEFINE CONSTANTS FOR THESE COLOUR RANGES
  //    blueLeft = true;
  //    motors.stop();
  //  }
  //  
  //  selectMuxPin(colourRightAddress);
  //  if (foundBlue(colourRight)){
  //    blueRight = true;
  //      motors.stop();
  //  }
  //

  //  // Check robot is facing center of target
//    if (blueLeft && blueRight){
//      // claw algo
//     // slow walk until ir sees it
//      motors.setSpeed(lowestMotorSpeed);
//      motors.forward();
//      if (getIRDist() < irLegoThreshold){
//          motors.stop();
//          closeClaw();
//        // break; // in final version
//        while(1){
//          // for now
//        } 
//      }     
//    }
//      else {
//        // potentially pushing the lego man if the ir sensor doesn't work
//        // timer to close claws and proceed with next step
//        // break;
//      }

  // NOTE: SHARP TURNS, STOP/SLOW DOWN OTHER MOTOR

    // WORKS - 7:10 PM TUES. NOV 16
    // Shift right
    selectMuxPin(colourLeftAddress);
    if (!foundRed(colourLeft))
    { // NOTE: DEFINE CONSTANTS FOR THESE COLOUR RANGES
      selectMuxPin(colourRightAddress);
      if (foundRed(colourRight))
      {
        while(foundRed(colourRight))
        {
          motors.setSpeedA(baseSpeedMotorA + 50);
          motors.forward();
//          Serial.println("Turning right");
        }
      }
    }
  
    motors.setSpeedB(standardMotorSpeed);
    motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
    motors.forward();
    
    // Shift left
    selectMuxPin(colourRightAddress);
    if (!foundRed(colourRight))
    {
      selectMuxPin(colourLeftAddress);
      if (foundRed(colourLeft))
      {
        while(foundRed(colourLeft))
        {
          motors.setSpeedB(standardMotorSpeed + 50);
          motors.forward();
//          Serial.println("Turning left");
        }
      }
    }
  
    motors.setSpeedB(standardMotorSpeed);
    motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
    motors.forward();
  }
}
