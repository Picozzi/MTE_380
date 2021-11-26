// Libraries
#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <SharpIR.h>            // IR proximity sensor library
#include <L298NX2.h>            // DC Motor drive controller library (powers 2 motors)
#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PINS 9 AND 10 FOR PWM*/
#include <Adafruit_ICM20X.h>    // IMU library
#include <Adafruit_ICM20948.h>  // IMU specific model library
#include <Adafruit_Sensor.h>    // Adafruit sensor library
#include <math.h>               // Math library for M_PI constant
#include <PID_v1.h>             // PID controller library

/*
   Models:
   MUX: Adafruit TCA9548A 1-to-8 I2C Multiplexer
   Colour: Adagruit TCS34725
   IR: Sharp IR
   IMU: Adafruit TDK InvenSense ICM-20948
   DC motor drive controller: L298NX2
   Servo motor
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
#define irLegoThreshold 11        // Distance threshold to find the Lego man
// 5 V

SharpIR SharpIR(irPin, irModel); // RENAME TO irSensor

/* --- IMU Sensor Defs --- */
Adafruit_ICM20948 imu;

/* --- Motor Controller Defs --- */
// Motor A: left
// Motor B: right
#define enablePinA 3               // PWM signal for controlling speed Motor A
#define inPin1A 4                  // Digital input pin to control spin direction of Motor A
#define inPin2A 5                  // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

#define lowestMotorSpeed 150        // Lowest motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed

#define motorLAMaxSpeed 255         // Left Motor A Max Speed
#define motorRBMaxSpeed 255         // Right Motor B Max Speed
uint16_t baseSpeedMotorB = 60;      // Base motor B driving speed
uint16_t baseSpeedMotorA = 60;      // Base motor A driving speed

#define baseRed 470
#define KP 2.1
#define KD 0.85

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B);

/* --- Servo Motor Defs --- */
Servo servoMotor;
#define servoPin 2
#define openAngle 135        // Normal state where robot drives with claws open
#define closeAngle 55

#define targetTimeThreshold 500     // Timer to close claws around Lego man if IR sensor doesn't work

/*
   --- Boolean Flags ---
   There will be 3 cases for the run
   1) Searching for Lego man
   2) Found Lego man, turn around to go home
   3) Return home to the start location
*/
bool case1 = true;
bool case2 = false;
bool case3 = false;

double timer;

/*
   Since the MUX only allows 1 IC2 device to be read at one instance,
   use flag for when the right and left colour sensors read a desired
   colour.

   Red, green, blue colour sensor right and left flags
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

int lastError = 0;
int derivative;
int integral;

/*
   Setup function
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
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward(); // for zigZag function

  // setup servo motors
  servoMotor.attach(servoPin);
  servoMotor.write(openAngle);

  //  // PID controller
  //  input = analogRead(inputPin_PID);     // set-up PID
  //  setpoint = 100;
  //
  //  pid.SetMode(AUTOMATIC);               // turn PID on
  //  pid.SetTunings(Kp, Ki, Kd);
  //  pid.SetOutputLimits(0, 255);
}

/*
 * Clean main loop for testing functions
 */
void initTesting(){
//  dcMotorCheck();

//  motors.forward();

//  isRightColourSensor();

//  readingColours();

//  irCheck();

//  servoCheck();

//  imuCheck();

//  readIMU();

//  zigZagBase();

//  zigZag1();

//  zigZag2(); // New one
}

/*
   Main loop: KEEP. THIS IS THE RUN CODE
*/
void loop(void) {
  //  // --- PID controller ---
  //  // In the works when line sensor comes
  //  input = analogRead(inputPin_PID);
  //  pid.Compute();                        // PID calculation
  //  analogWrite(outputPin_PID, Output);
  //
  //  Serial.print(input);
  //  Serial.print(" ");
  //  Serial.println(output);

  int error = (rL - baseRed) - (rR - baseRed);
  error = error / 100;
//  Serial.println(error);

  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  
  
  // --- RUN ---
  if (case1)
  {
    // Case 1: follow line to find Lego man at target
//    motors.setSpeedB(60);
//    motors.setSpeedA(60);
//    motors.forward();

    // Red line following code
    int LAMotorSpeed = baseSpeedMotorA - motorSpeed; //motorA left
    int RBMotorSpeed = baseSpeedMotorB + motorSpeed; //motorB right
//    Serial.println(motorSpeed);

    // set motor speeds using the two motor speed variables above
    set_motors(LAMotorSpeed, RBMotorSpeed);
    
    // Check if found blue for target
    selectMuxPin(colourLeftAddress);
    if (foundBlue(colourLeft))
    {
      blueLeft = true;
      motors.stop(); // For testing
    }

    selectMuxPin(colourRightAddress);
    if (foundBlue(colourRight))
    {
      blueRight = true;
      motors.stop(); // For testing
    }

    // Check robot is facing center of target - if so, correct orientation
    if (blueLeft || blueRight) // initially wanted &&
    {
      timer = millis();

//      // Pause line following to find inner red ring of target
//      // Theoretically, only one colour sensor is enough of an indicator b/c the other should also be on top
//      selectMuxPin(colourLeftAddress);
//      if (foundRed(colourLeft))
//      {
//        redLeft = true;
//      }

      // If IR sensor sees Lego man or timed threshold is reached
      if ((getIRDist() < irLegoThreshold) || (millis() - timer) > targetTimeThreshold)
      {
        motors.stop();
        closeClaw(); // Grip man
        
        // Reset R/L colour sensor flags
        redRight = false;
        redLeft = false;
        blueRight = false;
        blueLeft = false;
        greenRight = false;
        greenLeft = false;

        case1 = false;
        case2 = true;
        return; // in final version
      }
    }
  }
  else if (case2)
  {
    // Case 2: picked-up Lego man and turn robot around to return to start location
    servoMotor.write(40); // Just in case, redundant could to hold Lego man
    motors.stop();

    // back up robot to avoid misinterpreting red inner ring of target as red line
//    timer = millis();
//    while ((millis() - timer) < 500) {
//      motors.backward();
//    }

    // Turn robot around
    // Use IMU for 180 deg; if condition
    motors.backwardA(); // Left wheel turns backward
    motors.forwardB();  // Right wheel turns forward
    turn180();
    motors.stop();
    
    case2 = false;
    case3 = true;
    return;
  }
  else if (case3) {
    // Case 3: follow line to return home to the start location
//    motors.setSpeedB(baseSpeedMotorB);
//    motors.setSpeedA(baseSpeedMotorA);
//    motors.forward();

    // Red line following code
    int LAMotorSpeed = baseSpeedMotorA - motorSpeed; //motorA left
    int RBMotorSpeed = baseSpeedMotorB + motorSpeed; //motorB right
//    Serial.println(motorSpeed);

    // set motor speeds using the two motor speed variables above
    set_motors(LAMotorSpeed, RBMotorSpeed);
    
    // Found red horizontal start line
    if (redLeft && redRight){
      motors.stop();
      openClaw();
      Serial.println("DONE");
      while(1){}  // DONE
      return;
    }

    // Red line following code
    // CALL FINAL ALGO THAT WORKS
    
    // Check if found red horizontal start line for both colour sensors
    selectMuxPin(colourLeftAddress);
    if (foundRed(colourLeft))
    {
      redLeft = true;
    }

    selectMuxPin(colourRightAddress);
    if (foundRed(colourRight))
    {
      redRight = true;
    }
  }
}
