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
#define irLegoThreshold 10        // ***CHANGE THROUGH TESTING; distance threshold to find the Lego man
#define irStarThreshold 15   // CHANGE THROUGH TESTING
// 5 V

SharpIR SharpIR(irPin, irModel); // RENAME TO irSensor

/* --- IMU Sensor Defs --- */
Adafruit_ICM20948 imu;

double prevAngVelocity;
double prevAngle;
double prevTime;
double angPosition = 0;

/* --- Motor Controller Defs --- */
// Motor A: left
// Motor B: right
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

// FINALIZE THESE ONCE TESTING IS COMPLETE
#define lowestMotorSpeed 150        // Lowest motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed

uint16_t baseSpeedMotorB = 140;                          // Base motor B driving speed
uint16_t baseSpeedMotorA = baseSpeedMotorB + 10;         // Base motor A driving speed

// motor A: left - bad (offset by 70)

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B);

/* --- Servo Motor Defs --- */
Servo servoMotor;
#define servoPin 2

#define targetTimeThreshold 500     // Timer to close claws around Lego man if IR sensor doesn't work
// TESTING FOR NUMBER

/*
   --- Boolean Flags ---
   There will be 3 cases for the run
   1) Searching for Lego man
   2) Found Lego man, drop him off at a safe zone
   3) Turn around to go home
   4) Return home to the start location
*/
bool case1 = true;
bool case2 = false;
bool case3 = false;
bool case4 = false;

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

/*
   Setup function
*/
void setup() {
  Serial.begin(9600);

  Wire.begin();

  Serial.println("START");

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

  imu.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

//  // setup DC motors (front wheels)
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward(); // for zigZag function

  //  // setup servo motors
  //  servoMotor.attach(servoPin);
  //  servoMotor.write(160);

  //  // PID controller
  //  input = analogRead(inputPin_PID);     // set-up PID
  //  setpoint = 100;
  //
  //  pid.SetMode(AUTOMATIC);               // turn PID on
  //  pid.SetTunings(Kp, Ki, Kd);
  //  pid.SetOutputLimits(0, 255);
}

/*
 * Main loop
 */
void loop(void){
  getYawAngle();
}

/*
 * 
 */
void getYawAngle(){
//  float currAngle = 0;

  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  float accelX = accel.accel.x;
  float accelY = accel.accel.y;
  float accelZ = accel.accel.z;
  
  float pitchAngle = 180 / M_PI * atan(accelX / sqrt(accelY * accelY + accelZ * accelZ));
  float rollAngle = 180 / M_PI * atan(accelY / sqrt(accelX * accelX + accelZ * accelZ));
  float yawAngle = 180 / M_PI * atan(accelZ / sqrt(accelX * accelX + accelZ * accelZ));

  Serial.println();
  Serial.print("\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  Serial.print("\tYaw Angle: ");
  Serial.print(yawAngle);
}
