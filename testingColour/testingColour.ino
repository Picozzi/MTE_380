#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)
#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PIN 9/10 FOR PWM*/
#include <Adafruit_ICM20X.h>    // IMU library
#include <Adafruit_ICM20948.h>  // IMU specific model library
#include <Adafruit_Sensor.h>    // Adafruit sensor library

/* --- Servo Motor Defs --- */
Servo servoMotor;
#define servoPin 2

//uint16_t angle;

/* --- IMU Sensor Defs --- */
Adafruit_ICM20948 imu;

double prevAngVelocity;
double prevAngle;
double prevTime;
double angPosition = 0;

/* --- Colour Sensor --- */
Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // NOTE: CAN PROBABLY CHANGE THE INPUT PARAMS
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Red, green, blue colour sensor right and left flags
bool redRight = false;
bool redLeft = false;
bool blueRight = false;
bool blueLeft = false;

bool foundMan = false;

/* --- MUX Defs --- */
#define muxAddress 0x70           // Multiplexer address for I2C
#define colourRightAddress 0
#define colourLeftAddress 1
#define imuAddress 2

/* --- Motor Controller Defs --- */
// Motor A: left
// Motor B: right
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

#define lowestMotorSpeed 150        // Lowest motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed
//#define baseSpeedMotorA 224         // Base motor A driving speed
//#define baseSpeedMotorB 150         // Base motor B driving speed

int baseSpeedMotorB = 140;
int baseSpeedMotorA = baseSpeedMotorB + 10;         // Base motor A driving speed

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B);

void setup() {
  Serial.begin(9600);

  Wire.begin();

  // Setup colour sensors
  selectMuxPin(colourLeftAddress);
  if (colourLeft.begin()) 
  {
    // If the sensor starts up correctly
    Serial.println("Found left colour sensor!");
  }
  else 
  {
    Serial.println("Left colour sensor was not found.");
  }

  selectMuxPin(colourRightAddress);
  if (colourRight.begin()) 
  {
    // If the sensor starts up correctly
    Serial.println("Found right colour sensor!");
  }
  else 
  {
    Serial.println("Right colour sensor was not found.");
  }

  // setup IMU
  selectMuxPin(imuAddress);
  if (imu.begin_I2C()) 
  {
    // If the IMU sensor was connected correctly.
    Serial.println("Found IMU sensor!");
  }
  else 
  {
    Serial.println("IMU sensor was not found.");
  }  

  imu.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  imu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);

  // setup servo motors
  //servoMotor.attach(servoPin);
  //  servoMotor.write(160);
  
  // setup DC motors (front wheels)
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward(); // for zigZag function
}

/*
 * Main loop
 */
void loop() {  
    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp, &mag);
      
    double nextAngPosition = 0; //should be velocity
    double currentTime;
    double endTime;
    double newAngPosition = 0;

      motors.forwardB();
      motors.backwardA();
      //while (abs(gyro.gyro.z) >= 0.05)
      while(1)
      {
        currentTime = millis();
        //while (abs(gyro.gyro.z) >= 0.05 )
        //{ 
//          if (abs(nextAngPosition) > abs(newAngPosition))
//          {
//            newAngPosition = nextAngPosition;
//          }
          imu.getEvent(&accel, &gyro, &temp, &mag);
          nextAngPosition = gyro.gyro.z;
        //}  
        endTime = millis();
        //angPosition += (nextAngPosition+newAngPosition)/2 *180/3.14 * (endTime - currentTime)/1000;
        angPosition += nextAngPosition *180/3.14 * (endTime - currentTime)/1000;
        if (angPosition > 720)
         break;
      }
      motors.setSpeed(0);
      
      Serial.print("\nAngular position: ");
      Serial.print(angPosition);
      Serial.println();
      Serial.print(" \tZ: ");
      Serial.print(gyro.gyro.z);
      Serial.println(" radians/s ");
      delay(1000);        
 
//  zigZagAndGrip();
}

void zigZagAndGrip(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
  }

  while (redRight)
  {
    motors.stopB();           
    selectMuxPin(colourLeftAddress);
    if (foundBlue(colourLeft))
    {
      blueLeft = true;
      motors.stop();
      delay(2000);
      motors.forward();
      if (foundRed(colourLeft))
      {
        delay(2000);
        foundMan = true;
        motors.stop();
        while(1){}
      }
    }
    if (foundRed(colourLeft)){
      motors.stopA();
      redRight = false;
      break;
    }
  } 
     
  redRight = false;
  motors.forwardB();

  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
  }
    
  while (redLeft)
  {
    motors.stopA();         
    selectMuxPin(colourRightAddress);
    if (foundBlue(colourRight))
    {
      blueRight = true;
      motors.stop();
      delay(2000);
      motors.forward();
      if (foundRed(colourRight))
      {
        delay(2000);
        foundMan = true;
        motors.stop();
        while(1){}
      }
    }         
    if (foundRed(colourRight))
    {
      motors.stopB();
      redLeft = false;
      break;
    }
  }
  
  redLeft = false;
  motors.forwardA();  
}

/*
 * Zigzag line following with colour sensors
 */
void testingZigZag(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
  }

  while (redRight){
     motors.stopB();
            
     selectMuxPin(colourLeftAddress);
     if (foundRed(colourLeft)){
        motors.stopA();
        redRight = false;
        break;
     }
  }

  redRight = false;
  motors.forwardB();
  
  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
  }

  while (redLeft){
     motors.stopA();
     
     selectMuxPin(colourRightAddress);
     if (foundRed(colourRight)){
        motors.stopB();
        redLeft = false;
        break;
     }
  }
  redLeft = false;

  motors.forwardA();
}

/*
 * One motor backs up
 */
void testingBackup(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
  }
  
  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
  }

  if (redRight && redLeft){
      motors.backward();
      delay(800);
      redRight = false;
      redLeft = false;
  }

  redRight = false;
  redLeft = false;
  motors.forward();

  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.setSpeedB(baseSpeedMotorB + 50);
    motors.backwardB();
    motors.stopA();
  }

  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
    motors.stopB();
  }

  motors.forward();  
}

/*
 * 
 */
void testingJames2(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.setSpeedB(baseSpeedMotorB + 50);
    motors.backwardB();
    motors.stopA();
  }

  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
    motors.stopB();
  }

  motors.forward();
}

/*
 * Current Matthew
 */
void testingReversalSimple(){  
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.backwardB();
//    delay(25);
  }

  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
//    delay(25);
  }

  motors.forward();
}

/*
 * Turns: one wheel goes backwards
 */
void testingReversal(){  
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

//  uint16_t startTime = millis();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
      motors.setSpeedB(baseSpeedMotorB + 30);
      motors.backwardB();
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
      motors.setSpeedA(baseSpeedMotorA + 30);
      motors.backwardA();
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();  
}
