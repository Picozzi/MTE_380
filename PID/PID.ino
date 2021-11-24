// Libraries
#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <L298NX2.h>            // DC Motor drive controller library (powers 2 motors)
#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PINS 9 AND 10 FOR PWM*/
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
#define irStartThreshold 15   // CHANGE THROUGH TESTING
// 5 V


/* --- IMU Sensor Defs --- */
Adafruit_ICM20948 imu;

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
#define motorLAMinSpeed 150        // Lowest motor driving speed
#define motorRBMinSpeed 150      // Standard motor driving speed
#define motorLAMaxSpeed 255 // Left Motor A Max Speed
#define motorRBMaxSpeed 255 // Right Motor B Max Speed

uint16_t motorRBBaseSpeed = 110;         // Base motor B driving speed
uint16_t motorLABaseSpeed = 125;         // Base motor A driving speed

#define baseRed 470
#define KP 2.1
#define KD 0.85

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B);

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

}

int lastError = 0;
int derivative;
int integral;

void loop(void) {  
  uint16_t rR, gR, bR, clearR;
  selectMuxPin(colourRightAddress);
  colourRight.getRawData(&rR, &gR, &bR, &clearR);
  
  uint16_t rL, gL, bL, clearL;
  selectMuxPin(colourLeftAddress);
  colourLeft.getRawData(&rL, &gL, &bL, &clearL);

  int error = (rL - baseRed) - (rR - baseRed);
  error = error / 100;
  Serial.println(error);

//  Serial.println("Right");
//  Serial.println(rL);
//  Serial.println("Left");
//  Serial.println(rR);
  
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  Serial.println(motorSpeed);
  
  int LAMotorSpeed = motorLABaseSpeed - motorSpeed; //motorA left
  int RBMotorSpeed = motorRBBaseSpeed + motorSpeed; //motorB right
  Serial.println(motorSpeed);
  // set motor speeds using the two motor speed variables above
  set_motors(LAMotorSpeed, RBMotorSpeed);
}

void set_motors(int leftMotor, int rightMotor)
{
  if (leftMotor > motorLAMaxSpeed){ 
    leftMotor = motorLAMaxSpeed; // limit top speed
  }
  
  if (rightMotor > motorRBMaxSpeed){
    rightMotor = motorRBMaxSpeed; // limit top speed
  }
  
  if (leftMotor < 0){
    leftMotor = 0; // keep motor above 0
  }
  
  if (rightMotor < 0){
    rightMotor = 0; // keep motor speed above 0
  }
  
  motors.setSpeedA(leftMotor);
  motors.setSpeedB(rightMotor);
  motors.forward();  
}


/*
 * Select desired channel pin on 12C address using multiplexer
 */
void selectMuxPin(uint8_t channelPin){
  if (channelPin > 7){
    return;
  }

  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channelPin);
  Wire.endTransmission();
}

void readingColours(){
  uint16_t rR, gR, bR, clearR;
  selectMuxPin(colourRightAddress);
  colourRight.getRawData(&rR, &gR, &bR, &clearR);
  Serial.println("\nRight");
  printColourInfo(rR, gR, bR, clearR);

  uint16_t rL, gL, bL, clearL;
  selectMuxPin(colourLeftAddress);
  colourLeft.getRawData(&rL, &gL, &bL, &clearL);
  Serial.println("\nLeft");
  printColourInfo(rL, gL, bL, clearL);  

  delay(2000); // For human legibility
}

void printColourInfo(uint16_t r, uint16_t g, uint16_t b, uint16_t clear) {
  Serial.print("\nC: ");
  Serial.print(clear);
  Serial.print("\tR: ");
  Serial.print(r);
  Serial.print("\tG: ");
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
}
