#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)


/* --- Motor Controller Defs --- */
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

#define lowestMotorSpeed 150        // Standard motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed
#define baseSpeedMotorA 220

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B); // RENAME TO dcMotors

void setup() {
  motors.setSpeed(standardMotorSpeed);        // Set initial speed for both motors
}

void loop() {
//  motors.setSpeedA(standardMotorSpeed + 35);
//  motors.setSpeedA(220); // offset for motor A is by 70
  motors.setSpeedA(standardMotorSpeed + 60); // offset for motor A is by 70
  motors.forward();
}
