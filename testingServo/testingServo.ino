#include <Servo.h>              // Servo motor library /*NOTE: DISABLES PIN 9/10 FOR PWM*/

/* --- Servo Motor Defs --- */
Servo servoMotor;
#define servoPin 2

void setup() {
  Serial.begin(9600); 
  // setup servo motors
  servoMotor.attach(servoPin);
  servoMotor.write(160);
}

void loop() {
  runServo();

  closeClaw();
}

void runServo(){
  // scan from 0 to 180 degrees
  delay(500);
  servoMotor.write(180);
  delay(500);

  for(int angle = 120; angle < 180; angle++)
  {
    servoMotor.write(angle);
    delay(15);
  } 
  
  // now scan back from 180 to 0 degrees
  for(int angle = 180; angle > 120; angle--)
  {
    servoMotor.write(angle);
    delay(15);
  } 
}

void closeClaw(){
  delay(500);
  servoMotor.write(180);
  delay(500);
  
  for(int angle = 180; angle > 120; angle--)
  {
    servoMotor.write(angle);
    delay(15);
  } 
}

void openClaw(){
  delay(500);
  servoMotor.write(10);
  delay(500);
  
  for(int angle = 10; angle < 180; angle++)
  {
    servoMotor.write(angle);
    delay(15);
  } 
}
