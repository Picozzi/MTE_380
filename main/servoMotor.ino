//Servo servo;
//int angle = 10;
//
//void setupServo() {
//  servo.attach(2); // pin
//  servo.write(10);
//}
//

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
