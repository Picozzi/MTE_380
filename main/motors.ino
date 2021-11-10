void setupMotors(){
    motors.setSpeed(standardMotorSpeed);        // Set initial speed for both motors
}

void runMotors(){
//  motors.forward();                            // Drive both motors forward
  motors.forwardA();                           // Drive motor A forward
  testingInfo();

  delay(3000);
  motors.stopA();
  testingInfo();

  motors.setSpeedA(255);
  motors.backwardA();
  testingInfo();

  delay(3000);
  motors.forwardA();
  testingInfo();
}

/*
 * TESTING: Print some informations in Serial Monitor
*/
void testingInfo()
{
  Serial.print("Motor A is moving = ");
  Serial.print(motors.isMovingA() ? "YES" : "NO");
  Serial.print(" at speed = ");
  Serial.println(motors.getSpeedA());
  Serial.print("Motor B is moving = ");
  Serial.print(motors.isMovingB() ? "YES" : "NO");
  Serial.print(" at speed = ");
  Serial.println(motors.getSpeedB());
}
