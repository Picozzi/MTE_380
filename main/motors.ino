void setupMotors(){
    motors.setSpeed(initialMotorSpeed);        // Set initial speed for both motors
}

void runMotors(){
  motors.forward();                            // Drive both motors forward
  printSomeInfo();
}

/*
 * TESTING: Print some informations in Serial Monitor
*/
void printSomeInfo()
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
