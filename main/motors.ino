void setupMotors(){
    motors.setSpeed(standardMotorSpeed);        // Set initial speed for both motors
}

void runMotors(){
  delay(2000);
  motors.setSpeed(255);

  /* Move forwards */
  // TESTING: One motor
//  motors.setSpeedA(255);
//  motors.forwardA();                            // Drive both motors forward

//  // Both motors 
  motors.forward();                           // Drive motor A forward
  Serial.println("Drive forward.");
  testingInfo();

  /* Move backwards */
  // TESTING: One motor
//  motors.setSpeedA(200);
//  motors.backwardA();
  
//  // Both motors 
  motors.backward();
  Serial.println("Drive backward.");
  testingInfo();

  // NOTE: deaccelerate b/w changing directions
  /* Turn around */
//  delay(2000);
//  // Motor A: increase speed
//  motors.setSpeedA(255);
//
//  // Motor B: decrease speed
//  motors.setSpeedB(127);
//
//  motors.forward();
//  testingInfo();

  /* End */
  // TESTING: One motor
//  motors.stopA();
//  delay(5000);
//  testingInfo();
//  Serial.println("Motors stopped.\n");

  
//  // Both motors
//  motors.stop();
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
