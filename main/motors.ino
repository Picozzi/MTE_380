void setupMotors(){
    motors.setSpeed(standardMotorSpeed);        // Set initial speed for both motors
}

// TESTING
void turnLeft(){
  // Both motors 
  motors.forward();
  motors.setSpeedA(120);
  motors.backward();
   
//  Serial.println("Drive forward.");
//  testingInfo();

  /* Move backwards */
  // TESTING: One motor
//  motors.setSpeedA(200);
//  motors.backwardA();
  
//  // Both motors 
//  motors.backward();
//  delay(2000);

//  Serial.println("Drive backward.");
//  testingInfo();

  // NOTE: deaccelerate b/w changing directions
  /* Turn around */
//  // Turn left
//  // Motor A: increase speed
//  motors.setSpeedA(220);
//
//  // Motor B: decrease speed
//  motors.setSpeedB(200);
//  delay(2000);
//
//  motors.setSpeed(255);
//  motors.forward();
//  testingInfo();

  /* End */
  // TESTING: One motor
//  motors.stopA();
//  delay(5000);
//  testingInfo();
//  Serial.println("Motors stopped.\n");

  
  // Both motors
//  delay(5000);
//  motors.stop();
}


void runMotors(){
//  delay(2000);
//  motors.setSpeed(255);

  /* Move forwards */
  // TESTING: One motor
//  motors.setSpeedA(255);
//  motors.forwardA();                            // Drive both motors forward

//  // Both motors 
//  motors.forward();
//  delay(2000);
  motors.setSpeedA(120);
  motors.backward();
   
//  Serial.println("Drive forward.");
//  testingInfo();

  /* Move backwards */
  // TESTING: One motor
//  motors.setSpeedA(200);
//  motors.backwardA();
  
//  // Both motors 
//  motors.backward();
//  delay(2000);

//  Serial.println("Drive backward.");
//  testingInfo();

  // NOTE: deaccelerate b/w changing directions
  /* Turn around */
//  // Turn left
//  // Motor A: increase speed
//  motors.setSpeedA(220);
//
//  // Motor B: decrease speed
//  motors.setSpeedB(200);
//  delay(2000);
//
//  motors.setSpeed(255);
//  motors.forward();
//  testingInfo();

  /* End */
  // TESTING: One motor
//  motors.stopA();
//  delay(5000);
//  testingInfo();
//  Serial.println("Motors stopped.\n");

  
  // Both motors
//  delay(5000);
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
