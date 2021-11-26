// This page is for testing modular functions to ensure I/O devices are properly working

/*
 * Checking DC motors
 */
void dcMotorCheck(){
  motors.setSpeedB(200);
  motors.setSpeedA(210);
//  motors.setSpeed(120);
  motors.forward();
}

/*
 * Determines which colour sensor address is for which through testing.
 * 
 * Uncomment readColours() in foundRed() to print colour sensor readings 
 * for the right colour sensor.
 */
void isRightColourSensor() { 
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
  
  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)) 
  {
    Serial.println("\nRight");
//    motors.stop();
    while (1) {}
  }
}

/*
 * Checking IR promixity sensor
 */
void irCheck(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  while (getIRDist() < 11){
    Serial.println("\nREACHED THRESHOLD");
    motors.stop();
    delay(2000);
  }
}

/*
 * Checking servo motor
 */
void servoCheck(){
  openClaw();

  delay(2000);

  closeClaw();
}

/*
 * TEST THIS ONE
 * Checking if IMU readings can turn robot 180 deg (around)
 */
void imuCheck(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forwardB();
  motors.backwardA();
  
  turn180();

  motors.stop();
  delay(2000);
}
