/*
 * This page is more so for testing/brainstorming
 */

uint16_t window = 500;
uint16_t count = 0;
//uint16_t countLeft = 0;

/*
 * Turns: one wheel goes backwards
 */
void testingCaitlyn(){  
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  uint16_t startTime = millis();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    
    if (count >= 2 && ((millis() - startTime) > 250)){
      motors.setSpeedB(baseSpeedMotorB + 30);
      motors.backwardB();
      count = 0;
    }
    else {
//      motors.setSpeedB(baseSpeedMotorB + 10);
      motors.stopB();
      count++;
    }
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    if (count >= 2 && ((millis() - startTime) > 250)){
      motors.setSpeedA(baseSpeedMotorA + 30);
      motors.backwardA();
      count = 0;
    }
    else {
//      motors.setSpeedA(baseSpeedMotorA + 10);
      motors.stopA();
      count++;
    }
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
}

/*
 * With b/w motors -- not the one we are using now
 */
void testingMatthew(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.forwardB();
  delay(50);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forwardA();

  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    //      Serial.println("Turning right");
    motors.setSpeedB(baseSpeedMotorB - 5);
    motors.forwardB();
    delay(50);
    motors.setSpeedA(baseSpeedMotorA + 10);
    motors.forwardA();
  }

  motors.setSpeedB(baseSpeedMotorB);
  motors.forwardB();
  delay(50);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forwardA();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    //      Serial.println("Turning right");
    motors.setSpeedB(baseSpeedMotorB + 10);
    motors.forwardB();
    delay(50);
    motors.setSpeedA(baseSpeedMotorA - 5);
    motors.forwardA();
  }
}

/*
 * Testing: one colour sensor sees red, stops a motor and vice versa
 * Right colour sensor sees red, stop Motor B (left)
 * Left colour sensor sees red, stop Motor A (right)
 */
void testingStopTurn() { // James
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.stopB();
  }

//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.stopA();
  }
}
