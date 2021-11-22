/*
 * Reset DC motors to drive at set base speed
 */
void resetMotorsSpeed(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA); 
}

/*
 * Zigzag line following with colour sensors
 */
void testingZigZag(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
  }

  while (redRight){
     motors.stopB();
            
     selectMuxPin(colourLeftAddress);
     if (foundRed(colourLeft)){
        motors.stopA();
        redRight = false;
        break;
     }
  }

  redRight = false;
  motors.forwardB();
  
  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
  }

  while (redLeft){
     motors.stopA();
     
     selectMuxPin(colourRightAddress);
     if (foundRed(colourRight)){
        motors.stopB();
        redLeft = false;
        break;
     }
  }
  redLeft = false;

  motors.forwardA();
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
