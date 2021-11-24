/*
 * Reset DC motors to drive at set base speed
 */
void resetMotorsSpeed(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA); 
}

void pickUpLegoMan(){
   // If IR sensor sees Lego man or timed threshold is reached
//  if ((getIRDist() < irLegoThreshold || redLeft) || (millis() - timer) > targetTimeThreshold) // NOT SURE IF WANT TO INCLUDE THE RED COLOUR CHECK, HENCE "OR"
  if (getIRDist() < irLegoThreshold) // NOT SURE IF WANT TO INCLUDE THE RED COLOUR CHECK, HENCE "OR"
  {
    motors.stop();
    closeClaw(); // Grip man

    // Reset R/L colour sensor flags
    redRight = false;
    redLeft = false;
    blueRight = false;
    blueLeft = false;
    greenRight = false;
    greenLeft = false;

    while(1){}

//    case1 = false;
//    case3 = true;
    //        case2 = true; // DROPPING OFF AT GREEN SAFE ZONE
    return; // in final version
  }
}

bool redRightMutex = false;
bool redLeftMutex = false;

/*
 * Attempt 2
 * Zigzag line following with colour sensors
 */
void zigZag2(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);

  selectMuxPin(colourRightAddress);
  if (foundBlue(colourRight)){
    blueRight = true;

    motors.stop();
    // pivot 10 deg maybe    
    // motors.forwardA();
    // motors.backwardB();
    // turn10();
    
    while(1){}
    // go to the next case: IR polling and closing claw
//    pickUpLegoMan();
  }

  selectMuxPin(colourLeftAddress);
  if (foundBlue(colourLeft)){
    blueLeft = true;

    motors.stop();
    // pivot 10 deg maybe    
    // motors.forwardB();
    // motors.backwardA();
    // turn10();

    while(1){}
    // go to the next case: IR polling and closing claw
//    pickUpLegoManLegoMan();
  }

  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight)){
     motors.stopB();
            
     selectMuxPin(colourLeftAddress);
     if (foundRed(colourLeft)){
        motors.stopA();
        break;
     }
  }

  motors.forwardB();
  
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft)){
     motors.stopA();
     
     selectMuxPin(colourRightAddress);
     if (foundRed(colourRight)){
        motors.stopB();
        break;
     }
  }

  motors.forwardA();
}

/*
 * Attempt 1: Waddle Walk
 * Zigzag line following with colour sensors
 * Includes mutexes
 */
void zigZag1(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
    redRightMutex = true;
  }

  while (redRight && !redLeftMutex){
     motors.stopB();
            
     selectMuxPin(colourLeftAddress);
     if (foundRed(colourLeft)){
        motors.stopA();
        redRight = false;
        redRightMutex = false;
        break;
     }
  }

  redRight = false;
  motors.forwardB();
  
  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
    redLeftMutex = true;
  }

  while (redLeft && !redRightMutex){
     motors.stopA();
     
     selectMuxPin(colourRightAddress);
     if (foundRed(colourRight)){
        motors.stopB();
        redLeft = false;
        redLeftMutex = false;
        break;
     }
  }
  redLeft = false;

  motors.forwardA();
}

/*
 * Very first initial zig zag idea that works-ish
 * Zigzag line following with colour sensors
 */
void zigZagBase(){
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
