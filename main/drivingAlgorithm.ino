/*
 * Line following algorithm to find target and pick-up Lego man
 */
void findTarget(int speed, bool leftStart, bool rightStart, bool initial){
  blueRight = false;
  blueLeft = false;
  
  selectMuxPin(colourRightAddress);
  while (initial)
  {
    if (!(leftStart || rightStart))
    {
      forward(speed - 5, 0);
    }

    if ((foundRedTape(colourLeft) || leftStart == true) && !blueLeft)
    {
      stopLeft();
      selectMuxPin(colourRightAddress);
      delay(50);
      driveRight(speed);
      
      while(!foundRedTape(colourRight))
      {
        if (foundBlueTape(colourRight))
        {
          blueRight = true;
          break;  
        }
      }
      stopRight();
      rightStart = true;
      leftStart = false;
      delay(100);
    }

    // Blue ring from target was found
    if (blueLeft || blueRight)
    {
      bullseye(speed, blueRight, blueLeft);
      return;
    }
    
    if ((foundRedTape(colourRight) || rightStart == true) && !blueRight)
    {
      stopRight();
      selectMuxPin(colourLeftAddress);
      delay(50);
      driveLeft(speed - 5);
        
      while (!foundRedTape(colourLeft))
      {
        if (foundBlueTape(colourLeft))
        {
          blueLeft = true;
          break;  
        }
      }
      stopLeft();
      leftStart = true;
      rightStart = false;
      delay(100);
    }
  }
}

/*
 * When the bulls-eye target is reached, pick-up Lego man
 */
void bullseye(int speed, bool blueRight, bool blueLeft){
  if (blueRight)
  {
    delay(500);
    reverseRight(speed);
    delay(300);
    stopLeft();
  }
  else if (blueLeft)
  {
    delay(500);
    reverseLeft(speed);
    delay(300);
    stopRight();
  }

  forward(speed, 0);
  
  // If IR sensor sees Lego man when proximity distance threshold is reached
  if (getIRDist() < irThreshold)
  {
    stopDriving();
    closeClaw();

    // Turn robot around using IMU for 180 deg
    clockwiseTurn(speed);
    turn180();
    stopDriving();
    delay(500);
    
    forward(speed, 0);
    return;
  }
}

/*
 * Returning robot back to start location
 */
void returnTrip(int speed, bool leftStart, bool rightStart, bool initial){
  while (initial && (getIRDist() > irThreshold))
  {
    // If the start wall has been detected
    if (getIRDist() < irThreshold)
    {
      stopDriving();
      openClaw();
      stopDriving();
      return;
    }
    
    selectMuxPin(colourRightAddress);
    delay(500);
    driveRight(speed);

    if (foundRedTape(colourRight) || rightStart == true)
    {
      stopRight();
      selectMuxPin(colourLeftAddress);
      delay(50);
      driveLeft(speed - 5);
        
      while (!foundRedTape(colourLeft)){}
      stopLeft();
      leftStart = true;
      rightStart = false;
      delay(100);
    }
      
    if (foundRedTape(colourLeft) || leftStart == true)
    {
      stopLeft();
      selectMuxPin(colourRightAddress);
      delay(50);
      driveRight(speed);
      
      while (!foundRedTape(colourRight)){}
      stopRight();
      rightStart = true;
      leftStart = false;
      delay(100);
    }
  }

  // If the start wall has been detected
  if (getIRDist() < irThreshold)
  {
    stopDriving();
    openClaw();
    stopDriving();
    return;
  }
}
