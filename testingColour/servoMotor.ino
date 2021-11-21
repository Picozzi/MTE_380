void runServo(){
  // scan from 0 to 180 degrees
//  delay(500);
//  servoMotor.write(180);
//  delay(500);

  for(int i = 120; i < 180; i++)
  {
    servoMotor.write(i);
    delay(15);
  } 
  
  // now scan back from 180 to 0 degrees
  for(int i = 180; i > 120; i--)
  {
    servoMotor.write(i);
    delay(15);
  } 
}

void closeClaw(){
//  delay(500);
//  servoMotor.write(180);
//  delay(500);
  
  for(int i = 180; i > 100; i--)
  {
    servoMotor.write(i);
    delay(15);
  }

  while(1){}
}

void openClaw(){
//  delay(500);
//  servoMotor.write(10);
//  delay(500);
  
  for(int i = 10; i < 180; i++)
  {
    servoMotor.write(i);
    delay(15);
  } 
}

void gripMan(){
  selectMuxPin(colourRightAddress);
  if (foundBlue(colourRight))
  {
    blueRight = true;
  }

  selectMuxPin(colourLeftAddress);
  if (foundBlue(colourLeft))
  {
    blueLeft = true;
  }

  if (!blueRight && !blueLeft)
  {
     if (blueRight)
     {
       motors.stopB();
     } 
     else if (blueLeft)
     {
       motors.stopA();
     }    
  }
  else {
    while(1){}
  }
}
