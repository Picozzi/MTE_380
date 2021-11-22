//void runServo(){
//  for(int i = 120; i < 180; i++)
//  {
//    servoMotor.write(i);
//    delay(15);
//  } 
//  
//  for(int i = 180; i > 120; i--)
//  {
//    servoMotor.write(i);
//    delay(15);
//  } 
//}

/*
 * Close claw gripper
 */
void closeClaw(){
  for(int i = 180; i > 100; i--)
  {
    servoMotor.write(i);
    delay(15);
  }

  while(1){}
}

/*
 * Open claw gripper
 */
void openClaw(){
  for(int i = 10; i < 180; i++)
  {
    servoMotor.write(i);
    delay(15);
  } 
}

/*
 * Pick up man
 * Uses blue colour sensing to determine target
 * 
 * NOTE: Obsolete code
 */
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
