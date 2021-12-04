/*
 * Close claw gripper
 */
void closeClaw(){
  for(int i = openAngle; i > closeAngle; i--)
  {
    servo.write(i);
    delay(15);
  }

  servo.write(closeAngle);
}

/*
 * Open claw gripper
 */
void openClaw(){
  for(int i = closeAngle; i < openAngle; i++)
  {
    servo.write(i);
    delay(15);
  } 

  servo.write(openAngle);
}
