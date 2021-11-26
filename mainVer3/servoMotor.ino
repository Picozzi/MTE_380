/*
 * Close claw gripper
 */
void closeClaw(){
  for(int i = 135; i > 40; i--)
  {
    servoMotor.write(i);
    delay(15);
  }

  servoMotor.write(40);
}

/*
 * Open claw gripper
 */
void openClaw(){
  for(int i = 40; i < 135; i++)
  {
    servoMotor.write(i);
    delay(15);
  } 

  servoMotor.write(135);
}
