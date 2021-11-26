/*
 * Get IR promixity sensor's current distance reading
 */
unsigned long getIRDist(){
  unsigned long irDist = 0.3129*SharpIR.distance()+2.24489;
  Serial.print("\nMy IR Distance:\t");
  Serial.print(irDist);

  return irDist;
}


/*
 * FOR TESTING
 * If there is a consistent error in measured distance, subtract/add the difference to correct it
*/

void testingIR(){  
  motors.forward();
  
  int myDist = 0.3129*SharpIR.distance()+2.24489;
  Serial.print("\nMy IR Distance:\t");
  Serial.print(myDist);

  if (myDist < 10){ // at night 11/10/2021 8:07 PM
    Serial.println("\nREACHED THRESHOLD");
    motors.stop();
    while(1){}
  } 
}
