//void setupIRSensor(){
//  // Seems that IR sensor doesn't require one
//}

/*
 * FOR TESTING
 * If there is a consistent error in measured distance, subtract/add the difference to correct it
*/

void runIRSensor(){
  /*NOTE: Can ommit/comment out the code relating to acknowledging time - for testing*/
  unsigned long startTime = millis();
  int irDistance = SharpIR.distance();      // returns measured distance of surroundings
  Serial.print("IR Distance:\t");
  Serial.print(irDistance);

  unsigned long elapsedTime = millis() - startTime;
  Serial.print("\nTime elapsed for IR reading [ms]:\t");
  Serial.print(elapsedTime);
  Serial.println();
}
