//void setupIRSensor(){
//  // Seems that IR sensor doesn't require one
//}

/*
 * FOR TESTING
 * If there is a consistent error in measured distance, subtract/add the difference to correct it
*/

void testingIR(){
  int irDistance = SharpIR.distance();      // returns measured distance of surroundings
  Serial.print("IR Distance:\t");
  Serial.print(irDistance);

  int myDist = 0.3129*SharpIR.distance()+2.24489;
  Serial.print("\nMy IR Distance:\t");
  Serial.print(myDist);
  
}

/*
 * 
 */
unsigned long getIRDistance(){
  /*NOTE: Can ommit/comment out the code relating to acknowledging time - for testing*/
  delay(1000);
  
  unsigned long startTime = millis();
  unsigned long irDistance = 0.3129*SharpIR.distance()+2.2448;      // returns measured distance of surroundings [cm]
  Serial.print("Improved IR Distance [cm]:\t");
  Serial.print(irDistance);

  unsigned long elapsedTime = millis() - startTime;
  Serial.print("\nTime elapsed for IR reading [ms]:\t");
  Serial.print(elapsedTime);
  Serial.println();


//  // NEW code
//  int reading = analogRead(A0);
//
//  int calculated = (6762/(reading-9))-4;
//
//  Serial.println(calculated);
//
//  sprintf(cmstring, "%3d", calculated);
//  display.write("distance: ");
//  display.write(cmstring);
//  display.write("cm");
//
//  delay(2000);
  return irDistance;
}
