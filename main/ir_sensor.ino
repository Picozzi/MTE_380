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

  int myDist = 4.3457*irDistance - 1.3399;
  Serial.print("\nMy IR Distance:\t");
  Serial.print(myDist);
  
}

void runIRSensor(){
  /*NOTE: Can ommit/comment out the code relating to acknowledging time - for testing*/
  delay(1000);
  
  unsigned long startTime = millis();
  int irDistance = 0.3129*SharpIR.distance()+2.2448;      // returns measured distance of surroundings
  Serial.print("New IR Distance:\t");
  Serial.print(irDistance);
  

//  int myDist = 4.3457*irDistance - 1.3399;
//  Serial.print("\nMy IR Distance:\t");
//  Serial.print(myDist);

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

}
