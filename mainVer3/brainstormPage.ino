///*
// * This page is more so for testing/brainstorming
// */
//
///*
// * Construction check demo code
// */
//void constructionCheckMotors(Adafruit_TCS34725 sensor1){
//  // Motors check
//  resetMotorsSpeed();
//  motors.forward();
//
//  delay(3000);
//
//  // Color sensor check
//  while (foundBlue(sensor1) == false){
//    motors.forward();
//  }
//
//  motors.backward();
//  while (foundRed(sensor1) == false){
//    motors.backward();
//  }
//
//  // Pivot
//  motors.backwardB();  
//  delay(1000); // FIGURE OUT HOW LONG IS NEEDED FOR SET 
//
//  // Reset motors
//  resetMotorsSpeed();
//  motors.forward();
//
////  delay(3000);
////  motors.forward();
//
////  // U-turn
////  // Turn left
////  motors.setSpeedB(standardMotorSpeed + 20);  
////  delay(1000); // FIGURE OUT HOW LONG IS NEEDED FOR SET SPEED
////
//////  // Turn left (return)
//////  motors.setSpeedA(leftMotorSpeed + 20);  
//////  delay(1000);
//////
////  resetMotorsSpeed(leftMotorSpeed);
////  motors.forward();
//
////  // Color sensor check
////  while (!foundBlue(sensor1)){
////    motors.forward();
////  }
////
////  motors.stop();
////
////  delay(5000);
////
//  // IR sensor check
//  while(getIRDist() < 10){
//    motors.forward();    
//  } 
//  
//  Serial.println("\nEND");
//  motors.stop();
//  while(1){} // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
//}
//
///*
// * Initial simple red pathfollowing code for testing
// */
//void followRedLine(Adafruit_TCS34725 sensor1, Adafruit_TCS34725 sensor2, uint16_t leftMotorSpeed){
//  // Timer to stop motors
//  //  unsigned long startTime = millis();
//  
//  // Reset speed
//  motors.setSpeed(standardMotorSpeed);
////  motors.setSpeedA(leftMotorSpeed);   
//  motors.forward();
//
//  // Sensor 1
//  uint16_t r1, g1, b1, clear1;//, lux1;
//  sensor1.getRawData(&r1, &g1, &b1, &clear1);
////  lux1 = sensor1.calculateLux(r1, g1, b1);
////  Serial.println("\nSensor1");
////  printColourInfo(r1, g1, b1, clear1, lux1);
//
//  // Sensor 2
//  uint16_t r2, g2, b2, clear2;//, lux2;
//  sensor2.getRawData(&r2, &g2, &b2, &clear2);
////  lux2 = sensor2.calculateLux(r2, g2, b2);
////  Serial.println("\nSensor2");
////  printColourInfo(r2, g2, b2, clear2, lux2);
//
//  if (foundRed(sensor1) && !foundRed(sensor2)){ // NOTE: DEFINE CONSTANTS FOR THESE COLOUR RANGES
//    // Shift leftwards
////    while (foundRed(sensor1) && !foundRed(sensor2)){
//       motors.stopA();
//       Serial.println("Stop A");
////    }
//  } 
//  else if (!foundRed(sensor1) && foundRed(sensor2)){
//    // Shift rightwards
////    while (!foundRed(sensor1) && foundRed(sensor2)){
//       motors.stopB();
//       Serial.println("Stop B");
////    }
//  }
////  else if (millis() - startTime > 8000){
////    // Stop motors
////    motors.stop();
////    while(1){}
////  }
//}
//
//// --- Line following ---
///*
// * One motor backs up
// */
//void testingBackup(){
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
//  selectMuxPin(colourRightAddress);
//  if (foundRed(colourRight)){
//    redRight = true;
//  }
//  
//  selectMuxPin(colourLeftAddress);
//  if (foundRed(colourLeft)){
//    redLeft = true;
//  }
//
//  if (redRight && redLeft){
//      motors.backward();
//      delay(800);
//      redRight = false;
//      redLeft = false;
//  }
//
//  redRight = false;
//  redLeft = false;
//  motors.forward();
//
//  // Shift right
//  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//    motors.setSpeedB(baseSpeedMotorB + 50);
//    motors.backwardB();
//    motors.stopA();
//  }
//
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
//  // Shift left
//  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//    motors.backwardA();
//    motors.stopB();
//  }
//
//  motors.forward();  
//}
//
///*
// * 
// */
//void testingJames2(){
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//  
//  // Shift right
//  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//    motors.setSpeedB(baseSpeedMotorB + 50);
//    motors.backwardB();
//    motors.stopA();
//  }
//
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
//  // Shift left
//  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//    motors.backwardA();
//    motors.stopB();
//  }
//
//  motors.forward();
//}
//
///*
// * Current Matthew
// */
//void testingReversalSimple(){  
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//  
//  // Shift right
//  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//    motors.backwardB();
////    delay(25);
//  }
//
//  motors.forward();
//
//  // Shift left
//  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//    motors.backwardA();
////    delay(25);
//  }
//
//  motors.forward();
//}
//
///*
// * Turns: one wheel goes backwards
// */
//void testingReversal(){  
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
////  uint16_t startTime = millis();
//  
//  // Shift right
////  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//      motors.setSpeedB(baseSpeedMotorB + 30);
//      motors.backwardB();
//  }
//
//  motors.forward();
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
//  // Shift left
////  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//      motors.setSpeedA(baseSpeedMotorA + 30);
//      motors.backwardA();
//  }
//
//  motors.forward();
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();  
//}
//
//
//uint16_t window = 500;
//uint16_t count = 0;
////uint16_t countLeft = 0;
//
///*
// * Turns: one wheel goes backwards
// */
//void testingCaitlyn(){  
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
//  uint16_t startTime = millis();
//  
//  // Shift right
////  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//    
//    if (count >= 2 && ((millis() - startTime) > 250)){
//      motors.setSpeedB(baseSpeedMotorB + 30);
//      motors.backwardB();
//      count = 0;
//    }
//    else {
////      motors.setSpeedB(baseSpeedMotorB + 10);
//      motors.stopB();
//      count++;
//    }
//  }
//
//  motors.forward();
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//
//  // Shift left
////  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//    if (count >= 2 && ((millis() - startTime) > 250)){
//      motors.setSpeedA(baseSpeedMotorA + 30);
//      motors.backwardA();
//      count = 0;
//    }
//    else {
////      motors.setSpeedA(baseSpeedMotorA + 10);
//      motors.stopA();
//      count++;
//    }
//  }
//
//  motors.forward();
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//}
//
///*
// * With b/w motors -- not the one we are using now
// */
//void testingMatthew(){
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.forwardB();
//  delay(50);
//  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
//  motors.forwardA();
//
//  // Shift right
////  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//    //      Serial.println("Turning right");
//    motors.setSpeedB(baseSpeedMotorB - 5);
//    motors.forwardB();
//    delay(50);
//    motors.setSpeedA(baseSpeedMotorA + 10);
//    motors.forwardA();
//  }
//
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.forwardB();
//  delay(50);
//  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
//  motors.forwardA();
//
//  // Shift left
////  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//    //      Serial.println("Turning right");
//    motors.setSpeedB(baseSpeedMotorB + 10);
//    motors.forwardB();
//    delay(50);
//    motors.setSpeedA(baseSpeedMotorA - 5);
//    motors.forwardA();
//  }
//}
//
///*
// * Testing: one colour sensor sees red, stops a motor and vice versa
// * Right colour sensor sees red, stop Motor B (left)
// * Left colour sensor sees red, stop Motor A (right)
// */
//void testingStopTurn() { // James
//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();
//  
//  // Shift right
////  selectMuxPin(colourRightAddress);
//  while (foundRed(colourRight))
//  {
//    motors.stopB();
//  }
//
////  motors.setSpeedB(baseSpeedMotorB);
////  motors.setSpeedA(baseSpeedMotorA);
////  motors.forward();
//
//  // Shift left
////  selectMuxPin(colourLeftAddress);
//  while (foundRed(colourLeft))
//  {
//    motors.stopA();
//  }
//}
