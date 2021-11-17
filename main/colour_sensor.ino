/*
 * TESTING: 11/08/2021
 * CARDBOARD: R: 110, G: 77, B: 54 
 * GREEN:     R: 52, G: 115, B: 72
 * BLUE:      R: 34, G: 82, B: 130
 * RED:       R: 179, G: 40, B: 38
 * BLACK:     R: 90, G: 80, B: 74
 * WHITE:     R: 74, G: 85, B: 82
 * 
 * NOTE: Black and white slight sketch
 */

/*
 * Uses MUX, prints both colour sensor readings
 */
void testingColourSensor(Adafruit_TCS34725 colourLeft, Adafruit_TCS34725 colourRight){  
  // Sensor Left
  uint16_t r1, g1, b1, clear1, lux1;
  selectMuxPin(colourLeftAddress);
  colourLeft.getRawData(&r1, &g1, &b1, &clear1);
  lux1 = colourLeft.calculateLux(r1, g1, b1);
  Serial.println("\nLeft Colour Sensor");
  printColourInfo(r1, g1, b1, clear1, lux1);

  // Sensor Right
  uint16_t r2, g2, b2, clear2, lux2;
  selectMuxPin(colourRightAddress);
  colourRight.getRawData(&r2, &g2, &b2, &clear2);
  lux2 = colourRight.calculateLux(r2, g2, b2);
  Serial.println("\nRight Colour Sensor");
  printColourInfo(r2, g2, b2, clear2, lux2);

  // 5:30 PM Night 11/11/2021
  if ((r1 < 200) && (g1 < 400) && (b1 >= 500)){ // at night 11/10/2021 8:07 PM
    Serial.print("\tFOUND BLUE.");
  }
  else if ((r1 >= 700) && (g1 < 300) && (b1 < 300)){
    Serial.print("\tFOUND RED.");
  }

//  // 5:30 PM Night 11/11/2021
//  if ((r1 < 200 && r2 < 200) && (g1 < 400 && g2 < 400) && (b1 >= 500 && b2 >= 500)){ // at night 11/10/2021 8:07 PM
//    Serial.print("\tFOUND BLUE.");
//  }
//  else if ((r1 >= 700 && r2 >= 700) && (g1 < 300 && g2 < 300) && (b1 < 300 && b2 < 300)){
//    Serial.print("\tFOUND RED.");
//  }
  
  Serial.println("\n");

  delay(5000);
}

bool foundRed(Adafruit_TCS34725 colourSensor){  
  uint16_t r, g, b, clear;//, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  lux = colourSensor.calculateLux(r, g, b);
  
//  printColourInfo(r, g, b, clear, lux);
//  if ((r >= 700) && (g < 300) && (b < 300)){ // at night 11/10/2021 8:07 PM 
//  if ((r >= 500) && (g < 300) && (b < 300)){ // at night 11/10/2021 11:07 PM (this prob is good)
  if ((r >= 500) && (g < 300) && (b < 300)){ // cloudy afternoon 11/14/2021 3:22 PM 
    Serial.print("\nFOUND RED.");
    return true;
  } 

  return false;
}
/*
 * 
 */
bool foundBlue(Adafruit_TCS34725 colourSensor){  
  uint16_t r, g, b, clear;//, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  lux = colourSensor.calculateLux(r, g, b);
  
//  printColourInfo(r, g, b, clear, lux);
  if ((r < 200) && (g < 400) && (b >= 500)){ // at night 11/10/2021 8:07 PM    
    Serial.println("\nFOUND BLUE.");
    return true;
    // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
  }

  return false;
}

void resetMotorsSpeed(uint16_t leftMotorSpeed){
  // Reset speed
  motors.setSpeed(standardMotorSpeed);
  motors.setSpeedA(leftMotorSpeed);   
}

/*
 * Construction check demo code
 */
void constructionCheckMotors(Adafruit_TCS34725 sensor1){
  // Motors check
  uint16_t leftMotorSpeed = standardMotorSpeed + 80;
  resetMotorsSpeed(leftMotorSpeed);
  motors.forward();

  delay(3000);

  // Color sensor check
  while (foundBlue(sensor1) == false){
    motors.forward();
  }

  motors.backward();
  while (foundRed(sensor1) == false){
    motors.backward();
  }

  // Pivot
  motors.backwardB();  
  delay(1000); // FIGURE OUT HOW LONG IS NEEDED FOR SET 

  // Reset motors
  resetMotorsSpeed(leftMotorSpeed);
  motors.forward();

//  delay(3000);
//  motors.forward();

//  // U-turn
//  // Turn left
//  motors.setSpeedB(standardMotorSpeed + 20);  
//  delay(1000); // FIGURE OUT HOW LONG IS NEEDED FOR SET SPEED
//
////  // Turn left (return)
////  motors.setSpeedA(leftMotorSpeed + 20);  
////  delay(1000);
////
//  resetMotorsSpeed(leftMotorSpeed);
//  motors.forward();

//  // Color sensor check
//  while (!foundBlue(sensor1)){
//    motors.forward();
//  }
//
//  motors.stop();
//
//  delay(5000);
//
  // IR sensor check
  while(getIRDist() < 10){
    motors.forward();    
  } 
  
  Serial.println("\nEND");
  motors.stop();
  while(1){} // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
}

/*
 * Initial simple red pathfollowing code for testing
 */
void followRedLine(Adafruit_TCS34725 sensor1, Adafruit_TCS34725 sensor2, uint16_t leftMotorSpeed){
  // Timer to stop motors
  //  unsigned long startTime = millis();
  
  // Reset speed
  motors.setSpeed(standardMotorSpeed);
//  motors.setSpeedA(leftMotorSpeed);   
  motors.forward();

  // Sensor 1
  uint16_t r1, g1, b1, clear1;//, lux1;
  sensor1.getRawData(&r1, &g1, &b1, &clear1);
//  lux1 = sensor1.calculateLux(r1, g1, b1);
//  Serial.println("\nSensor1");
//  printColourInfo(r1, g1, b1, clear1, lux1);

  // Sensor 2
  uint16_t r2, g2, b2, clear2;//, lux2;
  sensor2.getRawData(&r2, &g2, &b2, &clear2);
//  lux2 = sensor2.calculateLux(r2, g2, b2);
//  Serial.println("\nSensor2");
//  printColourInfo(r2, g2, b2, clear2, lux2);

  if (foundRed(sensor1) && !foundRed(sensor2)){ // NOTE: DEFINE CONSTANTS FOR THESE COLOUR RANGES
    // Shift leftwards
//    while (foundRed(sensor1) && !foundRed(sensor2)){
       motors.stopA();
       Serial.println("Stop A");
//    }
  } 
  else if (!foundRed(sensor1) && foundRed(sensor2)){
    // Shift rightwards
//    while (!foundRed(sensor1) && foundRed(sensor2)){
       motors.stopB();
       Serial.println("Stop B");
//    }
  }
//  else if (millis() - startTime > 8000){
//    // Stop motors
//    motors.stop();
//    while(1){}
//  }
}


/*
 * 
 */
//String identifyColour(Adafruit_TCS34725 colourSensor){   
//  // Get readings from given sensor
//  uint16_t r, g, b, clear, lux;
//  colourSensor.getRawData(&r, &g, &b, &clear);
////  colourTemp = colourSensor.calculateColorTemperature_dn40(r, g, b, clear);
//  lux = colourSensor.calculateLux(r, g, b);
//
//  delay(2000);
//  printColourInfo(r, g, b, clear, lux);
//
////  delay(50);    
//  // Determine colour being read
//  // +/- 3 of the vals we got on 11/08/2021
//  if (r >= 600){
//    Serial.println("Current colour: RED");
//    return "red";
//  } 
//  else if (b <= 133 && b >= 127){
//    Serial.println("Current colour: BLUE");
//    return "blue";
//  } 
//  else if (g <= 118 && g >= 112){
//    Serial.println("Current colour: GREEN");
//    return "green";
//  } else {
//    printColourInfo(r, g, b, clear, lux);
//  }
//  
//  return "none";
//}

void printColourInfo(uint16_t r, uint16_t g, uint16_t b, uint16_t clear, uint16_t lux){
  // Print read colour sensor values
  Serial.print("\nC: ");
  Serial.print(clear);
  Serial.print("\tR: ");
  Serial.print(r);
  Serial.print("\tG: ");
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
  Serial.print("\tLux: ");
  Serial.print(lux);
  delay(3000);
}
