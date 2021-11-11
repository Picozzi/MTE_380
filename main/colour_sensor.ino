//void setupColourSensor(){
//  if (colourSensor.begin()){
//    // If the sensor starts up correctly
//    Serial.println("Found colour sensor!");
//  } else {
//    Serial.println("The colour sensor was not found...");
//    // while (1); // pause
//  }
//
//  // Set pin outputs
////  pinMode(redPin, OUTPUT);
////  pinMode(greenPin, OUTPUT);
////  pinMode(bluePin, OUTPUT);
//
//  for (int i=0; i<256; i++){
//    float x = i;
//    x /= 255;
//    x = pow(x, 2.5);
//    x *= 255;
//
//    if (commonAnode){
//      gammaTable[i] = 255 - x;  
//    } else {
//      gammaTable[i] = x;
//    }
//  }
// }

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

void stopAtRed(Adafruit_TCS34725 colourSensor){
  runMotors();
  
  uint16_t r, g, b, clear, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
  lux = colourSensor.calculateLux(r, g, b);
  
//  delay(2000);
  printColourInfo(r, g, b, clear, lux);
  if (r >= 700){ // at night 11/10/2021 7:54 PM
    Serial.println("\nFOUND RED. STOP");
    motors.stop();
    while(1){}
    // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
  } 
}

void stopAtBlue(Adafruit_TCS34725 colourSensor){
  runMotors();
  
  uint16_t r, g, b, clear, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
  lux = colourSensor.calculateLux(r, g, b);
  
//  delay(2000);
  printColourInfo(r, g, b, clear, lux);
//  delay(2000);
  if (b >= 400){ // at night 11/10/2021 8:07 PM
    Serial.println("\nFOUND BLUE. STOP");
    motors.stop();
    while(1){}
    // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
  } 
}


/*
 * 
 */
String identifyColour(Adafruit_TCS34725 colourSensor){   
  // Get readings from given sensor
  uint16_t r, g, b, clear, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  colourTemp = colourSensor.calculateColorTemperature_dn40(r, g, b, clear);
  lux = colourSensor.calculateLux(r, g, b);

  delay(2000);
  printColourInfo(r, g, b, clear, lux);

//  delay(50);    
  // Determine colour being read
  // +/- 3 of the vals we got on 11/08/2021
  if (r >= 600){
    Serial.println("Current colour: RED");
    return "red";
  } 
  else if (b <= 133 && b >= 127){
    Serial.println("Current colour: BLUE");
    return "blue";
  } 
  else if (g <= 118 && g >= 112){
    Serial.println("Current colour: GREEN");
    return "green";
  } else {
    printColourInfo(r, g, b, clear, lux);
  }
  
  return "none";
}

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
}
