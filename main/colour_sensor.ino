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


/*
 * 
 */
String identifyColour(Adafruit_TCS34725 colourSensor){ 
  delay(500);
  
  // Get readings from given sensor
  uint16_t r, g, b, clear, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  colourTemp = colourSensor.calculateColorTemperature_dn40(r, g, b, clear);
  lux = colourSensor.calculateLux(r, g, b);

  delay(50);    
  printColourInfo(r, g, b, clear, lux);

  // Determine colour being read
  // +/- 3 of the vals we got on 11/08/2021
  if ((r <= 182 && r >= 176) && (g <= 37 && g >= 43) && (b <= 35 && b >= 41)){
    Serial.println("Current colour: RED");
    return "red";
  } 
  else if ((r <= 37 && r >= 31) && (g <= 85 && g >= 79) && (b <= 133 && b >= 127)){
    Serial.println("Current colour: BLUE");
    return "blue";
  } 
  else if ((r <= 55 && r >= 49) && (g <= 118 && g >= 112) && (b <= 75 && b >= 69)){
    Serial.println("Current colour: GREEN");
    return "green";
  }
  
  return "none";
}

void printColourInfo(uint16_t r, uint16_t g, uint16_t b, uint16_t clear, uint16_t lux){
  // Print read colour sensor values
  Serial.print("C: ");
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
