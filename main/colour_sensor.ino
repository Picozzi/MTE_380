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


void runColourSensor(){
  uint16_t clear, red, green, blue, colourTemp, lux;

//  colourSensor.setInterrupt(false);                       // turn on LED
  delay(500);                                            // wait 1000 ms for reading input
  colourSensor.getRawData(&red, &green, &blue, &clear);   // read sensor input
  colourTemp = colourSensor.calculateColorTemperature_dn40(red, green, blue, clear);
  lux = colourSensor.calculateLux(red, green, blue);
//  colourSensor.setInterrupt(true);                        // turn off LED

  // Print read colour sensor values
  Serial.print("C: ");
  Serial.print(clear);
  Serial.print("\tR: ");
  Serial.print(red);
  Serial.print("\tG: ");
  Serial.print(green);
  Serial.print("\tB: ");
  Serial.print(blue);
  Serial.print("\tLux: ");
  Serial.print(lux);
}
