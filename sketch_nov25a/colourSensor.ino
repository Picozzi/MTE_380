void readingColours(){
  uint16_t rR, gR, bR, clearR;
  selectMuxPin(colourRightAddress);
  colourRight.getRawData(&rR, &gR, &bR, &clearR);
  Serial.println("\nRight");
  printColourInfo(rR, gR, bR);

  uint16_t rL, gL, bL, clearL;
  selectMuxPin(colourLeftAddress);
  colourLeft.getRawData(&rL, &gL, &bL, &clearL);
  Serial.println("\nLeft");
  printColourInfo(rL, gL, bL);  

  delay(2000); // For human legibility
}

void printColourInfo(uint16_t r, uint16_t g, uint16_t b) {
  Serial.println();
  Serial.print("R: ");
  Serial.print(r);
  Serial.print("\tG: ");
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
}

bool foundRed(Adafruit_TCS34725 colourSensor) {
  colourSensor.getRawData(&r, &g, &b, &clear);
  if (r>4000){
  //Serial.print("\nFound Red");
  return true;
  }
  return false;
}

bool foundRedTape(Adafruit_TCS34725 colourSensor) {
  colourSensor.getRawData(&r, &g, &b, &clear);
  if (r>3000 && b < 3000 && g < 3000){
  //Serial.print("\nFound Red");
  return true;
  }
  return false;
}

bool foundBlueTape(Adafruit_TCS34725 colourSensor) {
  colourSensor.getRawData(&r, &g, &b, &clear);
  if (r < 3000 && b > 4500 && g < 4000){
  //Serial.print("\nFound Red");
  return true;
  }
  return false;
}

bool foundGreenTape(Adafruit_TCS34725 colourSensor) {
  colourSensor.getRawData(&r, &g, &b, &clear);
  if (r < 2000 && b <3000 && g < 3000){
  //Serial.print("\nFound Red");
  return true;
  }
  return false;
}

void calibrateColour(){
  Serial.print("\nLeft");
  selectMuxPin(colourLeftAddress);
  foundRed(colourLeft);
  printColourInfo(r,g,b);

  Serial.print("\nRight");
  selectMuxPin(colourRightAddress);
  foundRed(colourRight);
  printColourInfo(r,g,b);
  delay(1000);
}
