/*
 * Print colour sensor values from current measurement readings
 */
void printColourInfo(uint16_t r, uint16_t g, uint16_t b) {
  Serial.println();
  Serial.print("R: ");
  Serial.print(r);
  Serial.print("\tG: ");
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
}

/*
 * Determines if given colour sensor sees RED
 */
bool foundRedTape(Adafruit_TCS34725 colourSensor) {
  colourSensor.getRawData(&r, &g, &b, &clear);
  if (r>3000 && b < 3000 && g < 3000){
    return true;
  }
  return false;
}

/*
 * Determines if given colour sensor sees BLUE
 */
bool foundBlueTape(Adafruit_TCS34725 colourSensor) {
  colourSensor.getRawData(&r, &g, &b, &clear);
  if (r < 3000 && b > 4500 && g < 4000){
    return true;
  }
  return false;
}

/*
 * Prints left and right colour sensor RGB readings.
 * Used for calibration of colour sensors to environment
 */
void calibrateColour(){
  Serial.print("\nLeft");
  selectMuxPin(colourLeftAddress);
  foundRedTape(colourLeft);
  printColourInfo(r,g,b);

  Serial.print("\nRight");
  selectMuxPin(colourRightAddress);
  foundRedTape(colourRight);
  printColourInfo(r,g,b);
  delay(1000);                       // For human legibility
}
