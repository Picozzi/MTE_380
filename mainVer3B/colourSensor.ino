/*
 * 11/08/2021
 * CARDBOARD: R: 110, G: 77,  B: 54 
 * GREEN:     R: 52,  G: 115, B: 72
 * BLUE:      R: 34,  G: 82,  B: 130
 * RED:       R: 179, G: 40,  B: 38
 * BLACK:     R: 90,  G: 80,  B: 74
 * WHITE:     R: 74,  G: 85,  B: 82
 * 
 * NOTE: Black and white slight sketch
 */

/*
 * 11/19/2021
 * Church Lab Environment (Minimum readings with rounded buffer - can lower threshold)
 * CORK:    R: 4000, G: 3970, B: 3074
 * GREEN:   R: 2000, G: 3600, B: 1700
 * RED:     R: 2800, G: 400,  B: 600
 * BLUE:    R: 500,  G: 1800, B: 3000
 */

/*
 * Reading colours to COM4
 * Used for calibration of colour sensors
 */
void readingColours(){
  uint16_t rR, gR, bR, clearR;
  selectMuxPin(colourRightAddress);
  colourRight.getRawData(&rR, &gR, &bR, &clearR);
  Serial.println("\nRight");
  printColourInfo(rR, gR, bR, clearR);

  uint16_t rL, gL, bL, clearL;
  selectMuxPin(colourLeftAddress);
  colourLeft.getRawData(&rL, &gL, &bL, &clearL);
  Serial.println("\nLeft");
  printColourInfo(rL, gL, bL, clearL);  

  delay(2000); // For human legibility
}

/*
 * Determines if given colour sensor sees RED
 * Mainly for line following
 */
bool foundRed(Adafruit_TCS34725 colourSensor) {
  uint16_t r, g, b, clear;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  printColourInfo(r, g, b, clear);

  //  if ((r >= 700) && (g < 300) && (b < 300)){ // at night 11/10/2021 8:07 PM
  //  if ((r >= 500) && (g < 300) && (b < 300)){ // at night 11/10/2021 11:07 PM
  //  if ((r >= 500) && (g < 300) && (b < 300)){ // cloudy afternoon 11/14/2021 3:22 PM
  //  if ((r >= 2000) && (g < 1000) && (b < 1000)) { // night 11/18/2021 6:42 PM cardboard
  //  if ((r >= 1000)) { // night 11/19/2021 - quick and dirty 
//  if (r >= 2000){ // afternoon 11/21/2021 3:45 PM - quick and dirty
  if (r >= 1800 && b < 1000 && g < 1000){ // 11/23/2021 night - quick and dirty
    Serial.print("\nFOUND RED.");
    return true;
  }

  return false;
}

/*
 * Determines if given colour sensor sees BLUE
 * Mainly for finding target
 */
bool foundBlue(Adafruit_TCS34725 colourSensor) {
  uint16_t r, g, b, clear;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  printColourInfo(r, g, b, clear);

  //  if ((r < 200) && (g < 400) && (b >= 500)){ // at night 11/10/2021 8:07 PM
  //  if ((r < 600) && (g < 1400) && (b >= 2000)) { // at night 11/19/2021 6:43 PM
//  if (b >= 2000){ // afternoon 11/21/2021 3:45 PM - quick and dirty
  if (b > 1500 && g < 1500 && r < 1000){ // 11/23/2021 night
    Serial.println("\nFOUND BLUE.");
    return true;
    // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
  }

  return false;
}

/*
 * Determines if given colour sensor sees GREEN
 * Mainly for finding safe zone
 */
bool foundGreen(Adafruit_TCS34725 colourSensor) {
  uint16_t r, g, b, clear;
  colourSensor.getRawData(&r, &g, &b, &clear);
//  printColourInfo(r, g, b, clear);

  if (g >= 2000){
    Serial.println("\nFOUND GREEN.");
    return true;
  }

  return false;
}

/*
 * Print colour sensor values from current measurement readings
 */
void printColourInfo(uint16_t r, uint16_t g, uint16_t b, uint16_t clear) {
  Serial.print("\nC: ");
  Serial.print(clear);
  Serial.print("\tR: ");
  Serial.print(r);
  Serial.print("\tG: ");
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
}
