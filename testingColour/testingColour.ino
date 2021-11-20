#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)

/* --- Colour Sensor --- */
Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // NOTE: CAN PROBABLY CHANGE THE INPUT PARAMS
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

bool redRight = false;
bool redLeft = false;

/* --- MUX Defs --- */
#define muxAddress 0x70           // Multiplexer address for I2C
#define colourRightAddress 0
#define colourLeftAddress 1

/* --- Motor Controller Defs --- */
// Motor A: left
// Motor B: right
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

#define lowestMotorSpeed 150        // Lowest motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed
//#define baseSpeedMotorA 224         // Base motor A driving speed
//#define baseSpeedMotorB 150         // Base motor B driving speed

int baseSpeedMotorB = 140;
int baseSpeedMotorA = baseSpeedMotorB + 10;         // Base motor A driving speed

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B); // RENAME TO dcMotors

void setup() {
  Serial.begin(9600);

  Wire.begin();

  // Setup colour sensors
  selectMuxPin(colourLeftAddress);
  if (colourLeft.begin()) {
    // If the sensor starts up correctly
    Serial.println("Found left colour sensor!");
  }
  else {
    Serial.println("Left colour sensor was not found.");
  }

  selectMuxPin(colourRightAddress);
  if (colourRight.begin()) {
    // If the sensor starts up correctly
    Serial.println("Found right colour sensor!");
  }
  else {
    Serial.println("Right colour sensor was not found.");
  }
  
  // setup DC motors (front wheels)
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward(); // for zigZag function
}

void loop() {
//  testingReversalSimple();  
//  testingHardTurn();
//  testingStopTurn();
//  testingJames2();
  testingZigZag();

//  testingBackup();

  // Game day environment colour sensor calibration
  // cork: R: 4000, G: 3970,, B: 3074
  // green: 2717 (2409) (2560) 2261, G: 3655 (3805) (4145), B: 2210 (1900) (2040) 1770
  // red: 2919-20 (2904), G: 830 (780) 429 , B: 827 (790) 653
  // blue: R: 697 (622), G: 2069 (1970), B: 3449 (3291)
//  readingColours();
}

void testingZigZag(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
  }

  while (redRight){
    motors.stopB();
     selectMuxPin(colourLeftAddress);
     if (foundRed(colourLeft)){
        motors.stopA();
        redRight = false;
        break;
     }
  }

  redRight = false;
  motors.forwardB();
  
  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
  }

  while (redLeft){
    motors.stopA();
     selectMuxPin(colourRightAddress);
     if (foundRed(colourRight)){
        motors.stopB();
        redLeft = false;
        break;
     }
  }
  redLeft = false;

  motors.forwardA();
}

/*
 * One motor backs up
 */
void testingBackup(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)){
    redRight = true;
  }
  
  selectMuxPin(colourLeftAddress);
  if (foundRed(colourLeft)){
    redLeft = true;
  }

  if (redRight && redLeft){
      motors.backward();
      delay(800);
      redRight = false;
      redLeft = false;
  }

  redRight = false;
  redLeft = false;
  motors.forward();

  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.setSpeedB(baseSpeedMotorB + 50);
    motors.backwardB();
    motors.stopA();
  }

  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
    motors.stopB();
  }

  motors.forward();  
}

/*
 * 
 */
void testingJames2(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.setSpeedB(baseSpeedMotorB + 50);
    motors.backwardB();
    motors.stopA();
  }

  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
    motors.stopB();
  }

  motors.forward();
}

/*
 * Current Matthew
 */
void testingReversalSimple(){  
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.backwardB();
//    delay(25);
  }

  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
//    delay(25);
  }

  motors.forward();
}

/*
 * Turns: one wheel goes backwards
 */
void testingReversal(){  
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

//  uint16_t startTime = millis();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
      motors.setSpeedB(baseSpeedMotorB + 30);
      motors.backwardB();
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
      motors.setSpeedA(baseSpeedMotorA + 30);
      motors.backwardA();
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();  
}

uint16_t window = 500;
uint16_t count = 0;
//uint16_t countLeft = 0;

/*
 * Turns: one wheel goes backwards
 */
void testingCaitlyn(){  
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  uint16_t startTime = millis();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    
    if (count >= 2 && ((millis() - startTime) > 250)){
      motors.setSpeedB(baseSpeedMotorB + 30);
      motors.backwardB();
      count = 0;
    }
    else {
//      motors.setSpeedB(baseSpeedMotorB + 10);
      motors.stopB();
      count++;
    }
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    if (count >= 2 && ((millis() - startTime) > 250)){
      motors.setSpeedA(baseSpeedMotorA + 30);
      motors.backwardA();
      count = 0;
    }
    else {
//      motors.setSpeedA(baseSpeedMotorA + 10);
      motors.stopA();
      count++;
    }
  }

  motors.forward();
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
}

/*
 * With b/w motors -- not the one we are using now
 */
void testingMatthew(){
  motors.setSpeedB(baseSpeedMotorB);
  motors.forwardB();
  delay(50);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forwardA();

  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    //      Serial.println("Turning right");
    motors.setSpeedB(baseSpeedMotorB - 5);
    motors.forwardB();
    delay(50);
    motors.setSpeedA(baseSpeedMotorA + 10);
    motors.forwardA();
  }

  motors.setSpeedB(baseSpeedMotorB);
  motors.forwardB();
  delay(50);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forwardA();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    //      Serial.println("Turning right");
    motors.setSpeedB(baseSpeedMotorB + 10);
    motors.forwardB();
    delay(50);
    motors.setSpeedA(baseSpeedMotorA - 5);
    motors.forwardA();
  }
}

/*
 * Testing: one colour sensor sees red, stops a motor and vice versa
 * Right colour sensor sees red, stop Motor B (left)
 * Left colour sensor sees red, stop Motor A (right)
 */
void testingStopTurn() { // James
  motors.setSpeedB(baseSpeedMotorB);
  motors.setSpeedA(baseSpeedMotorA);
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.stopB();
  }

//  motors.setSpeedB(baseSpeedMotorB);
//  motors.setSpeedA(baseSpeedMotorA);
//  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.stopA();
  }
}

/*
 * Has a print of colour sensor readings for the right colour sensor
 */
void testingColour() {
  selectMuxPin(colourRightAddress);
  if (foundRed(colourRight)) 
  {
    Serial.println("\nRight");
    motors.stop();
    while (1) {}
  }
}

/*
 * Reading colours to console
 * Use for colour calibration
 */
void readingColours(){
  uint16_t r, g, b, clear;
  selectMuxPin(colourRightAddress);
  colourRight.getRawData(&r, &g, &b, &clear);
  Serial.println("\n\nRight");
  printColourInfo(r, g, b, clear);

  uint16_t rL, gL, bL, clearL;
  selectMuxPin(colourLeftAddress);
  colourLeft.getRawData(&rL, &gL, &bL, &clearL);
  Serial.println("\nLeft");
  printColourInfo(rL, gL, bL, clearL);  

//  delay(2000);

  // Church Lab Environment
  // cork: R: 4000, G: 3970,, B: 3074
  // green: 2717 (2409) (2560) 2261, G: 3655 (3805) (4145), B: 2210 (1900) (2040) 1770
  // red: 2919-20 (2904), G: 830 (780) 429 , B: 827 (790) 653
  // blue: R: 697 (622), G: 2069 (1970), B: 3449 (3291)
}

/*
   Select desired channel pin on 12C address using multiplexer
*/
void selectMuxPin(uint8_t channelPin) {
  if (channelPin > 7) {
    return;
  }

  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channelPin);
  Wire.endTransmission();
}

/*
 * Finds red
 */
bool foundRed(Adafruit_TCS34725 colourSensor) {
  uint16_t r, g, b, clear;
  colourSensor.getRawData(&r, &g, &b, &clear);

//  printColourInfo(r, g, b, clear);
  //  if ((r >= 700) && (g < 300) && (b < 300)){ // at night 11/10/2021 8:07 PM
  //  if ((r >= 500) && (g < 300) && (b < 300)){ // at night 11/10/2021 11:07 PM (this prob is good)
  //  if ((r >= 500) && (g < 300) && (b < 300)){ // cloudy afternoon 11/14/2021 3:22 PM
//  if ((r >= 2000) && (g < 1000) && (b < 1000)) { // night 11/18/2021 6:42 PM cardboard
  if ((r >= 1000)) { // night 11/19/2021 
    Serial.print("\nFOUND RED.");
    return true;
  }

  return false;
}

/*

*/
bool foundBlue(Adafruit_TCS34725 colourSensor) {
  uint16_t r, g, b, clear;//, lux;
  colourSensor.getRawData(&r, &g, &b, &clear);
  //  lux = colourSensor.calculateLux(r, g, b);

  //  printColourInfo(r, g, b, clear, lux);
  //  if ((r < 200) && (g < 400) && (b >= 500)){ // at night 11/10/2021 8:07 PM
  if ((r < 600) && (g < 1400) && (b >= 2000)) { // at night 11/19/2021 6:43 PM
    Serial.println("\nFOUND BLUE.");
    return true;
    // NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
  }

  return false;
}

void printColourInfo(uint16_t r, uint16_t g, uint16_t b, uint16_t clear) {
  // Print read colour sensor values
  Serial.print("\nC: ");
  Serial.print(clear);
  Serial.print("\tR: ");
  Serial.print(r);
  Serial.print("\tG: ");
  Serial.print(g);
  Serial.print("\tB: ");
  Serial.print(b);
}
