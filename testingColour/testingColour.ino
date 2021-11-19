#include <Wire.h>               // I2C library
#include <Adafruit_TCS34725.h>  // Colour sensor library
#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)

/* --- Colour Sensor --- */
Adafruit_TCS34725 colourRight = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // NOTE: CAN PROBABLY CHANGE THE INPUT PARAMS
Adafruit_TCS34725 colourLeft = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/* --- MUX Defs --- */
#define muxAddress 0x70           // Multiplexer address for I2C
#define colourRightAddress 0
#define colourLeftAddress 1

/* --- Motor Controller Defs --- */
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

#define lowestMotorSpeed 150        // Standard motor driving speed
#define standardMotorSpeed 150      // Standard motor driving speed
#define baseSpeedMotorA 224

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
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
}

void loop() {
  testingTurn();
}

/*
 * Turns: one wheel goes backwards
 */
void testingTurn(){
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.backwardB();
  }

  motors.forwardB();
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.backwardA();
  }

  motors.forwardA();
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forward();
}

/*
 * With b/w motors
 */
void testingMatthew(){
  motors.setSpeedB(standardMotorSpeed);
  motors.forwardB();
  delay(50);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forwardA();

  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    //      Serial.println("Turning right");
    motors.setSpeedB(standardMotorSpeed - 5);
    motors.forwardB();
    delay(50);
    motors.setSpeedA(standardMotorSpeed + 10);
    motors.forwardA();
  }

  motors.setSpeedB(standardMotorSpeed);
  motors.forwardB();
  delay(50);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forwardA();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    //      Serial.println("Turning right");
    motors.setSpeedB(standardMotorSpeed + 10);
    motors.forwardB();
    delay(50);
    motors.setSpeedA(standardMotorSpeed - 5);
    motors.forwardA();
  }
}

/*
 * Testing: one colour sensor sees red, stops a motor and vice versa
 * Right colour sensor sees red, stop Motor B (left)
 * Left colour sensor sees red, stop Motor A (right)
 */
void testingMotorsAndColour() { // James
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forward();
  
  // Shift right
  selectMuxPin(colourRightAddress);
  while (foundRed(colourRight))
  {
    motors.stopB();
    delay(500);
  }

  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forward();

  // Shift left
  selectMuxPin(colourLeftAddress);
  while (foundRed(colourLeft))
  {
    motors.stopA();
    delay(500);
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
  if ((r >= 2000) && (g < 1000) && (b < 1000)) { // night 11/18/2021 6:42 PM cardboard
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
  //  Serial.print("\tLux: ");
  //  Serial.print(lux);
  //  delay(1000);
}
