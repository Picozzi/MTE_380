#include <L298NX2.h>            // Motor drive controller library (powers 2 motors)
#include <SharpIR.h>            // IR proximity sensor library

/* --- Motor Controller Defs --- */
/* NOTE: May name motorA as motorRight instead later */
#define enablePinA 3                // PWM signal for controlling speed Motor A
#define inPin1A 4                   // Digital input pin to control spin direction of Motor A
#define inPin2A 5                   // Digital input pin to control spin direction of Motor A

#define enablePinB 6                // PWM signal for controlling speed Motor B
#define inPin1B 7                   // Digital input pin to control spin direction of Motor B
#define inPin2B 8                   // Digital input pin to control spin direction of Motor B

#define lowestMotorSpeed 150        // Standard motor driving speed
#define standardMotorSpeed 200      // Standard motor driving speed
#define baseSpeedMotorA 224

L298NX2 motors(enablePinA, inPin1A, inPin2A, enablePinB, inPin1B, inPin2B); // RENAME TO dcMotors

/* --- IR Proximity Sensor Defs --- */
#define irPin A0                  // Analog input pin
#define irModel 1080              // sensor model library uses for GP2Y0A21YK
#define irLegoThreshold 5         // ***CHANGE THROUGH TESTING; distance threshold to find the Lego man

SharpIR SharpIR(irPin, irModel); // Unsure if I can/it's better to assign a var to this object

void setup() {
  Serial.begin(9600); 
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
}

void loop() {
  motors.setSpeedB(standardMotorSpeed);
  motors.setSpeedA(baseSpeedMotorA); // seperate motorABaseSpeed and motorB
  motors.forward();

  getIRDist();
  
  while (getIRDist() < 11){
    Serial.println("\nREACHED THRESHOLD");
    motors.stop();
    delay(2000);
//    while(1){}
   //  NOTE: WHEELS KEEP GOING FOR ~0.5 S AFTER .STOP()
  }
}

/*
 * Get IR promixity sensor's current distance reading
 */
unsigned long getIRDist(){
  unsigned long irDist = 0.3129*SharpIR.distance()+2.24489;
  Serial.print("\nMy IR Distance:\t");
  Serial.print(irDist);
  delay(500);
  return irDist;
}
