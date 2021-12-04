/*
 * Drive forward
 */
void forward(int speed, int offset){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, speed);
  analogWrite(enB, speed + offset);
}

/*
 * Drive backwards
 */
void reverse(int speed, int offset){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, speed);
  analogWrite(enB, speed + offset);
}

/*
 * Stop robot from moving
 */
void stopDriving(){
  analogWrite(enA,0);
  analogWrite(enA,0);
}

/*
 * Stop right DC motor
 */
void stopRight(){
  analogWrite(enB, 0);
}

/*
 * Stop left DC motor
 */
void stopLeft(){
  analogWrite(enA,0);
}

/*
 * Drive right DC motor
 */
void driveRight(int speed){
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, speed);
}

/*
 * Drive left DC motor
 */
void driveLeft(int speed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
}

/*
 * Drive right DC motor backwards
 */
void reverseRight(int speed){
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, speed);
}

void reverseLeft(int speed){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
}

/*
 * Drive in a CW turn
 */
void clockwiseTurn(int speed){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}
