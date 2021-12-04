/* Global variables for making a defined rotation */
float prevTime = 0;
float currAngle = 0;
float newAngle = 0;     // Reset reference angle position each time

/*
 * Determines if 180 deg of angle position difference was reached
 */
void turn180(){
  newAngle = 0;
  currAngle = 0;
  float angle = 0;
    
  while(abs(angle) < 175){ // Set to 175 deg since dc motors need buffer time
    angle = getAngleZ();
  }
}

/*
 * Gets yaw angle (from gyro_z) and returns the current accumulated angle difference 
 * from the initial set reference position. Use in conjunction with above turn180()
 */
float getAngleZ(){
  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  selectMuxPin(imuAddress);
  imu.getEvent(&accel, &gyro, &temp, &mag);

  float gyroX = gyro.gyro.x;
  float gyroY = gyro.gyro.y;
  float gyroZ = gyro.gyro.z;

  Serial.print("\n\tGyro X: ");
  Serial.print(gyroX);
  Serial.print(" \tY: ");
  Serial.print(gyroY);
  Serial.print(" \tZ: ");
  Serial.print(gyroZ);
  Serial.println(" radians/s ");
  Serial.println();

  float currAngVelocity = gyroZ;

  if (abs(gyroZ) >= 0.05){
    float currTime = millis();
  
    float timeElapsed = (currTime - prevTime)/1000;
    float deltaAngle = 180/M_PI * (currAngVelocity * timeElapsed);
  
    currAngle += deltaAngle; // [deg]
    prevTime = currTime;
  }
  
  return currAngle;
}
