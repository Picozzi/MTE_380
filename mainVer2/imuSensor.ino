// Need these global vars for the turnX() functions
float prevTime = 0;
float currAngle = 0;
float newAngle = 0; // reset ref angle position each time

/*
 * Determines if 90 deg of angle position difference was reached
 * NOTE: In real life, probably 150 deg b/c motors have to stop
 */
void turn180(){
  newAngle = 0;
  currAngle = 0;
  float angle = 0;
    
  while(abs(angle) < 180){
    angle = getAngleZ();
  }

  Serial.println("180 deg turn");
  while(1){}
}

/*
 * Determines if 90 deg of angle position difference was reached
 */
void turn90(){
  newAngle = 0;
  currAngle = 0;
  float angle = 0;
    
  while(abs(angle) < 90){
    angle = getAngleZ();
  }

  Serial.println("90 deg turn");
  while(1){}
}

/*
 * Attempt 2
 * USE
 * Gets yaw angle (from gyro_z) and returns the current accumulated angle difference 
 * from the initial set reference position. Use in conjunction with above turnX()
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
    
    Serial.println();
    Serial.print("\ttime elapsed: ");
    Serial.print(timeElapsed);
    Serial.print(" \tdeltaAngle: ");
    Serial.print(deltaAngle);
    Serial.print(" \tcurrAngle: ");
    Serial.print(currAngle);
    Serial.print(" deg");
    Serial.println();
  }
  return currAngle;
}

/*
 * Attempt 1
 */
void getYawAngle(){
//  float currAngle = 0;

  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  selectMuxPin(imuAddress);
  imu.getEvent(&accel, &gyro, &temp, &mag);

  // Raw readings
  float accelX = accel.acceleration.x;
  float accelY = accel.acceleration.y;
  float accelZ = accel.acceleration.z - 10; // Subtract gravity
  float magX_raw = mag.magnetic.x;
  float magY_raw = mag.magnetic.y;
  float magZ_raw = mag.magnetic.z;
  
  float pitchAngle = 180 / M_PI * atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2)));
  float rollAngle = 180 / M_PI * atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2)));

//  float magX = magX_raw*cos(pitchAngle) + magY_raw*sin(rollAngle)*sin(pitchAngle) + magZ_raw*cos(rollAngle)*sin(pitchAngle);
//  float magY = magY_raw*cos(rollAngle) - magZ_raw*sin(rollAngle);
//
//  float yawAngle = 180 / M_PI * atan2(-magY, magX);

  float yawAngle = 180 / M_PI * atan(accelZ / sqrt(pow(accelX, 2) + pow(accelZ, 2)));

//  Serial.println("Here");
//  Serial.print("\tAccel X: ");
//  Serial.print(accel.acceleration.x);
//  Serial.print(" \tY: ");
//  Serial.print(accel.acceleration.y);
//  Serial.print(" \tZ: ");
//  Serial.print(accelZ);
//  Serial.println(" m/s^2 ");

  Serial.print("\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  Serial.print("\tPitch Angle: [deg] ");
  Serial.print(pitchAngle);
  Serial.println();
  Serial.print("\tRoll Angle: [deg] ");
  Serial.print(rollAngle);
  Serial.println();
  Serial.print("\tYaw Angle: [deg] ");
  Serial.print(yawAngle);
  Serial.println();
}

/*
 * Trying to get angular position (theta) from IMU gyroscope
 * James' idea
 */
void testingIMU(){
  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);
      
  float nextAngPosition = 0; //should be velocity
  double currentTime;
  double endTime;
  float newAngPosition = 0;
  
  //code to stop at an angle in the middle of a turn
  while(1)
  {
    currentTime = millis();
    imu.getEvent(&accel, &gyro, &temp, &mag);
    nextAngPosition = gyro.gyro.z;  
    endTime = millis();
    angPosition += nextAngPosition *180/3.14 * (endTime - currentTime)/1000;
    if (angPosition > 720){
      break;
    }
  }
  motors.setSpeed(0);
      
  Serial.print("\nAngular position: ");
  Serial.print(angPosition);
  Serial.println();
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  
  //Code to update angular position after each turn
  while (abs(gyro.gyro.z) >= 0.05)
  {
    currentTime = millis();
    while (abs(gyro.gyro.z) >= 0.05 )
    { 
      if (abs(nextAngPosition) > abs(newAngPosition))
      {
        newAngPosition = nextAngPosition;
      }
      imu.getEvent(&accel, &gyro, &temp, &mag);
      nextAngPosition = gyro.gyro.z;
     }  
     endTime = millis();
  }
  angPosition += (nextAngPosition+newAngPosition)/2 *180/3.14 * (endTime - currentTime)/1000;
      
  Serial.print("\nAngular position: ");
  Serial.print(angPosition);
  Serial.println();
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
}

/*
 * Get IMU readings for accelerometer and gyroscope
 */
void readIMU(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;
  imu.getEvent(&accel, &gyro, &temp, &mag);
//
//  Serial.print("\t\tTemperature ");
//  Serial.print(temp.temperature);
//  Serial.println(" deg C");

  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(2000);
}
