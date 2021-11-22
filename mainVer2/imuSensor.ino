/*
 * Trying to get angular position (theta) from IMU gyroscope
 */
void testingIMU(){
  // Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);
      
  double nextAngPosition = 0; //should be velocity
  double currentTime;
  double endTime;
  double newAngPosition = 0;
  
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
