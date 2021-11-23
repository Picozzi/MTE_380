////code to stop at an angle in the middle of a turn
//      while(1)
//      {
//        currentTime = millis();
//          imu.getEvent(&accel, &gyro, &temp, &mag);
//          nextAngPosition = gyro.gyro.z;  
//        endTime = millis();
//        angPosition += nextAngPosition *180/3.14 * (endTime - currentTime)/1000;
//        if (angPosition > 720)
//         break;
//      }
//      motors.setSpeed(0);
//      
//      Serial.print("\nAngular position: ");
//      Serial.print(angPosition);
//      Serial.println();
//      Serial.print(" \tZ: ");
//      Serial.print(gyro.gyro.z);
//      Serial.println(" radians/s ");
//
////Code to update angular position after each turn
//      while (abs(gyro.gyro.z) >= 0.05)
//      {
//        currentTime = millis();
//        while (abs(gyro.gyro.z) >= 0.05 )
//        { 
//          if (abs(nextAngPosition) > abs(newAngPosition))
//          {
//            newAngPosition = nextAngPosition;
//          }
//          imu.getEvent(&accel, &gyro, &temp, &mag);
//          nextAngPosition = gyro.gyro.z;
//        }  
//        endTime = millis();
//      }
//      angPosition += (nextAngPosition+newAngPosition)/2 *180/3.14 * (endTime - currentTime)/1000;
//      
//      Serial.print("\nAngular position: ");
//      Serial.print(angPosition);
//      Serial.println();
//      Serial.print(" \tZ: ");
//      Serial.print(gyro.gyro.z);
//      Serial.println(" radians/s ");
