void bullseye(int speed, bool blueRight, bool blueLeft){
  if (blueRight){
//    selectMuxPin(colourLeftAddress);
    delay(500);
    reverseRight(speed);
    delay(300);
    //while(!foundBlueTape(colourLeft)){}
    stopLeft();
  }
  else if (blueLeft){
//    selectMuxPin(colourRightAddress);
    delay(500);
    reverseLeft(speed);
    delay(300);
    //while(!foundBlueTape(colourRight)){}
    stopRight();
  }
  forward(speed, 0);
  delay(750);
  
  stopRight();
  stopLeft();
  servoClose();
  delay(1000);

  clockwiseTurn(speed);
  delay(750);

  stopRight();
  stopLeft();

  delay(1000);
  forward(speed,0);
  /*
  while(1){
    Serial.println("end");
  }*/
}

void dropOff(int speed){
  selectMuxPin(colourRightAddress);
  delay(500);
  driveRight(speed);
  while(!foundGreenTape(colourRight)){}
  stopRight();
  
  servoOpen();
  reverse(speed+10,0);
  while(!foundRedTape(colourLeft)){}
  delay(1000);
  servoClose();
  stopRight();
  stopLeft();
}


void zigZag(int speed, bool leftStart, bool rightStart, bool initial){
  bool blueRight = false;
  bool blueLeft = false;
  selectMuxPin(colourRightAddress);
  while(initial){
  if (!(leftStart || rightStart)){
    forward(55,0);
    //Serial.println("forward");
  }
/*
  if(!(leftStart || rightStart)){
    selectMuxPin(colourLeftAddress);
  }*/
  if ((foundRedTape(colourLeft) || leftStart == true) && !blueLeft){
//    Serial.println("Entering left if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    stopLeft();
    selectMuxPin(colourRightAddress);
    delay(50);
    //printColourInfo(r, g, b);
    driveRight(speed);
    
    while(!foundRedTape(colourRight)){
      if (foundBlueTape(colourRight)){
        blueRight = true;
        break;  
      }
    }
    stopRight();
    rightStart = true;
    leftStart = false;
//    Serial.println("Exiting left if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    delay(100);
  }
  
  if (blueLeft || blueRight){
    bullseye(speed, blueRight, blueLeft);
    return;
  }
  /*
  if (!(leftStart || rightStart)){
    selectMuxPin(colourRightAddress);
  }*/
  if ((foundRedTape(colourRight) || rightStart == true) && !blueRight){
//    Serial.println("Entering right if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart); 
    stopRight();
    selectMuxPin(colourLeftAddress);
    delay(50);
    //printColourInfo(r, g, b);
    driveLeft(speed-5);
      
    while(!foundRedTape(colourLeft)){
      if (foundBlueTape(colourLeft)){
        blueLeft = true;
        break;  
      }
    }
    stopLeft();
    leftStart = true;
    rightStart = false;
//    Serial.println("Exiting right if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    delay(100);
  }
  }
  //printColourInfo(r, g, b);
}

void rescueMan(int speed, bool leftStart, bool rightStart, bool initial){
  bool greenLeft = false;
  
  if (!(leftStart || rightStart)){
    forward(55,0);
    //Serial.println("forward");
  }
  
  selectMuxPin(colourRightAddress);
  while(initial){
/*
  if(!(leftStart || rightStart)){
    selectMuxPin(colourLeftAddress);
  }*/
  if (foundRedTape(colourLeft) || leftStart == true){
//    Serial.println("Entering left if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    stopLeft();
    selectMuxPin(colourRightAddress);
    delay(50);
    //printColourInfo(r, g, b);
    driveRight(speed);
    
    while(!foundRedTape(colourRight)){}
    stopRight();
    rightStart = true;
    leftStart = false;
//    Serial.println("Exiting left if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    delay(100);
  }
  /*
  if (!(leftStart || rightStart)){
    selectMuxPin(colourRightAddress);
  }*/
  if (foundRedTape(colourRight) || rightStart == true){
//    Serial.println("Entering right if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart); 
    stopRight();
    selectMuxPin(colourLeftAddress);
    delay(50);
    //printColourInfo(r, g, b);
    driveLeft(speed-5);
      
    while(!foundRedTape(colourLeft)){
      if (foundGreenTape(colourLeft)){
        greenLeft = true;
        break;  
      }
    }
    stopLeft();
    leftStart = true;
    rightStart = false;
//    Serial.println("Exiting right if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    delay(100);
  }

  if (greenLeft){
    dropOff(speed);
    return;
  }
  }
  //printColourInfo(r, g, b);
}

void returnTrip(int speed, bool leftStart, bool rightStart, bool initial){
  while(initial){
  selectMuxPin(colourRightAddress);
  delay(500);
  driveRight(speed);
  /*
  if (!(leftStart || rightStart)){
    selectMuxPin(colourRightAddress);
  }*/
  if (foundRedTape(colourRight) || rightStart == true){
//    Serial.println("Entering right if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart); 
    stopRight();
    selectMuxPin(colourLeftAddress);
    delay(50);
    //printColourInfo(r, g, b);
    driveLeft(speed-5);
      
    while(!foundRedTape(colourLeft)){}
    stopLeft();
    leftStart = true;
    rightStart = false;
//    Serial.println("Exiting right if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    delay(100);
  }
    
/*
  if(!(leftStart || rightStart)){
    selectMuxPin(colourLeftAddress);
  }*/
  if (foundRedTape(colourLeft) || leftStart == true){
//    Serial.println("Entering left if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    stopLeft();
    selectMuxPin(colourRightAddress);
    delay(50);
    //printColourInfo(r, g, b);
    driveRight(speed);
    
    while(!foundRedTape(colourRight)){}
    stopRight();
    rightStart = true;
    leftStart = false;
//    Serial.println("Exiting left if:");
//    Serial.println(leftStart);
//    Serial.println(rightStart);
    delay(100);
  }
  }
  //printColourInfo(r, g, b);
}
