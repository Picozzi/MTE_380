void stopAtRed(int speed){
  bool red = false;
  if(!red){
  forward(speed,0);
  }
  
  while(!red){
    selectMuxPin(colourLeftAddress);
    if (foundRedTape(colourLeft)){
      red = true;
      printColourInfo(r, g, b);
      break;
    }
    
    
    selectMuxPin(colourRightAddress);
    if (foundRedTape(colourRight)){
      red = true;
      printColourInfo(r, g, b);
      break;
    }
    printColourInfo(r, g, b);
  }
  forward(0,0);
  while(1){
    //Serial.println("End");
  }
}

void reverseTest(int speed){
  Serial.println("Reverseing right");
  reverseRight(speed);
  delay(1000);
  stopRight();
  delay(1000);
  Serial.println("Reversing left");
  reverseLeft(speed);
  delay(1000);
  stopLeft();
  delay(1000);
}
