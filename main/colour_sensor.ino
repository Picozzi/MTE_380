void setupColourSensor(){
  if (colourSensor.begin()){
    // If the sensor starts up correctly
    Serial.println("Found colour sensor!");
  } else {
    Serial.println("The colour sensor was not found...");
    // while (1); // pause
  }

  // Set pin outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  for (int i=0; i<256; i++){
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode){
      gammaTable[i] = 255 - x;  
    } else {
      gammaTable[i] = x;
    }
  }
 }

void runColourSensor(){
  uint16_t clear, red, green, blue, colourTemp, lux;

  colourSensor.setInterrupt(false);                       // turn on LED
  delay(1000);                                            // wait 1000 ms for reading input
  colourSensor.getRawData(&red, &green, &blue, &clear);   // read sensor input

  colourSensor.setInterrupt(true);                        // turn off LED

  // Print read colour sensor values
  Serial.print("C:\t");
  Serial.print(clear);
  Serial.print("\tR:\t");
  Serial.print(red);
  Serial.print("\tG:\t");
  Serial.print(green);
  Serial.print("\tB:\t");
  Serial.print(blue);
}
