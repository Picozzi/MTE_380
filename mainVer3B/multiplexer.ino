/*
 * Select desired channel pin on 12C address using multiplexer
 */
void selectMuxPin(uint8_t channelPin){
  if (channelPin > 7){
    return;
  }
  
 

  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channelPin);
  Wire.endTransmission();
  Serial.print("hi");
}
