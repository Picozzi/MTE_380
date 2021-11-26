void selectMuxPin(uint8_t channelPin){
  if (channelPin > 7){
    return;
  } 

  Wire.beginTransmission(muxAddress);
  Wire.write(1 << channelPin);
  Wire.endTransmission();
}
