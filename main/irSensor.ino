/*
 * Get IR promixity sensor's current distance reading
 */
unsigned long getIRDist(){
  unsigned long irDist = 0.3129*SharpIR.distance()+2.24489;
  Serial.print("\nMy IR Distance:\t");
  Serial.print(irDist);

  return irDist;
}
