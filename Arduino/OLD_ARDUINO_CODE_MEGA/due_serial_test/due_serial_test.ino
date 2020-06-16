byte count = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print ("Count = ");
  Serial.println (count);
  count++;
  delay(500);
}
