
float digi = 0;
float ana = 0;

int digitalqre = 6;
int analogqre = 19;

void setup() {
  
  Serial.begin(115200);

  pinMode(digitalqre,INPUT);
  pinMode(analogqre,INPUT);
}

void loop() {
  digi = digitalRead(digitalqre);
  ana = analogRead(analogqre);

  Serial.print(digi);
  Serial.print(",");
  Serial.print(ana);
  Serial.println();
  delay(50);

}
