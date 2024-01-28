String content = "";
void setup() {
  // put your setup code here, to run once:
pinMode(8, OUTPUT);
digitalWrite(8, 0);
digitalWrite(8, 1);
delayMicroseconds(5);
digitalWrite(8, 0);
Serial.begin(9600);
}

void loop() {
  while (Serial.available()){
    content = Serial.readStringUntil('\r');
    content = content.substring(1);
  }
  Serial.print(content);
  Serial.println("mm");
digitalWrite(8, 1);
delay(10);
digitalWrite(8, 0);
}
