void setup() {
  // put your setup code here, to run once:
pinMode(5, OUTPUT);
digitalWrite(5, LOW);
pinMode(6, OUTPUT);
digitalWrite(6, HIGH);
pinMode(10, OUTPUT);
digitalWrite(10, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  
digitalWrite(5, HIGH);
digitalWrite(6, LOW);
delay(1);
digitalWrite(6, HIGH);
digitalWrite(5 , LOW);
delay(1);
}
