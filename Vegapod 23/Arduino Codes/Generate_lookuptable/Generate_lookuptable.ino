#include<math.h>
int a = 0, count = 100;
void setup() {
  // put your setup code here, to run once:
Serial.begin(2000000);
Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
Serial.println("int lookUp2[] = {");
while (a<=count){
  Serial.print((sin(6.0*(PI/3.0)*(a/(float)count)) * 800) + 800);
  Serial.println(", ");
  a++;
}
Serial.println("};");
Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
a = 0;
Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
while (a<=count){
  Serial.print((sin(6.0*(PI/3.0)*(a/(float)count) + ((4*PI)/3)) * 800) + 800);
  Serial.println(", ");
  a++;
}
Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
a = 0;
Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
while (a<=count){
  Serial.print((sin(6.0*(PI/3.0)*(a/(float)count) + ((2*PI)/3)) * 800) + 800);
  Serial.println(", ");
  a++;
}
Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
}
void loop() {
  // put your main code here, to run repeatedly:

}
