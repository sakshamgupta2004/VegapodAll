#include <SPI.h>
#include <RFID.h>
#define SS_PIN 10
#define RST_PIN 9
RFID rfid(SS_PIN, RST_PIN);

String rfidCard;
String allowedCards[] = {"179 250 223 18", "136 5 127 25"}; //51 165 239 18
bool allowed = false;
void setup() {
  SPI.begin();
  rfid.init();
  Serial.begin(9600);
  pinMode(8, OUTPUT);
  pinMode(14, OUTPUT);
}

void loop() {
  if (rfid.isCard()) {
    if (rfid.readCardSerial()) {
      rfidCard = String(rfid.serNum[0]) + " " + String(rfid.serNum[1]) + " " + String(rfid.serNum[2]) + " " + String(rfid.serNum[3]);
      Serial.println(rfidCard);
      if (sizeof(allowedCards) > 0) {
        for (int i = 0; i < (sizeof(allowedCards) / sizeof(allowedCards[0])); i++) {
          if (strcmp(rfidCard.c_str(), allowedCards[i].c_str()) == 0) {
            allowed = true;
            break;
          }
          else {
            allowed = false;
          }
        }
        if (true) {
          if (sizeof(allowedCards) > 0) {
            for (int i = 0; i < (sizeof(allowedCards) / sizeof(allowedCards[0])); i++) {
              if (strcmp(rfidCard.c_str(), allowedCards[i].c_str()) == 0) {
                allowed = true;
                break;
              }
              else {
                allowed = false;
              }
            }
            if (allowed) {
              digitalWrite(8, HIGH);
              digitalWrite(14, HIGH);
              delay(500);
              digitalWrite(8, LOW);
              digitalWrite(14, LOW);
              allowed = false;
            }
            else {
              digitalWrite(14, HIGH);
              delay(100);
              digitalWrite(14, LOW);
              delay(100); 
              digitalWrite(14, HIGH);
              delay(100);
              digitalWrite(14, LOW);
            }

          }
        }
        rfid.halt();
      }
    }
  }}
