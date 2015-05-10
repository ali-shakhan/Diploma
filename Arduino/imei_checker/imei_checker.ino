#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

#define PN532_SCK  (2)
#define PN532_MOSI (3)
#define PN532_SS   (4)
#define PN532_MISO (5)
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

uint8_t selectApdu[] = { 
          0x00, /* CLA */
          0xA4, /* INS */
          0x04, /* P1  */
          0x00, /* P2  */
          0x07, /* Length of AID  */
          0xF0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
          0x00  /* Le  */};
char IMEI[15];
uint8_t response[15];
uint8_t responseLength = sizeof(response);

void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  
  
  //NFC initialization
  nfc.begin();
  if (! nfc.getFirmwareVersion()) {
    Serial.println("Did not find the shield - locking up");
    while (true) {
    }
  }
  Serial.println("Found chip PN5"); ;
  nfc.SAMConfig();
  nfc.begin();
  delay(1000);
}


void loop(){
  //Serial.println("Waiting...");
    if(nfc.inListPassiveTarget()){
        if(nfc.inDataExchange(selectApdu, sizeof(selectApdu), response, &responseLength)) {
           Wire.beginTransmission(4); 
           for (int i = 0; i < 15; i++) {
             Wire.write((char)response[i]);              
            }         
             Wire.endTransmission();
             delay(100);
             Wire.beginTransmission(4);
             Wire.requestFrom(4, (uint8_t)1);
             char receivedValue = Wire.read(); 
             Serial.println(receivedValue);
             Wire.endTransmission();
        }
        else{
            Serial.println("Failed sending SELECT AID"); 
        }
    }
}
