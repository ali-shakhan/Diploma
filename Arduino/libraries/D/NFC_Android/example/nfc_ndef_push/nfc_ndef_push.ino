/**
 * This example demonstrates pushing a NDEF from Arduino + NFC Shield to Android 4.0+
 *
 * Note: to enable NFC Shield wake up Arduino, D2/INT0 need to be connected with IRQ.
 */


#include <PN532.h>
#include <NFCLinkLayer.h>
#include <SNEP.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <NdefMessage.h>

#define SCK 13
#define MOSI 11
#define SS 10
#define MISO 12

PN532 nfc(SCK, MISO, MOSI, SS);
NFCLinkLayer linkLayer(&nfc);
SNEP snep(&linkLayer);


// NDEF messages
#define MAX_PKT_HEADER_SIZE  50
#define MAX_PKT_PAYLOAD_SIZE 100
uint8_t txNDEFMessage[MAX_PKT_HEADER_SIZE + MAX_PKT_PAYLOAD_SIZE];
uint8_t *txNDEFMessagePtr; 
uint8_t txLen;


void phoneInRange()
{
  //sleep_disable(); // Prevents the arduino from going to sleep if it was about too. 
}

void setup(void) {
    Serial.begin(115200);
    Serial.println(F("----------------- nfc ndef push --------------------"));


    txNDEFMessagePtr = &txNDEFMessage[MAX_PKT_HEADER_SIZE];
    NdefMessage message = NdefMessage();
    message.addTextRecord("This is NFC Shield from Seeed Studio");
    txLen = message.getEncodedSize();
    if (txLen <= MAX_PKT_PAYLOAD_SIZE) {
      message.encode(txNDEFMessagePtr);
    } else {
      Serial.println("Tx Buffer is too small.");
      while (1) {
      }
    }
    
    
    nfc.initializeReader();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
        Serial.print("Didn't find PN53x board");
        while (1); // halt
    }
    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
    Serial.print("Supports "); Serial.println(versiondata & 0xFF, HEX);
    
    // set power sleep mode
    set_sleep_mode(SLEEP_MODE_ADC);

    nfc.SAMConfig();
    
    // interrupt to wake MCU
    attachInterrupt(0, phoneInRange, FALLING);
}

uint8_t buf[180];

void loop(void) 
{
   Serial.println();
   Serial.println(F("---------------- LOOP ----------------------"));
   Serial.println();


    uint32_t txResult = GEN_ERROR;
    
     if (IS_ERROR(nfc.configurePeerAsTarget(SNEP_SERVER))) {
        sleepMCU();
//        nfc.configurePeerAsTarget(SNEP_SERVER);
        PN532_CMD_RESPONSE *response = (PN532_CMD_RESPONSE *)buf;
        nfc.readspicommand(PN532_TGINITASTARGET, response);
     }
    
    do {
  
        if (snep.pushPayload(txNDEFMessagePtr, txLen) == RESULT_SUCCESS) {
          Serial.println(F("Succeed to push a NDEF message."));
          delay(3000); 
        } else {
          Serial.println(F("Fail to push a NDEF message."));
        }
     } while(0);
     
     
}

void sleepMCU()
{
    delay(100);  // delay so that debug message can be printed before the MCU goes to sleep
    
    // Enable sleep mode
    sleep_enable();          
    
    power_adc_disable();
    power_spi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_timer2_disable();
    power_twi_disable();
    
    // Puts the device to sleep.
    sleep_mode();  
    Serial.println("Woke up");          
    
    // Program continues execution HERE
    // when an interrupt is recieved.

    // Disable sleep mode
    sleep_disable();         
    
    
    power_all_enable();
}   
   
