#include "SIM900.h"
#include <SoftwareSerial.h>
#include "inetGSM.h"
#include <EEPROM.h>
#include <Wire.h>
#include <Timer.h>

InetGSM inet;
char IMEI[15];
char newIMEI[15];
char path[30];
char msg[1];
int eeAddress = 0;
int imeiCounter = 0;
Timer timer;
boolean started = false;
char open = 'c';

void setup() {
  path[0] = '/';
  path[1] = 'i';
  path[2] = 'm';
  path[3] = 'e';
  path[4] = 'i';
  path[5] = '.';
  path[6] = 'p';
  path[7] = 'h';
  path[8] = 'p';
  //  path[9] = '?';
  //  path[10] = 'I';
  //  path[11] = 'M';
  //  path[12] = 'E';
  //  path[13] = 'I';
  //  path[14] = '=';
  Serial.begin(9600);

  Wire.begin(4);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestCallback);
  Serial.begin(115200);

  timer.every(120000, updateUsers);

  updateUsers();
  Serial.print("char Size: ");
  Serial.print(sizeof(char));
}
void loop() {
  timer.update();
}
void updateUsers() {
  //GSM initialization
  if (gsm.begin(2400)) {
    started = true;
  } else
    Serial.println("\nstatus=IDLE");

  if (started) {
    if (!inet.attachGPRS("internet.beeline.kz", "@internet.beeline", "beeline"))
      Serial.println("status=ERROR");
  }
  delay(2000);
  inet.httpGET("www.smart-lock.org", 80, path, msg, 0);
  delay(1000);
  readGSM();
}

int wCounter = 0;
int j = 0;

void receiveEvent(int howMany) {
  Serial.print("Received: ");
  for (j = 0; j < 15; j++) {
    IMEI[j] = Wire.read();
  }
  Serial.println(IMEI);
  j = 0;
  eeAddress = 0;
  for (int i = 0; i < 10; i++) {
    free(newIMEI);
    eeAddress = i  * sizeof(char) * 15;
    EEPROM.get( eeAddress, newIMEI);
    Serial.println(newIMEI);
    for (int i = 0; i < 15; i++) {
      if (newIMEI[i] == IMEI[i]) {
        wCounter++;
      }
      if (newIMEI[i] == ' ') {
        j++;
      }
    }
    if (wCounter == 15 && j != 15) {
      open = 'o';
      j = 0;
      wCounter = 0;
      break;
    }
    j = 0;
    wCounter = 0;
  }
}
void requestCallback() {
  Serial.println("Request");
  Wire.write(open);
  open = 'c';
}


boolean imeiStart = false;
int counter = 0;


void readGSM() {
  Serial.println("READ GSM");

  eeAddress = 0;
  counter = 0;
  imeiCounter = 0;
  boolean haveImei = false;
  boolean start = false;
  char data = 1;
  int i = 0;
  while (data > 0) {
    data = gsm.readChar(start);
    start = true;
    Serial.print(data);
    delay(1);
    if (imeiStart) {
      newIMEI[counter++] = data;
      if (counter == 15) {
        if (haveImei && imeiCounter < 1) {
          for (int k = 0; k < EEPROM.length(); k++) {
            EEPROM.put( k, 0 );
          }
        }
        EEPROM.put( eeAddress, newIMEI);
        free(newIMEI);
        imeiCounter++;
        eeAddress = imeiCounter * sizeof(char) * 15;
        counter = 0;
        imeiStart = false;
      }
    }
    if (data == '*') {
      haveImei = true;
      imeiStart = true;
    }
    i++;
  }
}
