/*
 * fota.c
 *
 *  Created on: Apr 29, 2024
 *      Author: wkhaled
 */
#include "fota.h"



#include <Update.h>

#define SW_VERSION 4
#define BUFFER_SIZE 1024
#define chunkSize 1024




 struct fileinfo_struct {
  long int contentLength;
  int filestart_postion;
};
typedef struct fileinfo_struct fileinfo_structfunc;
typedef struct fileinfo_struct fileinfo_structuse;
static int counter_flash;
static size_t bufferIndex = 0;
static char SW_version_new;



/*-------------------------------------------function prototype -----------------------------------------------------*/



void connectTo_HTTP_get_SW(void) ;
void connectTo_HTTP_get(void);
fileinfo_structfunc READ_firstCHUNKextrectfota(void) ;


void connectTo_HTTP_get(void) {

  modem.sendAT("+QFDEL=\"UFS:1.bin\"");
  modem.waitResponse(1000);

  modem.sendAT("+QHTTPCFG=\"contextid\",1");
  modem.waitResponse(1000);
  modem.sendAT("+QHTTPCFG=\"responseheader\",1");
  modem.waitResponse(1000);
  modem.sendAT("+QICSGP=1,1,\"INTERNET\","
               ","
               ",1");
  modem.waitResponse(1000);
  modem.sendAT("+QIACT=1");
  modem.waitResponse(2000);
  modem.sendAT("+QHTTPURL=49,80");
  // modem.sendAT("+QHTTPURL=80,80");


  delay(2000);
  String n = "http://3.77.81.209:8081/firmware/ATdebug1.ino.bin";
  SerialAT.println(n);  // Send URL to modem via UART
  delay(2000);          // 1000 ms pause

  modem.sendAT("+QHTTPGET=80");
  modem.waitResponse(5000);
  delay(1000);
  modem.sendAT("+QHTTPREADFILE=\"UFS:1.bin\",80");
  modem.waitResponse(10000);
  delay(1000);
}






void READ_CHUNKsizeflash(int BUFFER_SIZE_of_chunk) {
  uint8_t data;
  uint8_t buffernew[BUFFER_SIZE_of_chunk];
  int bufferIndex = 0;
  char command[50];  // Adjust the buffer size as needed
    // unsigned long startTimeof_blockage = millis();
  SerialAT.flush();



  snprintf(command, sizeof(command), "AT+QFREAD=1027,%d\r\n", BUFFER_SIZE_of_chunk);
  SerialAT.write(command);
  delay(300);  // 500

  if (SerialAT.available()) {
    while (1) {

      String line = SerialAT.readStringUntil('\n');
      line.trim();


      if (line.startsWith("CONNECT ")) {


        while ((bufferIndex < BUFFER_SIZE_of_chunk)) {
          data = SerialAT.read();
          buffernew[bufferIndex] = data;
          bufferIndex++;
        }
        Serial.print("i'm here ");
        Serial.println(bufferIndex);
        counter_flash++;
        if (Update.write(buffernew, BUFFER_SIZE_of_chunk) == BUFFER_SIZE_of_chunk) {

          Serial.println("flash_success");
        } else {
          Update.printError(Serial);
        }
        break;
      }

      //

      // if (millis() - startTimeof_blockage < 3000) {
      //   break;
      // }


      //
    }
  }
}


void close_file() {

  modem.sendAT("+QFCLOSE=1027");  // to be put in setup

  modem.waitResponse(1000);
}











String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

fileinfo_structfunc READ_firstCHUNKextrectfota(void) {
  fileinfo_structfunc s1;
  uint8_t data;
  uint8_t buffernew[1000];
  int bufferIndex = 0;
  SerialAT.flush();
  modem.sendAT("+QFOPEN=\"UFS:1.bin\"");  // to be put in setup

  modem.waitResponse(1000);
  modem.sendAT("+QFSEEK=1027,0,0");
  modem.waitResponse(300);
  //modem.sendAT("+QFREAD=1027,1000");
  //modem.waitResponse(300);
  SerialAT.write("AT+QFREAD=1027,1000\r\n");
  delay(1000);

  if (SerialAT.available()) {
    while (1) {
      String line = SerialAT.readStringUntil('\n');
      line.trim();
      if (line.startsWith("CONNECT 1000")) {
        Serial.print("i'm here");
        while ((bufferIndex < 1000)) {
          data = SerialAT.read();
          buffernew[bufferIndex] = data;
          bufferIndex++;
        }
        break;
      }
    }
  }
  s1.filestart_postion = -1;
  s1.contentLength = -1;
  for (int itra = 0; itra < 1000; ++itra) {
    if (buffernew[itra] == 0xE9) {
      s1.filestart_postion = itra;  // Return the position of the target character
      break;
    }
  }
  Serial.print("file start postion and checked magic number:");
  Serial.println(s1.filestart_postion);
  const char* contentLengthTag = "Content-Length: ";
  const char* contentLengthStart = strstr((char*)buffernew, contentLengthTag);
  if (contentLengthStart) {
    s1.contentLength = atol(contentLengthStart + strlen(contentLengthTag));
  }
  Serial.println("recieved " + String(s1.contentLength) + " bytes for fota code ");


  modem.sendAT("+QFCLOSE=1027");  // to be put in setup

  modem.waitResponse(1000);
  return s1;
}


void flashing(long int contentLengthflash, int startofmagicbit) {
  char command[50];

  modem.sendAT("+QFOPEN=\"UFS:1.bin\"");  // to be put in setup

  modem.waitResponse(2000);
  if (!Update.begin(contentLengthflash, U_FLASH)) {
    Update.printError(Serial);
    Serial.println("issue in reading ");
    return;
  }


  snprintf(command, sizeof(command), "+QFSEEK=1027,%d,0", startofmagicbit);
  modem.sendAT(command);
  modem.waitResponse(1000);
  int itr = 0;
  while (itr < ((contentLengthflash / 1024) + 1)) {
    if (itr == (contentLengthflash / 1024)) {

      int flashlast = (contentLengthflash % 1024);

      READ_CHUNKsizeflash(flashlast);
      delay(1000);
      Serial.println("congratulation");
      modem.sendAT("+QFPOSITION=1027");
      modem.waitResponse(1000);

      break;
    }
    Serial.println("i'm here");
    Serial.print(itr);
    READ_CHUNKsizeflash(1024);
    delay(60);  //100
    ++itr;
  }
  if (Update.end(true)) {  //requires sufficient space to finalize the update process successfully. If the flash memory is nearly full, it may cause errors during the update.
    if (Update.isFinished()) {

      close_file();
      delay(1000);
      Serial.println(" el7 ,Update successfully completed. Rebooting...");
      //  return_uart();
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("Firmware update failed");
    }

  } else {
    Update.printError(Serial);
    Serial.println("not end ");
    return;
  }
}

void connectTo_HTTP_get_SW(void) {

  modem.sendAT("+QFDEL=\"UFS:1.bin\"");
  modem.waitResponse(1000);

  modem.sendAT("+QHTTPCFG=\"contextid\",1");
  modem.waitResponse(1000);
  modem.sendAT("+QHTTPCFG=\"responseheader\",1");
  modem.waitResponse(1000);
  modem.sendAT("+QICSGP=1,1,\"INTERNET\","
               ","
               ",1");
  modem.waitResponse(1000);
  modem.sendAT("+QIACT=1");
  modem.waitResponse(2000);
  modem.sendAT("+QHTTPURL=30,80");



  delay(2000);
  String n = "http://3.77.81.209:8081/SW_VER";
  SerialAT.println(n);  // Send URL to modem via UART
  delay(2000);          // 1000 ms pause

  modem.sendAT("+QHTTPGET=80");
  modem.waitResponse(5000);
  delay(1000);
  // modem.sendAT("+QHTTPREAD=80");
  // modem.waitResponse(5000);
  SerialAT.write("AT+QHTTPREAD=80\r\n");
  delay(500);
  if (SerialAT.available()) {
    while (1) {
      String line = SerialAT.readStringUntil('\n');
      // Serial.println(line);
      line.trim();

      if (line.startsWith("Connection: ")) {
        Serial.print("i'm here");
        SerialAT.readStringUntil('\n');

        SW_version_new = SerialAT.read();
        Serial.print("SW version =");
        Serial.println(SW_version_new);

        break;
      }
    }
  }

  modem.sendAT("+QHTTPSTOP");
  modem.waitResponse(5000);
}



void fota(bool nbool) {

  if (true == nbool) {
    connectTo_HTTP_get_SW();

    if (SW_version_new != SW_VERSION) {
      SerialAT.end();
      delay(100);

      SerialAT.setRxBufferSize(2048);
      SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
      delay(200);
      connectTo_HTTP_get();
      delay(5000);
      fileinfo_structfunc s2 = READ_firstCHUNKextrectfota();
      delay(1000);
      flashing(s2.contentLength, s2.filestart_postion);
    }
  }
}

