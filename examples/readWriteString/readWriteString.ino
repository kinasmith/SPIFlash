/*
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                             readWriteString.ino                                                               |
  |                                                               SPIFlash_Marzogh library                                                                |
  |                                                                   v 2.5.0                                                                     |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                                    Marzogh                                                                    |
  |                                                                  30.09.2016                                                                   |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
  |                                                                                                                                               |
  |                        This program shows the method of reading a string from the console and saving it to flash memory                       |
  |                                                                                                                                               |
  |~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~|
*/
#include<SPIFlash_Marzogh.h>

int strPage, strSize;
byte strOffset;

SPIFlash_Marzogh flash;

bool readSerialStr(String &inputStr);

void setup() {
  Serial.begin(115200);

  flash.begin();

  //Serial.println(F("Please type the string into the console"));
  randomSeed(analogRead(A0));
  strPage = random(0, 4095);
  strOffset = random(0, 255);
  String inputString = "This is a test String";
  //while (!readSerialStr(inputString));
  if (flash.writeStr(strPage, strOffset, inputString)) {
    //delay(250);
    Serial.print(F("Written string: "));
    Serial.print(inputString);
    Serial.print(F(" to page "));
    Serial.print(strPage);
    Serial.print(F(", at offset "));
    Serial.println(strOffset);
  }
  String outputString = "";
  if (flash.readStr(strPage, strOffset, outputString)) {
    Serial.print(F("Read string: "));
    Serial.print(outputString);
    Serial.print(F(" from page "));
    Serial.print(strPage);
    Serial.print(F(", at offset "));
    Serial.println(strOffset);
  }
  while (!flash.eraseSector(strPage, 0));
}

void loop() {

}

//Reads a string from Serial
bool readSerialStr(String &inputStr) {
  if (!Serial)
    Serial.begin(115200);
  while (Serial.available()) {
    inputStr = Serial.readStringUntil('\n');
    Serial.println(inputStr);
    return true;
  }
  return false;
}
