#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <ctype.h>
#include <Wire.h>

/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 3(rx) and 2(tx).
*/
static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;
#define sdcardPin  8

// The TinyGPS++ object
TinyGPSPlus gps;
File myFile;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  pinMode(10,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  SD.begin(sdcardPin);                                       //Start SD Card so it is available for other function calls
  Serial.begin(115200);
  ss.begin(GPSBaud);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      logInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
  myFile = SD.open("gpsdata.txt", FILE_WRITE);   //open test.txt
  if (myFile) {                                    //if the file opened okay, write to it:
    myFile.println(F("No GPS detected: check wiring."));
    myFile.close();
  }
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void logInfo()
{
  myFile = SD.open("gpsdata.txt", FILE_WRITE);   //open test.txt
  if (myFile) {                                    //if the file opened okay, write to it:
  myFile.print(F("Location: ")); 
    Serial.print(F("Location: ")); 
  if (gps.location.isValid()){
    myFile.print(gps.location.lat(), 6);
    myFile.print(F(","));
    myFile.print(gps.location.lng(), 6);
      Serial.print(gps.location.lat(), 6);
//      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
  }
  else{
    myFile.print(F("INVALID"));
      Serial.print(F("INVALID"));
  }

  myFile.print(F("  Date/Time: "));
    Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()){
    myFile.print(gps.date.month());
    myFile.print(F("/"));
    myFile.print(gps.date.day());
    myFile.print(F("/"));
    myFile.print(gps.date.year());
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.print(gps.date.year());
  }
  else{
    myFile.print(F("INVALID"));
      Serial.print(F("INVALID"));
  }

  myFile.print(F(" "));
  if (gps.time.isValid()){
    if (gps.time.hour() < 10) {
      myFile.print(F("0"));
        Serial.print(F("0"));      
    }
    myFile.print(gps.time.hour());
    myFile.print(F(":"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
    if (gps.time.minute() < 10) {
      myFile.print(F("0"));
        Serial.print(F("0"));
    }
    myFile.print(gps.time.minute());
    myFile.print(F(":"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
    if (gps.time.second() < 10) {
      myFile.print(F("0"));
        Serial.print(F("0"));
    }
    myFile.print(gps.time.second());
    myFile.print(F("."));
      Serial.print(gps.time.second());
      Serial.print(F("."));
    if (gps.time.centisecond() < 10) {
      myFile.print(F("0"));
        Serial.print(F("0"));
    }
    myFile.print(gps.time.centisecond());
      Serial.print(gps.time.centisecond());
  }
  else{
    myFile.print(F("INVALID"));
      Serial.print(F("INVALID"));
  }
  myFile.println();
    Serial.println();
  myFile.close();                                //close the file
  } 
}
