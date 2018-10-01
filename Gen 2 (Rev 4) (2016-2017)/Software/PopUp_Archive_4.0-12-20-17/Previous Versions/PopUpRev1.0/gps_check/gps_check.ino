//include various libraries
#include <ctype.h>
#include <Wire.h>
#include <SPI.h>  
#include <math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <IridiumSBD.h>
#include <RTClib.h>
#include <RTC_DS3234.h>

const byte MON = 1;        
const byte DAY = 29;
const byte YEAR = 16;

const byte HOUR = 13;    
const byte MIN = 0;
const byte SEC = 0;

#define TEMP_SWITCH 3
#define SENSOR_POWER_5V 6
#define SENSOR_POWER_3V3 22
#define GPS_ON 9
#define IRIDIUM_POWER 8
#define IRIDIUM_ON_OFF A6
#define ACCEL_CS A8
#define SDCARD_CS A5
#define RTC_CS A2
#define SHUTDOWN A0
#define GPS_TX 11
#define GPS_RX 12
#define IRIDIUM_RX A14
#define IRIDIUM_TX A15
#define SDA 20
#define SCL 21

SoftwareSerial ss(GPS_TX,GPS_RX);
SoftwareSerial nss(IRIDIUM_RX,IRIDIUM_TX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

void setup(){
    pinMode(3,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(9,OUTPUT);
    pinMode(22,OUTPUT);
    pinMode(47,OUTPUT);
    pinMode(A8,OUTPUT);
    pinMode(A6,OUTPUT);
    pinMode(A5,OUTPUT);
    pinMode(A2,OUTPUT);
    pinMode(A0,OUTPUT);
    
    digitalWrite(TEMP_SWITCH,LOW);
    digitalWrite(SENSOR_POWER_5V,LOW);
    digitalWrite(SENSOR_POWER_3V3,LOW);
    digitalWrite(GPS_ON,LOW);
    digitalWrite(IRIDIUM_POWER,LOW);
    digitalWrite(IRIDIUM_ON_OFF,LOW);
    digitalWrite(ACCEL_CS,HIGH);
    digitalWrite(SDCARD_CS,HIGH);
    digitalWrite(RTC_CS,HIGH);
    digitalWrite(SHUTDOWN,HIGH);

    digitalWrite(GPS_ON,LOW);
    Serial.begin(115200);
    ss.begin(GPSBaud);
    Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  

}


void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


