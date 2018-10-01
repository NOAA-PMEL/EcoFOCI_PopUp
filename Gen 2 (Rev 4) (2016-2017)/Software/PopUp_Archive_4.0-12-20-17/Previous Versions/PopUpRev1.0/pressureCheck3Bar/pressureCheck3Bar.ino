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

SoftwareSerial ss(GPS_RX,GPS_TX);
SoftwareSerial nss(IRIDIUM_RX,IRIDIUM_TX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);

#define keller3bar 0x40       //Define pressure sensor control register
#define checkpressure 0xAC       //Define pressure sensor control register


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
    
    digitalWrite(SENSOR_POWER_5V,LOW);
    digitalWrite(SENSOR_POWER_3V3,LOW);
    Wire.begin();
    Serial.begin(9600);
 }


void loop(){
  PressureRead();
  delay(3000);
}



void PressureRead(){
  char p3status;
  unsigned int p3;
  unsigned int t3;
  int eocDelay=8;
  Wire.beginTransmission(keller3bar);  
  Wire.write(checkpressure);  
  Wire.endTransmission();
  delay(eocDelay);                         //give sensor time to take measurement (8ms specified in protocol)
  Wire.requestFrom(keller3bar,5);  //Take sample from pressure sensor
  while(Wire.available()){ // ensure all the data comes in
    p3status = Wire.read(); 
    p3 = Wire.read(); 
    Serial.print("P3msb=");
    Serial.println(p3,BIN);    
    p3 = ((unsigned int)p3 << 8);
    Serial.print("P3shf=");
    Serial.println(p3,BIN);
    p3 += Wire.read(); // low byte
    Serial.print("P3end=");
    Serial.println(p3,BIN);
    t3 = Wire.read(); 
    Serial.print("t3msb=");
    Serial.println(t3,BIN);    
    t3 = ((unsigned int)t3 << 8);
    Serial.print("t3shf=");
    Serial.println(t3,BIN);
    t3 += Wire.read(); // low byte
    Serial.print("t3end=");
    Serial.println(t3,BIN);
  }
  Wire.endTransmission();

  Serial.println((keller3bar<<1|1), BIN);
  Serial.println(checkpressure, BIN);
  Serial.print("Status: ");
  Serial.println(p3status,BIN);
  Serial.print("P     : ");
  Serial.println(p3,DEC);
  Serial.print("T     : ");
  Serial.println(t3,DEC);
  Serial.println();
}


