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

#define TEMP_SWITCH 3
#define SENSOR_POWER_5V 6
#define SENSOR_POWER_3V3 22
#define GPS_POWER 9
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

SoftwareSerial ss(GPS_RX,GPS_TX);
SoftwareSerial nss(IRIDIUM_RX,IRIDIUM_TX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);

#define keller3bar 0x40       //Define pressure sensor control register
#define keller10bar 0x41      //Define pressure sensor control register
#define checkpressure 0xAC    //Define pressure sensor control register

void setup(){
    pinMode(SHUTDOWN,OUTPUT);
    digitalWrite(SHUTDOWN,HIGH);

    pinMode(TEMP_SWITCH,OUTPUT);
    digitalWrite(TEMP_SWITCH,LOW);

    pinMode(SENSOR_POWER_5V,OUTPUT);
    digitalWrite(SENSOR_POWER_5V,HIGH);
    
    pinMode(SENSOR_POWER_3V3,OUTPUT);
    digitalWrite(SENSOR_POWER_3V3,HIGH);
    
    pinMode(GPS_POWER,OUTPUT);
    digitalWrite(GPS_POWER,HIGH);

    pinMode(IRIDIUM_POWER,OUTPUT);
    digitalWrite(IRIDIUM_POWER,HIGH);
    
    pinMode(IRIDIUM_ON_OFF,OUTPUT);
    digitalWrite(IRIDIUM_ON_OFF,LOW);

    pinMode(ACCEL_CS,OUTPUT);
    digitalWrite(ACCEL_CS,HIGH);

    pinMode(SDCARD_CS,OUTPUT);
    digitalWrite(SDCARD_CS,HIGH);
    
    digitalWrite(RTC_CS,HIGH);
    pinMode(RTC_CS,OUTPUT);
    
    digitalWrite(SENSOR_POWER_5V,LOW);
    digitalWrite(SENSOR_POWER_3V3,LOW);
    Wire.begin();
    Serial.begin(9600);
 }


void loop(){
  delay(3000);
  PressureChangeADDR();
  Serial.println("Change complete");
  delay(100000);
}

void PressureChangeADDR(){
  Serial.println("Entering Command Mode...");
  
  Wire.beginTransmission(0x40);  
  Wire.write(0xA9);  
  Wire.endTransmission();  

  delay(50);
  Serial.println("Changing Address to 0x41...");
  Wire.beginTransmission(0x40);  
  Wire.write(0x42);  
  Wire.write(0x00);
  Wire.write(0x41);  
  Wire.endTransmission();  
  delay(50);
}




