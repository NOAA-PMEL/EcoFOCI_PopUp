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

#define ads1100A1 0x49       //Define ADC control register
#define keller3bar 0x40       		//Define pressure sensor control register
#define keller10bar 0x41      		//Define pressure sensor control register
#define checkpressure 0xAC    		//Define pressure sensor control register


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
  char p10status;
  unsigned int p10;
  unsigned int t10;
  int eocDelay=8;
  delay(50);
  Wire.beginTransmission(keller10bar);  
  Wire.write(checkpressure);  
  Wire.endTransmission();
  delay(eocDelay);                         //give sensor time to take measurement (8ms specified in protocol)
  Wire.requestFrom(keller10bar,5);  //Take sample from pressure sensor
  while(Wire.available()){ // ensure all the data comes in
    p10status = Wire.read(); 
    p10 = Wire.read(); 
    Serial.print("P10msb=");
    Serial.println(p10,BIN);    
    p10 = ((unsigned int)p10 << 8);
    Serial.print("P10shf=");
    Serial.println(p10,BIN);
    p10 += Wire.read(); // low byte
    Serial.print("P10end=");
    Serial.println(p10,BIN);
    t10 = Wire.read(); 
    Serial.print("t10msb=");
    Serial.println(t10,BIN);    
    t10 = ((unsigned int)t10 << 8);
    Serial.print("t10shf=");
    Serial.println(t10,BIN);
    t10 += Wire.read(); // low byte
    Serial.print("t10end=");
    Serial.println(t10,BIN);
  }
  Wire.endTransmission();

  Serial.println((keller10bar<<1|1), BIN);
  Serial.println(checkpressure, BIN);
  Serial.print("Status: ");
  Serial.println(p10status,BIN);
  Serial.print("P     : ");
  Serial.println(p10,DEC);
  Serial.print("T     : ");
  Serial.println(t10,DEC);
  Serial.println();
}


void ADCSetup(){
  Wire.beginTransmission(ads1100A1);
  Wire.write(B10001111); //set ADC gain to 8, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(500);
}

int ADCRead(){
  byte controlRegister =0;
  int adcVal = 0;
  Wire.requestFrom(ads1100A1, 3);  //Take sample from ADC
      while(Wire.available()){ // ensure all the data comes in
        adcVal = Wire.read(); // high byte * B11111111
        adcVal = ((unsigned int)adcVal << 8);
        adcVal += Wire.read(); // low byte
        controlRegister = Wire.read();
      }
      return adcVal;
}


