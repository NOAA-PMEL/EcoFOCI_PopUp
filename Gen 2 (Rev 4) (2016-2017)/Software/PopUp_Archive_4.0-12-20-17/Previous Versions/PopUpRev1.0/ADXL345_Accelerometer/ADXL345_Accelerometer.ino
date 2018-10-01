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
 
    Wire.begin();
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    SPI.setClockDivider(SPI_CLOCK_DIV16);    //Set SPI Clock Speed    
    SPI.setDataMode(SPI_MODE3);                                //Set SPI Data Mode
    Serial.begin(9600);                      //Turn the Serial Protocol ON
    Serial.println(F("**Setting Up ADXL345 Accelerometer"));
    setupADXL();
    Serial.println(F("Set up Complete!!"));
    delay(2000);
}


void loop(){
      //"OPTIONS" Menu
      
      readADXL();
      delay(2000);
}


void setupADXL(){
  sendSPIdata(ACCEL_CS,0x2C,B00001010);
  sendSPIdata(ACCEL_CS,0x31,B00001011);
  sendSPIdata(ACCEL_CS,0x2D,B00001000);
  
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}

void readADXL(){
  digitalWrite(ACCEL_CS, LOW);
  SPI.transfer(0x32 | 0xC0);
  byte x0 = SPI.transfer(-1);
  byte x1 = SPI.transfer(-1);
  byte y0 = SPI.transfer(-1);
  byte y1 = SPI.transfer(-1);
  byte z0 = SPI.transfer(-1);
  byte z1 = SPI.transfer(-1);
  digitalWrite(ACCEL_CS, HIGH);
//  Serial.print(x0,BIN);
//  Serial.print("-");
//  Serial.println(x1,BIN);
//  Serial.print(y0,BIN);
//  Serial.print("-");
//  Serial.println(y1,BIN);
//  Serial.print(z0,BIN);
//  Serial.print("-");
//  Serial.println(z1,BIN);
  
  float x = x1<<8 | x0;
  float y = y1<<8 | y0;
  float z = z1<<8 | z0;
//  Serial.println();
//  Serial.print(F("X="));
//  Serial.println(x);
//  Serial.print(F("Y="));
//  Serial.println(y);
//  Serial.print(F("Z="));
//  Serial.println(z);

  float xg = x*3.9/1000;
  float yg = y*3.9/1000;
  float zg = z*3.9/1000;
  //Serial.println();
  //Serial.print(F("Xg="));
  //Serial.println(xg,4);
  //Serial.print(F("Yg="));
  //Serial.println(yg,4);
  //Serial.print(F("Zg="));
  //Serial.println(zg,4);
  
  float gForce=sqrt(sq(xg)+sq(yg)+sq(zg));
  float tiltCos=yg/gForce;
  
  Serial.print(F("G Force = "));      
  Serial.print(gForce,4);
  Serial.print(F(", Inverse Cos of Tilt Angle = "));      
  Serial.println(tiltCos,4);

  
}
