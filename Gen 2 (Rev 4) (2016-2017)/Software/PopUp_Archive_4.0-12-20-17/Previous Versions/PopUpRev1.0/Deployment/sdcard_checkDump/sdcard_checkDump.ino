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

const int sampleinterval = 3600;   //sample interval in seconds

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

RTC_DS3234 RTC(RTC_CS);
DateTime alarmtime;
DateTime firstAlarm;


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
    SPI.setDataMode(SPI_MODE0);              //Configure SPI Communication (needed for various ICs)
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    Serial.begin(115200);                      //Turn the Serial Protocol ON
    setupSD();
    Serial.println(F("Set up Complete!!"));
    delay(2000); 
}


void loop(){
      //"OPTIONS" Menu
      sdcardDump();
      delay(100000);
}


void setupSD(){
   SD.begin(SDCARD_CS);                                       //Start SD Card so it is available for other function calls 
}


void sdcardDump(){
//    double gpsLat=47.2442;
//    double gpsLon=-123.2442;
//    unsigned long gpsTimeVal=12345678;
//    unsigned long gpsDateVal=345678;
//    String sendGPSFix = "";
//    sendGPSFix += String(gpsLat,4);
//    sendGPSFix += ",";
//    sendGPSFix += String(gpsLon,4);
//    sendGPSFix += ",";
//    sendGPSFix += gpsTimeVal;
//    sendGPSFix += ",";
//    sendGPSFix += gpsDateVal;  
    const int sendbuffer_size = 52;
    char iridiumSendBuffer[sendbuffer_size];
    
    iridiumSendBuffer[0]='h';
    iridiumSendBuffer[1]='i';
    Serial.println("Buffer=");
    Serial.println(iridiumSendBuffer);

//    sendGPSFix.toCharArray(iridiumSendBuffer,50);
//    sendGPSFix = "";
//  
    Sd2Card card;
    SPI.setClockDivider(SPI_CLOCK_DIV16);            //Set SPI Clock Speed    
    File myFile = SD.open("icedat.txt");                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println("icedat.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:

          Serial.write(myFile.read());
          
      }
      myFile.close();                                //close the file
    } 
    myFile = SD.open("icedat.txt");                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
          Serial.print("file position:");
          Serial.println(myFile.position());
          for(int i=0;i<50;i++){
            iridiumSendBuffer[i]=myFile.read();
          }
          iridiumSendBuffer[50]=0;      
          Serial.print("buffer contents:");
          Serial.println(iridiumSendBuffer);

      }
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening test.txt");      // if the file didn't open, print an error:
    }
    
}

