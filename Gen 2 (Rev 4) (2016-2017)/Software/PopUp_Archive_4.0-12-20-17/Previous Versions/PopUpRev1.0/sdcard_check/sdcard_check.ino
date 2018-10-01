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
    SPI.setDataMode(SPI_MODE2);              //Configure SPI Communication (needed for various ICs)
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    Serial.begin(9600);                      //Turn the Serial Protocol ON
    setupSD();
    Serial.println(F("Set up Complete!!"));
    delay(2000); 
}


void loop(){
      //"OPTIONS" Menu
      sdcardCheck();
      delay(5000);
}


void setupSD(){
   SD.begin(SDCARD_CS);                                       //Start SD Card so it is available for other function calls 
}


void sdcardCheck(){
    Sd2Card card;
    if (!card.init(SPI_HALF_SPEED, SDCARD_CS)) {
      Serial.println();
      Serial.println(F("***Initialization FAILED. Is a card is inserted? "));
      return;
    } 
    else {
      Serial.println();
      Serial.println(F("***SD Card Initialized successfully.  Card is present and communicating properly.")); 
    }        
    
    SPI.setClockDivider(SPI_CLOCK_DIV16);            //Set SPI Clock Speed
    
    File myFile = SD.open("pro.txt", FILE_WRITE);   //open test.txt
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.print("Writing to pro.txt...");        //write to serial window 
      myFile.println("testing 1, 2, 3.");            //write to file
      myFile.close();                                //close the file
      Serial.println("done.");                       //write to serial window
    } 
    else {                                           //if the file didn't open, print an error:
      Serial.println("error opening test.txt");      //write to serial window
    }
    
    myFile = SD.open("pro.txt");                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println("pro.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening test.txt");      // if the file didn't open, print an error:
    }
    SPI.setDataMode(SPI_MODE2);                      //Reset SPI Communication to Mode 2
}

