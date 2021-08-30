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
#include <MemoryFree.h>

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


#define keller3bar 0x40       		//Define pressure sensor control register
#define keller10bar 0x41      		//Define pressure sensor control register
#define checkpressure 0xAC    		//Define pressure sensor control register
#define ads1100A0 0x48        		//Define tempADC control register
#define ads1100A1 0x49        		//Define parADC control register

SoftwareSerial ss(GPS_TX,GPS_RX);		//Software Serial Object for GPS Communication 
IridiumSBD isbd(Serial1,IRIDIUM_ON_OFF);	//ISBD Object for Iridium Commands
TinyGPSPlus gps;			        //Object for GPS commands and data
RTC_DS3234 RTC(RTC_CS);			        //Object for RTC

DateTime wakeupTime;				//Global Variable for when unit first wakes
DateTime alarmtime;			        //Global Variable for RTC Alarm
boolean foundGPS=false; 			//Global Variable for GPS lock success
char whichFile='s';			        //Global variable to designate which file to read when sending data.  's' = summary (first message to send)
unsigned long filePosition=0;			//Global variable to designate position in file when sending data
boolean newFile=true;


void setup(){
    initializePins();
    Wire.begin();
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setDataMode(SPI_MODE0);              //Configure SPI Communication (needed for various ICs)
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    Serial.begin(115200);                      //Turn the Serial Protocol ON
    setupSD();
    delay(2000); 
}

void loop(){
}

void setupSD(){
   SD.begin(SDCARD_CS);                                       //Start SD Card so it is available for other function calls 
   Serial.println(F("Set up Complete!!"));
   SD.remove("filepos.txt");
   SD.remove("icedat.txt");
   sdcardCheck();
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
    
    File myFile = SD.open("botdat.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println("botdat.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening botdat.txt");      // if the file didn't open, print an error:
    }
    
    myFile = SD.open("icedat.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println("icedat.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening icedat.txt");      // if the file didn't open, print an error:
    }
    
    myFile = SD.open("prodat.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println("prodat.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening prodat.txt");      // if the file didn't open, print an error:
    }
  
  myFile = SD.open("summary.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println("summary.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening summary.txt");      // if the file didn't open, print an error:
    }
    
      
  myFile = SD.open("filepos.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println("filepos.txt:");                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening filepos.txt");      // if the file didn't open, print an error:
    }
  
}

void initializePins(){
  pinMode(SHUTDOWN,OUTPUT);		        //SHUTDOWN PIN for turning unit off (Sleep mode)
  digitalWrite(SHUTDOWN,HIGH);			//SHUTDOWN PIN HIGH keeps unit's power on
  pinMode(TEMP_SWITCH,OUTPUT);			//TEMP_SWITCH PIN for switching between thermistor and reference resistor
  digitalWrite(TEMP_SWITCH,HIGH);		//TEMP_SWITCH PIN HIGH for reading thermistor
  pinMode(SENSOR_POWER_5V,OUTPUT);		//SENSOR_POWER_5V PIN for turning 5V sensor power on and off
  digitalWrite(SENSOR_POWER_5V,HIGH);		//SENSOR_POWER_5V PIN HIGH for 5V sensor power off (P-Channel Transistor)
  pinMode(SENSOR_POWER_3V3,OUTPUT);		//SENSOR_POWER_3V3 PIN for turning 3.3V power on and off
  digitalWrite(SENSOR_POWER_3V3,HIGH);		//SENSOR_POWER_3V3 PIN HIGH for 3.3V sensor power off (P-Channel Transistor)
  pinMode(GPS_POWER,OUTPUT);			//GPS_POWER PIN for turning power to GPS on and off
  digitalWrite(GPS_POWER,HIGH);			//GPS_POWER PIN HIGH for GPS power off (P-Channel Transistor)
  pinMode(IRIDIUM_POWER,OUTPUT);		//IRIDIUM_POWER PIN for turning power to Iridium on and off
  digitalWrite(IRIDIUM_POWER,HIGH);		//IRIDIUM_POWER PIN HIGH for Iridium power off (P-Channel Transistor)
  pinMode(IRIDIUM_ON_OFF,OUTPUT);		//IRIDIUM_ON_OFF PIN for Iridium sleep or awake
  digitalWrite(IRIDIUM_ON_OFF,LOW);		//IRIDIUM_ON_OFF PIN LOW for Iridium sleep mode
  pinMode(ACCEL_CS,OUTPUT);			//ACCEL_CS PIN for SPI communication to Accelerometer
  digitalWrite(ACCEL_CS,HIGH);			//ACCEL_CS PIN HIGH for SPI communication to Accelerometer inactive
  pinMode(SDCARD_CS,OUTPUT);			//SDCARD_CS PIN for SPI communication to SDCard
  digitalWrite(SDCARD_CS,HIGH);    		//ACCEL_CS PIN HIGH for SPI communication to SDCard inactive
  pinMode(RTC_CS,OUTPUT);			//RTC_CS PIN for SPI communication to RTC
  digitalWrite(RTC_CS,HIGH);			//ACCEL_CS PIN HIGH for SPI communication to RTC inactive
}
