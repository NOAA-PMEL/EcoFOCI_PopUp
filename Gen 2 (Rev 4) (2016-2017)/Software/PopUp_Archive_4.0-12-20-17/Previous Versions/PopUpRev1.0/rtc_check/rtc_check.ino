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

const byte MON = 2;        
const byte DAY = 29;
const byte YEAR = 16;

const byte HOUR = 13;    
const byte MIN = 0;
const byte SEC = 0;

const int sampleinterval = 32000;   //sample interval in seconds

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
    Serial.begin(115200);                      //Turn the Serial Protocol ON
    setupRTC();
    timeCheck();    
    Serial.println(F("Set up Complete!!"));
    delay(2000); 
}


void loop(){
      //"OPTIONS" Menu
      rtcCheck();
      delay(5000);
}



void setupRTC(){
  SPI.setClockDivider(SPI_CLOCK_DIV16);                      //Set SPI Clock Speed
  SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode
  RTC.begin();                                               //Start RTC (defined by RTC instance in global declarations)
  DateTime now = RTC.now();                                  //Get time from RTC
  firstAlarm =  DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);  //Calculate when the unit should first wake up
  alarmtime = (now.unixtime() + sampleinterval-1);           //Calculate next Alarm time
  setAlarmTime(alarmtime);                                   //Set next Alarm
}

void rtcCheck(){
    SPI.setClockDivider(SPI_CLOCK_DIV16);        //Set SPI Clock Speed
    SPI.setDataMode(SPI_MODE2);                  //Set SPI Data Mode
    const int len = 32;                          //buffer length for RTC display on Serial comm
    static char buf[len];                        //string for RTC display on Serial comm
    DateTime now = RTC.now();                    // Get time from RTC
    Serial.print(F("***Current RTC Time: "));
    Serial.println(now.toString(buf,len));        
    Serial.print(F("***Alarm Time:       "));
    Serial.print(alarmtime.toString(buf,len));        
    Serial.println(F("   (Time of Program load + Sample Interval)"));
    Serial.print(F("---Unit Start Time:  "));
    Serial.print(firstAlarm.toString(buf,len));        
    Serial.println(F("   (Unit will only sample after this date)"));
    Serial.println(); 
}


void timeCheck(){
    SPI.setClockDivider(SPI_CLOCK_DIV16);                            //Set SPI Clock Speed
    SPI.setDataMode(SPI_MODE2);                                      //Set SPI Data Mode    
    RTC.adjust(DateTime(__DATE__, __TIME__));
    Serial.println(F("***Time Synchronized with System Clock!"));
    DateTime now = RTC.now();                                        // Get time from RTC
    alarmtime = (now.unixtime() + sampleinterval);                   // Calculate next Alarm time
    setAlarmTime(alarmtime);                                         // Set next Alarm
    rtcCheck();    
} 


void setAlarmTime(DateTime alarmtime){
  RTCWrite(0x87,alarmtime.second() & 0x7F);  //set alarm time: seconds     //87=write to location for alarm seconds    //binary & second with 0x7F required to turn alarm second "on"
  RTCWrite(0x88,alarmtime.minute() & 0x7F);  //set alarm time: minutes     //88=write to location for alarm minutes    //binary & minute with 0x7F required to turn alarm minute "on"
  RTCWrite(0x89,alarmtime.hour() & 0x7F);    //set alarm time: hour        //89=write to location for alarm hour       //binary & hour with 0x7F required to turn alarm hour "on"
  RTCWrite(0x8A,alarmtime.day() & 0x3F);     //set alarm time: day         //8A=write to location for alarm day        //binary & day with 0x3F required to turn alarm day "on" (not dayofWeek) 
  RTCWrite(0x8B,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8C,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8D,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8F,B00000000);                  //reset flags                //8F=write to location for control/status flags    //B00000000=Ocillator Stop Flag 0, No Batt Backed 32 KHz Output, Keep Temp CONV Rate at 64 sec (may change later), disable 32 KHz output, temp Not Busy, alarm 2 not tripped, alarm 1 not tripped
  RTCWrite(0x8E,B00000101);                  //set control register       //8E=write to location for control register        //B01100101=Oscillator always on, SQW on, Convert Temp off, SQW freq@ 1Hz, Interrupt enabled, Alarm 2 off, Alarm 1 on
}

void RTCWrite(char reg, char val){
  SPI.setClockDivider(SPI_CLOCK_DIV16);      //Set SPI Clock Speed
  SPI.setDataMode(SPI_MODE2);                //Set SPI Data Mode
  digitalWrite(RTC_CS, LOW);                 //enable SPI read/write for chip
  SPI.transfer(reg);                         //define memory register location
  SPI.transfer(bin2bcd(val));                //write value
  digitalWrite(RTC_CS, HIGH);                //disable SPI read/write for chip
  delay(5);                                  //delay 5 ms to make sure chip is off
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}
