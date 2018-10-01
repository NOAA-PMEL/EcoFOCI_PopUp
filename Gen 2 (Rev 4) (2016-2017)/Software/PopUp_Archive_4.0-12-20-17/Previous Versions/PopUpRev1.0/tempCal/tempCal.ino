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

const int sampleinterval = 3600; 

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

RTC_DS3234 RTC(RTC_CS);
DateTime alarmtime;
DateTime firstAlarm;

SoftwareSerial ss(GPS_RX,GPS_TX);
SoftwareSerial nss(IRIDIUM_RX,IRIDIUM_TX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);

long stamp;

#define ads1100A0 0x48       //Define ADC control register

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
    digitalWrite(SENSOR_POWER_5V,HIGH);
    digitalWrite(SENSOR_POWER_3V3,HIGH);
    digitalWrite(GPS_ON,HIGH);
    digitalWrite(IRIDIUM_POWER,HIGH);
    digitalWrite(IRIDIUM_ON_OFF,LOW);
    digitalWrite(ACCEL_CS,HIGH);
    digitalWrite(SDCARD_CS,HIGH);
    digitalWrite(RTC_CS,HIGH);
    digitalWrite(SHUTDOWN,HIGH);
    
    digitalWrite(SENSOR_POWER_5V,LOW);
    digitalWrite(SENSOR_POWER_3V3,LOW);
    Wire.begin();
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setDataMode(SPI_MODE2);              //Configure SPI Communication (needed for various ICs)
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    //Serial.begin(9600);
    setupRTC();
    setupSD();
    
    ADCSetupTemp();
    stamp = millis();
 }


void loop(){
  digitalWrite(TEMP_SWITCH,HIGH);
  while(stamp>millis()-1000){}
  stamp = millis();
  int measure_VISH = ADCRead();
  
  digitalWrite(TEMP_SWITCH,LOW);
  while(stamp>millis()-1000){}
  stamp=millis();
  int measure_THERM = ADCRead();
  
  const int len = 32;                          //buffer length for RTC display on Serial comm
  static char buf[len];                        //string for RTC display on Serial comm
  DateTime now = RTC.now();                    //Get time from RTC
  
  sdCardWriteTime(now);
  File myFile = SD.open("realcal.txt", FILE_WRITE);
  myFile.print(measure_VISH);
  myFile.print(",");
  myFile.println(measure_THERM);
  myFile.close();
  
  //Serial.print(now.toString(buf,len));          
  //Serial.print(",");
  //Serial.print(measure_VISH);
  //Serial.print(",");
  //Serial.println(measure_THERM);
}


void sdCardWriteTime(DateTime timestamp){
  SPI.setClockDivider(SPI_CLOCK_DIV16);               //Set SPI Clock Speed
  SPI.setDataMode(SPI_MODE0);                         //Configure SPI Communication (needed for various ICs)
  File dataFile = SD.open("realcal.txt", FILE_WRITE);    //open the file with write permissions
  if (dataFile) {                                     //if the file is available, write to it:
    dataFile.print(timestamp.month());                //Write the Month
    dataFile.print(F("/"));                           // '/'
    dataFile.print(timestamp.day());                  //Write the Day
    dataFile.print(F("/"));                           // '/'
    dataFile.print(timestamp.year());                 //Write the Month
    dataFile.print(F(","));                           // ,
    dataFile.print(timestamp.hour());                 //Write the Month
    dataFile.print(F(":"));                           // ,
    dataFile.print(timestamp.minute());               //Write the Month
    dataFile.print(F(":"));                           // ,
    dataFile.print(timestamp.second());               //Write the Second
    dataFile.print(F(","));                           // ,
    dataFile.close();                                 //close the file (also makes sure all data in buffer is written)
  }  
    //if file doesn't open, do nothing
}

void setupRTC(){
  SPI.setClockDivider(SPI_CLOCK_DIV16);                      //Set SPI Clock Speed
  SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode
  RTC.begin();                                               //Start RTC (defined by RTC instance in global declarations)
  RTC.adjust(DateTime(__DATE__, __TIME__));
  DateTime now = RTC.now();                                  //Get time from RTC
  firstAlarm =  DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);         //Calculate when the unit should first wake up
  alarmtime = (now.unixtime() + sampleinterval-1);           //Calculate next Alarm time
  setAlarmTime(alarmtime);                                   //Set next Alarm
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

void ADCSetupTemp(){
  Wire.beginTransmission(ads1100A0);
  Wire.write(B10001101); //set ADC gain to 2, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(500);
}

int ADCRead(){
  byte controlRegister =0;
  int adcVal = 0;
  Wire.requestFrom(ads1100A0, 3);  //Take sample from ADC
      while(Wire.available()){ // ensure all the data comes in
        adcVal = Wire.read(); // high byte * B11111111
        adcVal = ((unsigned int)adcVal << 8);
        adcVal += Wire.read(); // low byte
        controlRegister = Wire.read();
      }
      return adcVal;
}

void setupSD(){
  SD.begin(SDCARD_CS);     //Start SD Card so it is available for other function calls       
}


