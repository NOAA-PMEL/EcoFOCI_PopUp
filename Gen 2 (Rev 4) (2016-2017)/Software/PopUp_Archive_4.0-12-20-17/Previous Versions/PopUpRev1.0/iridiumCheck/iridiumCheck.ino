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
#define IRIDIUM_RXD A14
#define IRIDIUM_TXD A15
#define SDA 20
#define SCL 21


SoftwareSerial ss(GPS_TX,GPS_RX);
SoftwareSerial nss(IRIDIUM_TXD,IRIDIUM_RXD);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);
static const int ledPin = 13;

//static const uint32_t GPSBaud = 9600;
//TinyGPSPlus gps;


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
    
    digitalWrite(A0,HIGH);
    digitalWrite(TEMP_SWITCH,LOW);
    digitalWrite(SENSOR_POWER_5V,HIGH);
    digitalWrite(SENSOR_POWER_3V3,HIGH);
    digitalWrite(GPS_ON,LOW);
    digitalWrite(ACCEL_CS,HIGH);
    digitalWrite(SDCARD_CS,HIGH);
    digitalWrite(RTC_CS,HIGH);
    digitalWrite(IRIDIUM_POWER,HIGH);
    digitalWrite(IRIDIUM_ON_OFF,LOW);

  
  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);
  
  digitalWrite(IRIDIUM_POWER,LOW);
  digitalWrite(IRIDIUM_ON_OFF,HIGH);
    
  Serial.begin(115200);
  nss.begin(19200);  //start serial comms on nss pins defined above

  isbd.attachConsole(Serial); //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only) 
  isbd.setPowerProfile(1); //1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.begin();  //wake up the 9602 and prepare it to communicate
  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

  err = isbd.sendSBDText("Pop Up Rig Test 128");
  if (err != 0)
  {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    return;
  }

  Serial.println("Hey, it worked!");
  Serial.print("Messages left: ");
  Serial.println(isbd.getWaitingMessageCount());
}

void loop()
{
   digitalWrite(ledPin, HIGH);
}

bool ISBDCallback()
{
   digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
   return true;
}
