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
    initializePins();		                //Initialize Arduino Pins for Output and Set Voltage Levels
    Wire.begin();				//Start I2C Communication
    SPI.begin();                                //Start up SPI Communication (needed for various ICs) 
    SPI.setDataMode(SPI_MODE2);                 //Set SPI Data Mode
    SPI.setBitOrder(MSBFIRST);                  //Configure SPI Communication (needed for various ICs)
    SPI.setClockDivider(SPI_CLOCK_DIV16);       //Set SPI Clock Speed
    Serial.begin(115200);                       //Turn the Serial Protocol ON
    turnOn3v3SensorPower;
    turnOn5vSensorPower;
    delay(3000);
    PressureChangeADDR();
    delay(100000);
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

void loop(){
}



void PressureChangeADDR(){
  Serial.println(F("Starting Protocol to Change Slave Address..."));
  delay(1000);  
  Serial.println(F("Turning Off Sensors..."));
  turnOff3v3SensorPower();
  turnOff5vSensorPower();
  delay(1000);
  Serial.println(F("Turning On Sensors..."));
  turnOn3v3SensorPower();
  turnOn5vSensorPower();
  delay(1000);
  
  Serial.println(F("Entering Command Mode..."));
  Wire.beginTransmission(0x40);  
  Serial.println(F("Transmission Begun"));
  
  Wire.write(0xA9);  
  Serial.println(F("Command Mode Started"));
  Wire.endTransmission();  

  Serial.println(F("Transmission Ended"));
  delay(50);
  Serial.println(F("Changing Address to 0x41..."));
  Wire.beginTransmission(0x40);  
  Wire.write(0x42);  
  Wire.write(0x00);
  Wire.write(0x41);  
  Wire.endTransmission();  
  delay(50);
  Serial.println(F("Change complete.  Disconnect from power and upload new sketch."));
}

void turnOn3v3SensorPower(){
  digitalWrite(SENSOR_POWER_3V3,LOW);  		//Pin LOW=Power ON (P-Channel switch)
}
void turnOff3v3SensorPower(){
  digitalWrite(SENSOR_POWER_3V3,HIGH);    	//Pin HIGH=Power OFF (P-Channel switch)
}
void turnOn5vSensorPower(){
  digitalWrite(SENSOR_POWER_5V,LOW);  		//Pin LOW=Power ON (P-Channel switch)
}
void turnOff5vSensorPower(){
  digitalWrite(SENSOR_POWER_3V3,HIGH);  	//Pin HIGH=Power OFF (P-Channel switch)
}



