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

#define keller3bar 0x40       //Define pressure sensor control register
#define keller10bar 0x41       //Define pressure sensor control register
#define checkpressure 0xAC    //Define pressure sensor control register
#define ads1100A0 0x48        //Define tempADC control register
#define ads1100A1 0x49        //Define parADC control register

SoftwareSerial ss(GPS_TX,GPS_RX);
SoftwareSerial nss(IRIDIUM_RX,IRIDIUM_TX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);
TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
RTC_DS3234 RTC(RTC_CS);
DateTime alarmtime;



void setup(){
    initializePins();
    Wire.begin();
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode
    Serial.begin(115200);
    digitalWrite(SENSOR_POWER_5V,LOW);
    digitalWrite(SENSOR_POWER_3V3,LOW);   
    setupAccel();
    setupTempADC();
    setupPARADC();
    setupSD();
 }
 
void loop(){
  readPressure3();
  readPressure10();
  readTemp();
  readPAR();
  readAccel();
  digitalWrite(SENSOR_POWER_5V,HIGH);
  digitalWrite(SENSOR_POWER_3V3,HIGH);
  
  digitalWrite(GPS_POWER,LOW);
  getGPSFix();
  digitalWrite(GPS_POWER,HIGH);
   
}

void initializePins(){
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
 }
 
 void setupSD(){
   SD.begin(SDCARD_CS);                                       //Start SD Card so it is available for other function calls 
}

void readPressure3(){
  char p3status;
  unsigned int p3;
  unsigned int t3;
  int eocDelay=8;
  Wire.beginTransmission(keller3bar);  
  Wire.write(checkpressure);  
  Wire.endTransmission();
  delay(eocDelay);                       //give sensor time to take measurement (8ms specified in protocol)
  Wire.requestFrom(keller3bar,5);        //Take sample from pressure sensor
  while(Wire.available()){               //Ensure all the data comes in
    p3status = Wire.read(); 
    p3 = Wire.read(); 
    p3 = ((unsigned int)p3 << 8);
    p3 += Wire.read(); // low byte
    //depth in m = [(reading-16384)*maxbar/32768]*10
    t3 = Wire.read(); 
    t3 = ((unsigned int)t3 << 8);
    t3 += Wire.read(); // low byte
    //Temp = (reading>>4-24)*.05-50
  }
  Wire.endTransmission();
  Serial.print("P3  Status: ");
  Serial.print(p3status,BIN);
  Serial.print(", P     : ");
  Serial.print(p3,DEC);
  Serial.print(", T     : ");
  Serial.println(t3,DEC);
}


void readPressure10(){
  char p10status;
  unsigned int p10;
  unsigned int t10;
  int eocDelay=8;
  Wire.beginTransmission(keller10bar);  
  Wire.write(checkpressure);  
  Wire.endTransmission();
  delay(eocDelay);                       //give sensor time to take measurement (8ms specified in protocol)
  Wire.requestFrom(keller10bar,5);        //Take sample from pressure sensor
  while(Wire.available()){               //Ensure all the data comes in
    p10status = Wire.read(); 
    p10 = Wire.read(); 
    p10 = ((unsigned int)p10 << 8);
    p10 += Wire.read(); // low byte
    //depth in m = [(reading-16384)*maxbar/32768]*10
    t10 = Wire.read(); 
    t10 = ((unsigned int)t10 << 8);
    t10 += Wire.read(); // low byte
    //Temp = (reading>>4-24)*.05-50
  }
  Wire.endTransmission();
  Serial.print("P10 Status: ");
  Serial.print(p10status,BIN);
  Serial.print(", P     : ");
  Serial.print(p10,DEC);
  Serial.print(", T     : ");
  Serial.println(t10,DEC);
}

void setupTempADC(){
  Wire.beginTransmission(ads1100A0);
  Wire.write(B10001101); //set ADC gain to 2, 15 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(500);
}

void readTemp(){
  int delaytime=500;
  digitalWrite(TEMP_SWITCH,LOW);
  delay(delaytime);
  int measurement = readTempADC();
  Serial.print("Reference ADC = ");
  Serial.print(measurement);
  digitalWrite(TEMP_SWITCH,HIGH);
  delay(delaytime);
  int measurement2 = readTempADC();
  Serial.print(", Thermistor ADC = ");
  Serial.println(measurement2); 
}

int readTempADC(){
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

void setupPARADC(){
  Wire.beginTransmission(ads1100A1);
  Wire.write(B10001100); //set ADC gain to 8, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(500);
}

void readPAR(){
  int measurement = readPARADC();
  Serial.print("PAR ADC = ");
  Serial.println(measurement);
  delay(500);
}

int readPARADC(){
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

void setupAccel(){
  SPI.setDataMode(SPI_MODE3);                                //Set SPI Data Mode
  sendSPIdata(ACCEL_CS,0x2C,B00001010);
  sendSPIdata(ACCEL_CS,0x31,B00001011);
  sendSPIdata(ACCEL_CS,0x2D,B00001000);
  SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode
  }


void readAccel(){
  SPI.setDataMode(SPI_MODE3);                                //Set SPI Data Mode
  digitalWrite(ACCEL_CS, LOW);
  SPI.transfer(0x32 | 0xC0);
  byte x0 = SPI.transfer(-1);
  byte x1 = SPI.transfer(-1);
  byte y0 = SPI.transfer(-1);
  byte y1 = SPI.transfer(-1);
  byte z0 = SPI.transfer(-1);
  byte z1 = SPI.transfer(-1);
  digitalWrite(ACCEL_CS, HIGH);
  SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode
  
  float x = x1<<8 | x0;
  float y = y1<<8 | y0;
  float z = z1<<8 | z0;
  float xg = x*3.9/1000;
  float yg = y*3.9/1000;
  float zg = z*3.9/1000;
  float gForce=sqrt(sq(xg)+sq(yg)+sq(zg));
  float tiltCos=yg/gForce;
  
  Serial.print(F("G Force = "));      
  Serial.print(gForce,4);
  Serial.print(F(", Inverse Cos of Tilt Angle = "));      
  Serial.println(tiltCos,4);
}

void getGPSFix(){
  ss.begin(GPSBaud);
  long maxGPSFixTime = 60;    //maximum time to look for gps signal in seconds
  long fixStartTime = millis();
  while(millis()<fixStartTime+maxGPSFixTime*1000){  //
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        displayInfo();
    if (millis() > (5000+fixStartTime) && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      return;
    }
  }
  ss.end();
}


void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}
