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

SoftwareSerial ss(GPS_RX,GPS_TX);
SoftwareSerial nss(IRIDIUM_RX,IRIDIUM_TX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);

#define keller3bar 0x40       //Define pressure sensor control register
#define checkpressure 0xAC    //Define pressure sensor control register
#define ads1100A0 0x48       //Define ADC control register
#define ads1100A1 0x49       //Define ADC control register


void setup(){
    initializePins();
    Wire.begin();
    Serial.begin(9600);
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode

    digitalWrite(SENSOR_POWER_5V,LOW);
    digitalWrite(SENSOR_POWER_3V3,LOW);   
    setupAccel();
    setupTempADC();
    setupPARADC();
}
 
void loop(){
  readPressure();
  readTemp();
  readPAR();
  readAccel();
  delay(1000);
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
 

void readPressure(){
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

  Serial.print("Status: ");
  Serial.println(p3status,BIN);
  Serial.print("P     : ");
  Serial.println(p3,DEC);
  Serial.print("T     : ");
  Serial.println(t3,DEC);
  Serial.println();
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
  Serial.print("debug");
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


void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}
