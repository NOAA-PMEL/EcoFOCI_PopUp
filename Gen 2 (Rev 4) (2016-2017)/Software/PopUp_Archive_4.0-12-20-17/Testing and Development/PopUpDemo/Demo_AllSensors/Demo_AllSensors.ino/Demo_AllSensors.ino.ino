#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <IridiumSBD.h>
#include <ctype.h>
#include <Wire.h>
#include <SPI.h>  
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_MCP9808.h>

#define SDLibPin  10
#define SDCardPin  8
#define Iridium_TXPin 7
#define Iridium_RXPin 6
#define Iridium_OnOff 5
#define ADXL_CSPin 4
#define GPS_RXPin 3
#define GPS_TXPin 2
#define GPS_FIXPin A0
#define ledPin A2

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;
File myFile;


SoftwareSerial ss(GPS_RXPin, GPS_TXPin);            // The serial connection to the GPS device
SoftwareSerial nss(Iridium_RXPin, Iridium_TXPin);   // The serial connection to the Iridium device
IridiumSBD isbd(nss, Iridium_OnOff);

float averageTilt=0;
float maxTilt=100;
double averageTemp=0;
float averageLux=0;
int numLoops=0;

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

long waitTimeForFix=60;
double gpsFixLat=0;
double gpsFixLon=0;
int gpsFixHDOP=32767;
unsigned int gpsFixYear=0;
byte gpsFixMonth=0;
byte gpsFixDay=0;
byte gpsFixHour=0;
byte gpsFixMinute=0;
byte gpsFixSecond=0;
boolean foundFix=false;

void setup()  {
  pinMode(SDLibPin,OUTPUT);
  pinMode(ledPin, OUTPUT);
  SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
  SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
  //Serial.begin(115200);
  setupADXL345();
  setupMCP9808();
  setupTSL2561();
  ss.begin(GPSBaud);
  numLoops = readSensors();
}

void loop()  {
  while (ss.available()>0&&millis()<waitTimeForFix*1000)
    if (gps.encode(ss.read())&&gps.time.isUpdated())
      logInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)  {
    //Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  if(gps.location.isValid()&&gps.date.isValid()&&gps.time.isValid()){
    foundFix=true;
    if(gps.hdop.value()<=gpsFixHDOP){
      gpsFixHDOP=gps.hdop.value();
      gpsFixLat=gps.location.lat();
      gpsFixLon=gps.location.lng();
      gpsFixYear=gps.date.year();
      gpsFixMonth=gps.date.month();
      gpsFixDay=gps.date.day();
      gpsFixHour=gps.time.hour();
      gpsFixMinute=gps.time.minute();
      gpsFixSecond=gps.time.second();
    }
  }

  if(foundFix){
    ss.end();
    sendSBD();
  }
}

void sendSBD(){
  nss.begin(19200);              //start serial comms on nss pins defined above
  isbd.attachConsole(Serial);    //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only) 
  isbd.setPowerProfile(1);       //1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.begin();                  //wake up the 9602 and prepare it to communicate

  int signalQuality = -1;
  int err = isbd.getSignalQuality(signalQuality);
  //Serial.print(F("Iridium Signal quality is "));
  //Serial.println(signalQuality);

  char dataBuffer[70];
  fillDataBuffer(dataBuffer,70);
  err = isbd.sendSBDText(dataBuffer);
  
  if (err != 0) {
    //Serial.print(F("sendSBD failed: error "));
    //Serial.println(err);
    return;
  }
  //Serial.println(F("Iridium Message Sent!"));
  //Serial.print(F("Messages left: "));
  //Serial.println(isbd.getWaitingMessageCount());
  while(true);
}


void fillDataBuffer(char dataBuffer[], byte len){
  char strLat[8];
  dtostrf(gpsFixLat,7,4,strLat);
  char strLon[10];
  dtostrf(gpsFixLon,9,4,strLon);
  char strTmp[7];
  dtostrf(averageTemp,5,2,strTmp);
  char strMaxTilt[7];
  dtostrf(maxTilt,6,4,strMaxTilt);
  char strAveTilt[7];
  dtostrf(averageTilt,6,4,strAveTilt);
  char strAveLux[10];
  dtostrf(averageLux,4,1,strAveLux);
  sprintf(dataBuffer,"%02i/%02i/%02i, %02i:%02i:%02i, %s*, %s*, %s*C, %s, %s max, %s lux",
          gpsFixMonth,gpsFixDay,gpsFixYear,
          gpsFixHour,gpsFixMinute,gpsFixSecond,
          strLat,strLon,
          strTmp,strAveTilt,strMaxTilt,strAveLux);
    
}

void logInfo() {
  //Serial.print(F("Location: ")); 
  if (gps.location.isValid()){
    //Serial.print(gps.location.lat(), 6);
    //Serial.print(F(","));
    //Serial.print(gps.location.lng(), 6);
  }
  else{
    //Serial.print(F("INVALID"));
  }

  //Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()){
    //Serial.print(gps.date.month());
    //Serial.print(F("/"));
    //Serial.print(gps.date.day());
    //Serial.print(F("/"));
    //Serial.print(gps.date.year());
  }
  else{
    //Serial.print(F("INVALID"));
  }

  //Serial.print(F(" "));
  if (gps.time.isValid()){
    if (gps.time.hour() < 10){} //Serial.print(F("0"));
    //Serial.print(gps.time.hour());
    //Serial.print(F(":"));
    if (gps.time.minute() < 10){} //Serial.print(F("0"));
    //Serial.print(gps.time.minute());
    //Serial.print(F(":"));
    if (gps.time.second() < 10){} //Serial.print(F("0"));
    //Serial.print(gps.time.second());
    //Serial.print(F("."));
    if (gps.time.centisecond() < 10){} //Serial.print(F("0"));
    //Serial.print(gps.time.centisecond());
  }
  else{
    //Serial.print(F("INVALID"));
  }
  //Serial.println();
}

int readSensors(){
  int numSeconds=10;  
  //Serial.print(F("Reading sensors for "));
  //Serial.print(numSeconds);
  //Serial.println(F(" seconds... Please wait"));
  long startTime=millis();
  float tiltCos = 0;
  float temp = 0;
  float lux = 0;
  averageTilt = 0;
  maxTilt = 100;
  averageTemp = 0;
  averageLux = 0;
  int numLoops = 0;
  while(millis()<(startTime+1000*numSeconds)){
    tiltCos = readADXL345(); 
    if(tiltCos<maxTilt) maxTilt=tiltCos;
    temp = readMCP9808();
    lux = readTSL2561();
    averageTilt+=tiltCos;
    averageTemp+=temp;
    averageLux+=lux;
    numLoops++;
    //Serial.println();
  }
  averageTilt/=numLoops;
  averageTemp/=numLoops;
  averageLux/=numLoops;
  //Serial.print(F("Max Tilt=\t"));          
  //Serial.println(maxTilt,4);
  //Serial.print(F("Average Tilt=\t"));      
  //Serial.println(averageTilt,4);
  //Serial.print(F("Average Temp=\t"));      
  //Serial.println(averageTemp,2);
  //Serial.print(F("Average Lux=\t"));       
  //Serial.println(averageLux,1);   
  //Serial.print(F("Num. of Loops=\t"));   
  //Serial.println(numLoops);   
  return numLoops;
}

float readTSL2561(){
  sensors_event_t event;
  tsl.getEvent(&event);
  //Serial.print(event.light); Serial.println(F(" lux"));
  return event.light;
}

void setupTSL2561(){
  tsl.begin();

  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

}

float readMCP9808(){
  float c = tempsensor.readTempC();
  //Serial.print(F("Temp: ")); Serial.print(c); Serial.println(F("*C")); 
  return c;
}

void setupMCP9808(){
  tempsensor.begin();
  tempsensor.shutdown_wake(0);             // Don't remove this line! required before reading temp
}

float readADXL345(){
  SPI.setDataMode(SPI_MODE3);             //Set SPI Data Mode
  digitalWrite(ADXL_CSPin, LOW);
  SPI.transfer(0x32 | 0xC0);
  byte x0 = SPI.transfer(-1);  
  byte x1 = SPI.transfer(-1);
  byte y0 = SPI.transfer(-1);  
  byte y1 = SPI.transfer(-1);
  byte z0 = SPI.transfer(-1);  
  byte z1 = SPI.transfer(-1);
  digitalWrite(ADXL_CSPin, HIGH);

  float x = x1<<8 | x0;
  float y = y1<<8 | y0;
  float z = z1<<8 | z0;
  float xg = x*3.9/1000;
  float yg = y*3.9/1000;
  float zg = z*3.9/1000;
  float gForce=sqrt(sq(xg)+sq(yg)+sq(zg));
  float tiltCos=zg/gForce;
  //Serial.print(F("Tilt Angle="));   Serial.println(tiltCos,4);  
  return tiltCos;
}


void setupADXL345(){
  pinMode(ADXL_CSPin,OUTPUT);
  digitalWrite(ADXL_CSPin,HIGH);
  SPI.setDataMode(SPI_MODE3);             //Set SPI Data Mode
  sendSPIdata(ADXL_CSPin,0x2C,B00001010);
  sendSPIdata(ADXL_CSPin,0x31,B00001011);
  //sendSPIdata(ADXL_CSPin,0x38,B10000000);  //FIFO Mode 'Stream'
  sendSPIdata(ADXL_CSPin,0x2D,B00001000);
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}

