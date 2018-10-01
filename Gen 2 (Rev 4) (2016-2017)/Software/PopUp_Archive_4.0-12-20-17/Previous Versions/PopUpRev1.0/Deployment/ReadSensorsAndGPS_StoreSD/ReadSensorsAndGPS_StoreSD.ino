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
#define ledPin 13

#define keller3bar 0x40       //Define pressure sensor control register
#define keller10bar 0x41      //Define pressure sensor control register
#define checkpressure 0xAC    //Define pressure sensor control register
#define ads1100A0 0x48        //Define tempADC control register
#define ads1100A1 0x49        //Define parADC control register

SoftwareSerial ss(GPS_TX,GPS_RX);
SoftwareSerial nss(IRIDIUM_TX,IRIDIUM_RX);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);
TinyGPSPlus gps;
RTC_DS3234 RTC(RTC_CS);
DateTime wakeupTime;
DateTime releaseTime =  DateTime(17,2,14,12,0,0);                  //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime sendDataDate = DateTime(17,6,1,12,0,0);                   //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime iceFreeDate =  DateTime(17,9,1,12,0,0);                   //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
boolean foundGPS=false; 
//unsigned long gpsDateVal=0;
//unsigned long gpsTimeVal=0;
//double gpsLat=0;
//double gpsLon=0;
//float maxtiltInvCos=0;

void setup(){
    initializePins();
    Wire.begin();
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    SPI.setDataMode(SPI_MODE2);              //Set SPI Data Mode
    Serial.begin(115200);
    setupRTCandSD();
    setupAccel();
    
    turnOn3v3SensorPower();
    turnOn5vSensorPower();
    
    setupTempADC();
    setupPARADC();
  unsigned int pressureVal10 = readPressure(true);
  unsigned int pressureVal3 = readPressure(false);
  int tempRefVal = readTemp(true);
  int tempVal = readTemp(false);
  int parVal = readPAR();
  float tiltInvCos = readAccel();
  SPI.setDataMode(SPI_MODE0);                         //Set SPI Data Mode
  File dataFile = SD.open("pro.txt", FILE_WRITE);      //open the file with write permissions
    if (dataFile) {                                     //if the file is available, write to it:
      dataFile.println();                               //Blank line between data sets
      
//      dataFile.print(F("Current RTC Time: "));
//      dataFile.print(now.toString(buf,len));        
//      dataFile.print(F(", Release Wake Up Time: "));
//      dataFile.print(releaseTime.toString(buf,len));        
//      dataFile.print(F(", Send Data Date:  "));
//      dataFile.println(sendDataDate.toString(buf,len));        
//      dataFile.print(F(", Ice Free Date:  "));
//      dataFile.println(iceFreeDate.toString(buf,len));        

      dataFile.print(wakeupTime.unixtime(),HEX);                    // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(pressureVal10);                    // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(pressureVal3);                     // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(tempRefVal);                       // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(tempVal);                          // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(parVal);                           // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(tiltInvCos,4);                       // ,
      dataFile.close();                                 //close the file (also makes sure all data in buffer is written)
    }
  else{
    Serial.println(F("Error opening file"));
  }  
    
//    dataFile = SD.open("testdata.txt");                    //re-open the file for reading:
//    if (dataFile) {                                    //if the file opened okay, write to it:
//      Serial.println(F("testdata.txt:"));                   //write to serial window
//      while (dataFile.available()) {                   //read from the file until there's nothing else in it:
//      	Serial.write(dataFile.read());                 //write output from file to serial window
//      }
//      dataFile.close();                                //close the file
//    } 
  SPI.setDataMode(SPI_MODE2);                         //Set SPI Data Mode
  
 }
 
void loop(){
  turnOff3v3SensorPower();
  turnOff5vSensorPower();
  turnOnGPSPower();
  foundGPS=lookForGPS();
  turnOffGPSPower();
  if(foundGPS==true){
    recordGPSFix();
    turnOnIridiumPower();
    sendIridiumAttempt();
    turnOffIridiumPower();
    Serial.println(F("I would Shut down here"));
    delay(100000);

  }
}

void recordGPSFix(){
  SPI.setDataMode(SPI_MODE0);                          //Set SPI Data Mode
  File dataFile = SD.open("botdat.txt", FILE_WRITE);   //open the file with write permissions
  if (dataFile) {                                    //if the file is available, write to it:
//    dataFile.print(F(","));                          // ,
//    dataFile.print(maxtiltInvCos,4);                      // ,
//    dataFile.print(F(","));                          // ,
//    dataFile.print(gpsDateVal);                      // ,
//    dataFile.print(F(","));                          // ,
//    dataFile.print(gpsTimeVal);                      // ,
//    dataFile.print(F(","));                          // ,
//    dataFile.print(gpsLat,4);                        // ,
//    dataFile.print(gpsLon,4);                        // ,
    dataFile.close();
  }
  SPI.setDataMode(SPI_MODE2);                         //Set SPI Data Mode
}


void sendIridiumAttempt(){
  int signalQuality = -1;
  nss.begin(19200);  //start serial comms on nss pins defined above
  isbd.attachConsole(Serial); //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only) 
  isbd.setPowerProfile(0); //1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.adjustSendReceiveTimeout(150);
  isbd.begin();  //wake up the 9602 and prepare it to communicate
//  int err = isbd.getSignalQuality(signalQuality);
//  if (err != 0)
//  {
//    Serial.print("SignalQuality failed: error ");
//    Serial.println(err);
//    return;
//  }
//  Serial.print("Signal quality is ");
//  Serial.println(signalQuality);

  
//  String sendGPSFix = "";
//  sendGPSFix += String(gpsLat,4);
//  sendGPSFix += ",";
//  sendGPSFix += String(gpsLon,4);
//  sendGPSFix += ",";
//  sendGPSFix += gpsTimeVal;
//  sendGPSFix += ",";
//  sendGPSFix += gpsDateVal;  
  const int sendbuffer_size = 52;
  char iridiumSendBuffer[sendbuffer_size];
  iridiumSendBuffer[0]='h';
  iridiumSendBuffer[1]='i';
//  sendGPSFix.toCharArray(iridiumSendBuffer,sendbuffer_size);
//  sendGPSFix = "";
// 
  uint8_t iridiumReceiveBuffer[6]={0,0,0,0,0,0};
  size_t bufferSize = sizeof(iridiumReceiveBuffer);

  int err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, bufferSize);

  if (err != 0)
  {
    Serial.print(F("sendSBDText failed: error "));
    Serial.println(err);
    return;
  }  
  Serial.println(F("Iridium Message Sent"));
  Serial.print(F("Messages left: "));
  Serial.println(isbd.getWaitingMessageCount());
  
  Serial.print(F("Inbound buffer size is "));
  Serial.println(bufferSize);
  for (int i=0; i<bufferSize; ++i)
  {
    Serial.write(iridiumReceiveBuffer[i]);
    Serial.print("(");
    Serial.print(iridiumReceiveBuffer[i], HEX);
    Serial.print(") ");
  }
  if(bufferSize==1||bufferSize==3||bufferSize==5){
    SPI.setDataMode(SPI_MODE0);                      //Set SPI Data Mode
    //File myFile;
    //if(iridiumReceiveBuffer[0]==49){myFile = SD.open("botdat.txt");}
    //if(iridiumReceiveBuffer[0]==51){myFile = SD.open("icedat.txt");}
    //if(iridiumReceiveBuffer[0]==53){myFile = SD.open("prodat.txt");}
    File myFile = SD.open("icedat.txt");
    if (myFile) {                                    //if the file opened okay, write to it:
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
        for(int i=0;i<50;i++){
          iridiumSendBuffer[i]=myFile.read();
        }      
        iridiumSendBuffer[50]=0;      
        Serial.print(F("Sending:"));
        Serial.println(iridiumSendBuffer);
        err = isbd.sendSBDText(iridiumSendBuffer); 
        if (err != 0){
          return;
        } 
      }
      myFile.close(); 
    }  
    SPI.setDataMode(SPI_MODE2);                          //Set SPI Data Mode
  }
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
    
    pinMode(ledPin, OUTPUT);
 }
 
 
void setupRTCandSD(){
  RTC.begin();                                               //Start RTC (defined by RTC instance in global declarations)
  SPI.setDataMode(SPI_MODE0);              //Set SPI Data Mode
  SD.begin(SDCARD_CS);                                       //Start SD Card so it is available for other function calls 
  SPI.setDataMode(SPI_MODE2);              //Set SPI Data Mode
}


unsigned int readPressure(boolean deepsensor){
  byte p3status;
  unsigned int p3;
  unsigned int t3;
  byte eocDelay=8;
  char kellerADDR = keller3bar;
  if(deepsensor){
    kellerADDR = keller10bar;
  }
  Wire.beginTransmission(kellerADDR);  
  Wire.write(checkpressure);  
  Wire.endTransmission();
  delay(eocDelay);                       //give sensor time to take measurement (8ms specified in protocol)
  Wire.requestFrom(kellerADDR,3);        //Take sample from pressure sensor
  while(Wire.available()){               //Ensure all the data comes in
    p3status = Wire.read(); 
    p3 = Wire.read(); 
    p3 = ((unsigned int)p3 << 8);
    p3 += Wire.read(); // low byte
    //depth in m = [(reading-16384)*maxbar/32768]*10
  }
  Wire.endTransmission();
  Serial.print(F("Status: "));
  Serial.println(p3status,BIN);
  Serial.print(F("P     : "));
  Serial.println(p3,DEC);
  Serial.println();
  return p3;
}

void setupTempADC(){
  Wire.beginTransmission(ads1100A0);
  Wire.write(B10001101); //set ADC gain to 2, 15 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(50);
}

int readTemp(boolean ref){
  int delaytime=1000;
  if(ref){
    digitalWrite(TEMP_SWITCH,HIGH);
    delay(delaytime);
  }
  int measurement = readTempADC();
  Serial.print(F("Temp ADC = "));
  Serial.print(measurement);
  Serial.println();
  if(ref){
    digitalWrite(TEMP_SWITCH,LOW);
    delay(delaytime);
  }
  return measurement;
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
  Wire.write(B10001111); //set ADC gain to 8, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(50);
}

int readPAR(){
  int measurement = readPARADC();
  Serial.print(F("PAR ADC = "));
  Serial.println(measurement);
  return measurement;
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


float readAccel(){
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
  
  //Serial.print(F("G Force = "));      
  //Serial.print(gForce,4);
  //Serial.print(F(", Inverse Cos of Tilt Angle = "));      
  //Serial.println(tiltCos,4);
  
  return tiltCos;
}

boolean lookForGPS(){
  Serial.println(F("Looking for GPS"));
  float maxtiltInvCos = readAccel();
  float tiltInvCos=maxtiltInvCos;
  ss.begin(9600);
  long endSearchTime=millis()+60000;  //60,000 ms = 1 min
  while((millis()<endSearchTime)&&(foundGPS==false)){
    while ((ss.available() > 0)&&(foundGPS==false)){
      if (gps.encode(ss.read())){
        tiltInvCos = readAccel();
        if(tiltInvCos<maxtiltInvCos){
          maxtiltInvCos=tiltInvCos;
          Serial.print(F("Max tilt now"));
          Serial.println(maxtiltInvCos,4);
        }
        if (gps.location.isValid()&&gps.date.isValid()&&gps.time.isValid()){
          foundGPS=true;
          double gpsLat = gps.location.lat();
          double gpsLon = gps.location.lng();
          unsigned long gpsTimeVal=gps.time.value();
          unsigned long gpsDateVal=gps.date.value();
          Serial.print(F("Found GPS,"));
          Serial.print(gpsLat,4);
          Serial.print(F(","));
          Serial.print(gpsLon,4);
          Serial.print(F(","));
          Serial.print(gpsTimeVal);
          Serial.print(F(","));
          Serial.println(gpsDateVal);
        }
      }
    }
  }  
  ss.end();
  return foundGPS;
}

void displayInfo()
{
  if (gps.location.isValid()&&gps.date.isValid()&&gps.time.isValid()){
    Serial.print(F("Location: ")); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F("  Date/Time: "));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    Serial.print(F(" "));
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
  else {
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


void turnOn3v3SensorPower(){
  digitalWrite(SENSOR_POWER_3V3,LOW);  
}
void turnOff3v3SensorPower(){
  digitalWrite(SENSOR_POWER_3V3,HIGH);    
}
void turnOn5vSensorPower(){
  digitalWrite(SENSOR_POWER_5V,LOW);  
}
void turnOff5vSensorPower(){
  digitalWrite(SENSOR_POWER_3V3,HIGH);  
}
void turnOnGPSPower(){
  digitalWrite(GPS_POWER,LOW);  
}
void turnOffGPSPower(){
  digitalWrite(GPS_POWER,HIGH);  
}
void turnOnIridiumPower(){
  digitalWrite(IRIDIUM_POWER,LOW);  
  digitalWrite(IRIDIUM_ON_OFF,HIGH);  
  Serial.println("Iridium power on");
  delay(1000);
  
}
void turnOffIridiumPower(){
  digitalWrite(IRIDIUM_POWER,HIGH);  
  digitalWrite(IRIDIUM_ON_OFF,LOW);  
}

void shutdownUnit(){
  digitalWrite(SHUTDOWN,LOW);
  delay(10000);
}



