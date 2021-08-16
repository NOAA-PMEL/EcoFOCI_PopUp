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
#define IRIDIUM_RXD A14
#define IRIDIUM_TXD A15

#define keller3bar 0x40       //Define pressure sensor control register
#define keller10bar 0x41      //Define pressure sensor control register
#define checkpressure 0xAC    //Define pressure sensor control register
#define ads1100A0 0x48        //Define tempADC control register
#define ads1100A1 0x49        //Define parADC control register

SoftwareSerial ss(GPS_TX,GPS_RX);
SoftwareSerial nss(IRIDIUM_TXD,IRIDIUM_RXD);
IridiumSBD isbd(nss,IRIDIUM_ON_OFF);
TinyGPSPlus gps;
RTC_DS3234 RTC(RTC_CS);
DateTime wakeupTime;
DateTime alarmtime;
DateTime unitStart =  DateTime(16,9,19,10,10,0);                    //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime releaseTime =  DateTime(16,9,19,11,0,0);                  //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime sendDataDate = DateTime(16,9,19,11,30,0);                   //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime iceFreeDate = DateTime(16,9,19,11,30,0);                   //DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
boolean foundGPS=false; 

void setup(){
  initializePins();
  Wire.begin();
  SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
  SPI.setDataMode(SPI_MODE2);              //Set SPI Data Mode
  SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
  SPI.setClockDivider(SPI_CLOCK_DIV16);    //Set SPI Clock Speed
//  Serial.begin(115200);                      //Turn the Serial Protocol ON
//  Serial.print(F("Serial Begin:"));
//  Serial.println(freeMemory());

  setupRTCandSD();
  setupAccel();
  
 
  turnOn3v3SensorPower();
  turnOn5vSensorPower();
  delay(100);
  setupTempADC();
  setupPARADC();
  
  char mode = selectMode();
  setAlarm(mode);
  sampleData(mode);    

  if(mode==1||mode==99){
    shutdownUnit();
  }
  else if(mode==2){
    waitForProfile();  
    collectProfile();
    shutdownUnit();
  }
  else if(mode==3){
    underIceEarlyMode();  
    shutdownUnit();
  }
  else if(mode==4){
    underIceLateMode(); 
    setAlarm(6); 
    delay(50);
    shutdownUnit();
  }    
}

void initializePins(){
  pinMode(SHUTDOWN,OUTPUT);
  digitalWrite(SHUTDOWN,HIGH);
  pinMode(TEMP_SWITCH,OUTPUT);
  digitalWrite(TEMP_SWITCH,HIGH);
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
  pinMode(RTC_CS,OUTPUT);
  digitalWrite(RTC_CS,HIGH);
}

void setupRTCandSD(){
  RTC.begin();                                               //Start RTC (defined by RTC instance in global declarations)
  wakeupTime = RTC.now();                                                        //Get time from RTC
  SPI.setDataMode(SPI_MODE0);              //Set SPI Data Mode
  SD.begin(SDCARD_CS);                                       //Start SD Card so it is available for other function calls 
  SPI.setDataMode(SPI_MODE2);              //Set SPI Data Mode
}

char selectMode(){
  char mode=0;
  alarmtime = wakeupTime.unixtime() + 86400;  //failsafe if program freezes (t+1day)
  setAlarmTime();                                                       //failsafe if program freezes (t+1day)
  if(wakeupTime.unixtime()<unitStart.unixtime()){                                                                   //if more than 1 minute,10s from release time, just take bottom sample
    mode=99;
  }
  //if(wakeupTime.unixtime()<(releaseTime.unixtime()-21660)){                                                              //if more than 6 hours, 1min from release time, just take bottom sample
  if(wakeupTime.unixtime()<(releaseTime.unixtime()-330)){                                                                //if more than 5 minutes,30s from release time, just take bottom sample
  //else if(wakeupTime.unixtime()<(releaseTime.unixtime()-70)){                                                                   //if more than 1 minute,10s from release time, just take bottom sample
    mode=1;
  }
  //else if((wakeupTime.unixtime()>=(releaseTime.unixtime()-21600))&&(wakeupTime.unixtime()<=(releaseTime.unixtime()))){   //if 6 hours or less from release time,go to WaitForProfile mode
  else if((wakeupTime.unixtime()>=(releaseTime.unixtime()-330))&&(wakeupTime.unixtime()<=(releaseTime.unixtime()))){     //if 5 minutes, 30s or less from release time,go to WaitForProfile mode
  //else if((wakeupTime.unixtime()>=(releaseTime.unixtime()-70))&&(wakeupTime.unixtime()<=(releaseTime.unixtime()))){        //if 1 minutes, 10s or less from release time,go to WaitForProfile mode
    mode=2;
  }  
  //else if((wakeupTime.unixtime()>releaseTime.unixtime())&&(wakeupTime.unixtime()<sendDataDate.unixtime())){              //if after Release Date and Before Send Data data, go to under ice mode
  else if((wakeupTime.unixtime()>releaseTime.unixtime())&&(wakeupTime.unixtime()<sendDataDate.unixtime())){              
    mode=3;
  }  
  //else if(wakeupTime.unixtime()>=sendDataDate.unixtime()){                                                               //attempt to transmit mode
  else if(wakeupTime.unixtime()>=sendDataDate.unixtime()){                                                               //attempt to transmit mode
    mode=4;
  }  
  return mode; 
}

void sampleData(char mode){
  unsigned int pressureVal10 = readPressure(true);
  unsigned int pressureVal3 = readPressure(false);
  int parVal = readPAR();
  float tiltInvCos = readAccel();
  int tempRefVal = 0;
  if(mode!=5){
    tempRefVal = readTemp(true);
  }
  int tempVal = readTemp(false);
  
  SPI.setDataMode(SPI_MODE0);                              //Set SPI Data Mode
  File dataFile;
  if(mode==1||mode==2){                                    //if at least 6 hours 1 minute from releaseWakeDate and unit is on bottom, set alarm for 6 hours from now
    dataFile = SD.open("botlake2.txt", FILE_WRITE);          //open the file with write permissions
  }
  else if(mode==3||mode==4){
    dataFile = SD.open("icelake2.txt", FILE_WRITE);          //open the file with write permissions
  }
  else if(mode==5){
    dataFile = SD.open("prolake2.txt", FILE_WRITE);          //open the file with write permissions      
  }
  if (dataFile) {                                          //if the file is available, write to it:
    dataFile.print(F(";"));                                // ,
    if(mode==5){
      SPI.setDataMode(SPI_MODE2);                          //Set SPI Data Mode
      DateTime now = RTC.now();
      SPI.setDataMode(SPI_MODE0);                          //Set SPI Data Mode
      dataFile.print(now.unixtime(),HEX);                        
    }
    else{
      dataFile.print(wakeupTime.unixtime(),HEX);           
    }
    dataFile.print(F(","));                                
    dataFile.print(pressureVal10);                         
    dataFile.print(F(","));                                
    dataFile.print(pressureVal3);                          
    dataFile.print(F(","));                                
    dataFile.print(tempRefVal);                      
    dataFile.print(F(","));                        
    dataFile.print(tempVal);                   
    dataFile.print(F(","));                      
    dataFile.print(parVal);                     
    dataFile.print(F(","));                      
    dataFile.print(tiltInvCos,4);                 
    dataFile.close();
  }
  SPI.setDataMode(SPI_MODE2);                         //Set SPI Data Mode
}

void setAlarm(char mode){
  if(mode==1){                                                                      
    //if(wakeupTime.unixtime()<(releaseTime.unixtime()-43260)){             //if more than 12 hours 1 minute from releaseTime set alarm for 6 hours from now
    if(wakeupTime.unixtime()<(releaseTime.unixtime()-610)){               //if more than 10 minutes,10s from releaseTime set alarm for 5 min from now
    //if(wakeupTime.unixtime()<(releaseTime.unixtime()-130)){                 //if more than 2 minutes, 10s from releaseTime set alarm for 1 min from now
      //alarmtime = wakeupTime.unixtime()+21600;                            //set alarm for 6 hours from wakeupTime (minus any spare seconds)
      alarmtime = wakeupTime.unixtime()+300;                              //set alarm for 5 minutes from wakeupTime 
      //alarmtime = wakeupTime.unixtime()+60;                                 //set alarm for 1 minute from wakeupTime 
    }      
    else{                                                                  //if less than 12 hours and 1 minute from releaseTime, set for 6 hours before release
      //alarmtime = releaseTime.unixtime()-21600;                          //set alarm for 6 hours before releaseTime
      alarmtime = releaseTime.unixtime()-300;                            //set alarm for 5 minutes before releaseTime
      //alarmtime = releaseTime.unixtime()-60;                               //set alarm for 1 minute before releaseTime
    } 
  }
  else if(mode==2){
    //alarmtime = releaseTime.unixtime()+21600;                           //set alarm for 6 hours after releaseTime 
    //alarmtime = releaseTime.unixtime()+1800;                            //set alarm for 30 min after releaseTime 
    alarmtime = releaseTime.unixtime()+900;                               //set alarm for 15 min after releaseTime 
  }
  else if(mode==3||mode==4){
    //alarmtime = wakeupTime.unixtime()+3600;  //set alarm for 1 hour from wakeupTime (minus any spare seconds)
    alarmtime = wakeupTime.unixtime()+300;     //set alarm for 5 minutes from wakeupTime (minus any spare seconds)
    //alarmtime = wakeupTime.unixtime()+60;    //set alarm for 1 min from wakeupTime (minus any spare seconds)
  }
  else if(mode==6){
    DateTime now = RTC.now();
    //alarmtime = now.unixtime()+3600;          //set alarm for 1 hour from wakeupTime (minus any spare seconds)
    alarmtime = now.unixtime()+300;          //set alarm for 1 hour from wakeupTime (minus any spare seconds)
  }
  else if(mode==99)  {
    alarmtime = unitStart.unixtime();  //failsafe if other modes don't work
  }
  else{
    alarmtime = wakeupTime.unixtime()+86400;  //failsafe if other modes don't work
  }
  setAlarmTime();
}

void waitForProfile(){
  unsigned int pressureAverageLong = averagePressureLong();
  unsigned int pressureAverageShort = averagePressureShort();
  DateTime now = RTC.now();
  //while(pressureAverageShort>(pressureAverageLong-1500)){        //while average of last 3 depth readings is at least .95 of original established depth
  while(pressureAverageShort>(pressureAverageLong-325)){          //while average of last 3 depth readings is at least .95 of original established depth
    now = RTC.now();                                     
    //if(now.unixtime()>(releaseTime.unixtime()+21540)){             //if 5hr59m hours past expected release time, shut the unit down (alarm already set for releaseTime+7 hours)
    if(now.unixtime()>(releaseTime.unixtime()+900)){             //if 5hr59m hours past expected release time, shut the unit down (alarm already set for releaseTime+7 hours)
      shutdownUnit();
    }
    delay(1000);
    pressureAverageShort=averagePressureShort();                  //update the short running depth average every 2ish seconds
  }
}

unsigned int averagePressureLong(){
  long depthAverage=0;
  for(char sample=0;sample<20;sample++){
    depthAverage=depthAverage+readPressure(true); 
    delay(1000);
  }
  depthAverage=depthAverage/20;
  return depthAverage;
}

unsigned int averagePressureShort(){
  long depthAverage=0;
  for(char sample=0;sample<3;sample++){
    depthAverage=depthAverage+readPressure(true); 
    delay(100);
  }
  depthAverage=depthAverage/3;
  return depthAverage;
}

void collectProfile(){
  //long endProfileTime=millis()+60000;  //60,000 ms = 1 min
  long endProfileTime=millis()+20000;  //20,000 ms = 30 sec
  long quarterSecond=millis()+220;  
    while(millis()<endProfileTime){
      while(millis()<quarterSecond){}
        sampleData(5);
        quarterSecond=millis()+220;
    }
}

void underIceEarlyMode(){
  turnOff3v3SensorPower();
  turnOff5vSensorPower();
  turnOnGPSPower();
  if(wakeupTime.hour()==15){
    foundGPS = lookForGPS();
  }
}

void underIceLateMode(){
  turnOff3v3SensorPower();
  turnOff5vSensorPower();
  turnOnGPSPower();
  if(wakeupTime.hour()==7||wakeupTime.hour()==15||wakeupTime.unixtime()>iceFreeDate.unixtime()){
    foundGPS = lookForGPS();
  }
  if(foundGPS==true){
    SPI.setDataMode(SPI_MODE0);                        //Set SPI Data Mode
    File dataFile = SD.open("gpstest4.txt", FILE_WRITE); //open the file with write permissions
    if (dataFile) {                                    //if the file is available, write to it:
      dataFile.print(F(",<--SendingIridiumMessage"));                          // ,
      dataFile.close();
    }
    SPI.setDataMode(SPI_MODE2);                        //Set SPI Data Mode
    sendIridiumAttempt();
  }
}

boolean lookForGPS(){
  double gpsLat;
  double gpsLon;
  unsigned long gpsTimeVal;
  unsigned long gpsDateVal;
  float maxtiltInvCos = readAccel();
  float tiltInvCos=maxtiltInvCos;
  ss.begin(9600);
  long endSearchTime=millis()+120000;  //60,000 ms = 1 min
  while((millis()<endSearchTime)&&(foundGPS==false)){
    while ((ss.available() > 0)&&(foundGPS==false)){
      if (gps.encode(ss.read())){
        tiltInvCos = readAccel();
        if(tiltInvCos<maxtiltInvCos){
          maxtiltInvCos=tiltInvCos;
        }
        if (gps.location.isValid()&&gps.date.isValid()&&gps.time.isValid()){
          foundGPS=true;
          gpsLat = gps.location.lat();
          gpsLon = gps.location.lng();
          gpsTimeVal=gps.time.value();
          gpsDateVal=gps.date.value();
        }
      }
    }
  }  
  ss.end();
  turnOffGPSPower();
  if(foundGPS=true){
    SPI.setDataMode(SPI_MODE0);                        //Set SPI Data Mode
    File dataFile = SD.open("gpstest2.txt", FILE_WRITE); //open the file with write permissions
    if (dataFile) {                                    //if the file is available, write to it:
      dataFile.print(F(","));                          // ,
      dataFile.print(maxtiltInvCos,4);                 // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(gpsDateVal);                      // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(gpsTimeVal);                      // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(gpsLat,4);                        // ,
      dataFile.print(F(","));                          // ,
      dataFile.print(gpsLon,4);                        // ,
        dataFile.print(F(","));                          // ,
        dataFile.print(foundGPS);                        // ,
      dataFile.close();
    }
    SPI.setDataMode(SPI_MODE2);                         //Set SPI Data Mode
  }
  return foundGPS;
}

void sendIridiumAttempt(){
  turnOnIridiumPower();
  delay(50);
  int signalQuality = -1;
  Serial.begin(115200);
  nss.begin(19200);  //start serial comms on nss pins defined above
  isbd.setPowerProfile(0); //1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.attachConsole(Serial); //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only) 
  isbd.adjustSendReceiveTimeout(180);
  isbd.setMinimumSignalQuality(1);
  isbd.begin();  //wake up the 9602 and prepare it to communicate
  
  const int sendbuffer_size = 51;
  char iridiumSendBuffer[sendbuffer_size];
  iridiumSendBuffer[0]='h';
  iridiumSendBuffer[1]='i';
  iridiumSendBuffer[2]='\0';
  uint8_t iridiumReceiveBuffer[50];
  size_t bufferSize = sizeof(iridiumReceiveBuffer);
  
  int err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, bufferSize);
  if (err != 0)
  {
    return;
  }
    SPI.setDataMode(SPI_MODE0);                      //Set SPI Data Mode
    File myFile=SD.open("botlake2.txt", FILE_READ);
    if (myFile) {                                    //if the file opened okay, write to it:
      while(myFile.available()){
        Serial.print("Parsing:");
        for(int i=0;i<50;i++){
          iridiumSendBuffer[i]=myFile.read();
          Serial.print(iridiumSendBuffer[i]);
          if(iridiumSendBuffer[i]==';'){
            iridiumSendBuffer[i]=myFile.read();
          }
        }      
        Serial.println();
        iridiumSendBuffer[50]='\0';    
        Serial.print("Sending:");
        Serial.println(iridiumSendBuffer);
        err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, bufferSize);
      }
      myFile.close();
    }
    SPI.setDataMode(SPI_MODE2);                      //Set SPI Data Mode
//        for(int i=0;i<50;i++){
//          iridiumSendBuffer[i]=';';
//        }      
//        iridiumSendBuffer[50]='\0';    
//        
        SPI.setDataMode(SPI_MODE0);                      //Set SPI Data Mode
        File dataFile = SD.open("iridbuf5.txt", FILE_WRITE); //open the file with write permissions
            if (dataFile) {                                    //if the file is available, write to it:
              dataFile.print(iridiumSendBuffer);                          // ,
              dataFile.print(F(",memory="));                          // ,
              dataFile.print(freeMemory());                          // ,
              if (err != 0){
                dataFile.print(F(",err="));                          // ,
                dataFile.print(err);                          // ,
                dataFile.print(F(",len="));                          // ,
                dataFile.print(sizeof(iridiumSendBuffer));                          // ,
                dataFile.print(F(",memory="));                          // ,
                dataFile.print(freeMemory());                          // ,
                dataFile.close();  
                return;
              }
              dataFile.close();  
            }
        SPI.setDataMode(SPI_MODE2);                      //Set SPI Data Mode
//    }
//      myFile.close(); 
    
//    myFile = SD.open("icelake2.txt");
//     if (myFile) {                                    //if the file opened okay, write to it:
//      while (myFile.available()) {                   //read from the file until there's nothing else in it:
//        for(int i=0;i<50;i++){
//          iridiumSendBuffer[i]=myFile.read();
//        }      
//        iridiumSendBuffer[50]=0;      
//        err = isbd.sendSBDText(iridiumSendBuffer); 
//        if (err != 0){
//          return;
//        } 
//      }
//      myFile.close(); 
//    }
//    
//    myFile = SD.open("prolake2.txt");    
//    if (myFile) {                                    //if the file opened okay, write to it:
//      while (myFile.available()) {                   //read from the file until there's nothing else in it:
//        for(int i=0;i<50;i++){
//          iridiumSendBuffer[i]=myFile.read();
//        }      
//        iridiumSendBuffer[50]=0;      
//        err = isbd.sendSBDText(iridiumSendBuffer); 
//        if (err != 0){
//          return;
//        } 
//      }
//      myFile.close(); 
//    }
//  SPI.setDataMode(SPI_MODE2);                          //Set SPI Data Mode
    
//  if(bufferSize==1||bufferSize==3||bufferSize==5){
//    SPI.setDataMode(SPI_MODE0);                      //Set SPI Data Mode
//    File myFile;
//      if(bufferSize==1){myFile = SD.open("botdat.txt");}
//      if(bufferSize==3){myFile = SD.open("icedat.txt");}
//      if(bufferSize==5){myFile = SD.open("prodat.txt");}    
//    if (myFile) {                                    //if the file opened okay, write to it:
//      while (myFile.available()) {                   //read from the file until there's nothing else in it:
//        for(int i=0;i<50;i++){
//          iridiumSendBuffer[i]=myFile.read();
//        }      
//        iridiumSendBuffer[50]=0;      
//        err = isbd.sendSBDText(iridiumSendBuffer); 
//        if (err != 0){
//          return;
//        } 
//      }
//      myFile.close(); 
//    }
//    SPI.setDataMode(SPI_MODE2);                          //Set SPI Data Mode
//  }
  turnOffIridiumPower();
}

void loop(){  
  shutdownUnit();
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
  return p3;
}

void setupTempADC(){
  Wire.beginTransmission(ads1100A0);
  Wire.write(B10001101); //set ADC gain to 2, 15 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(50);
}

int readTemp(boolean ref){
  int delaytime=500;
  if(ref){
    digitalWrite(TEMP_SWITCH,LOW);
    delay(delaytime);
  }
  int measurement = readTempADC();
  if(ref){
    digitalWrite(TEMP_SWITCH,HIGH);
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
  Wire.write(B10001100); //set ADC gain to 1, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();
  delay(50);
}

int readPAR(){
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
  return tiltCos;
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}

void setAlarmTime(){
  SPI.setDataMode(SPI_MODE1);                                //Set SPI Data Mode  
  RTCWrite(0x87,alarmtime.second() & 0x7F);  //set alarm time: seconds     //87=write to location for alarm seconds    //binary & second with 0x7F required to turn alarm second "on"
  RTCWrite(0x88,alarmtime.minute() & 0x7F);  //set alarm time: minutes     //88=write to location for alarm minutes    //binary & minute with 0x7F required to turn alarm minute "on"
  RTCWrite(0x89,alarmtime.hour() & 0x7F);    //set alarm time: hour        //89=write to location for alarm hour       //binary & hour with 0x7F required to turn alarm hour "on"
  RTCWrite(0x8A,alarmtime.day() & 0x3F);     //set alarm time: day         //8A=write to location for alarm day        //binary & day with 0x3F required to turn alarm day "on" (not dayofWeek) 
  RTCWrite(0x8B,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8C,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8D,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8F,B00000000);                  //reset flags                //8F=write to location for control/status flags    //B00000000=Ocillator Stop Flag 0, No Batt Backed 32 KHz Output, Keep Temp CONV Rate at 64 sec (may change later), disable 32 KHz output, temp Not Busy, alarm 2 not tripped, alarm 1 not tripped
  RTCWrite(0x8E,B00000101);                  //set control register       //8E=write to location for control register        //B01100101=Oscillator always on, SQW on, Convert Temp off, SQW freq@ 1Hz, Interrupt enabled, Alarm 2 off, Alarm 1 on
  SPI.setDataMode(SPI_MODE2);                                //Set SPI Data Mode
}

void RTCWrite(char reg, char val){
  digitalWrite(RTC_CS, LOW);                 //enable SPI read/write for chip
  SPI.transfer(reg);                         //define memory register location
  SPI.transfer(bin2bcd(val));                //write value
  delay(10);                                  //delay 5 ms to make sure chip is off
  digitalWrite(RTC_CS, HIGH);                //disable SPI read/write for chip
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
}
void turnOffIridiumPower(){
  digitalWrite(IRIDIUM_POWER,HIGH);  
  digitalWrite(IRIDIUM_ON_OFF,LOW);  
}

void shutdownUnit(){
  digitalWrite(SHUTDOWN,LOW);
  delay(100000);
}

bool ISBDCallback()
{
   return true;
}


