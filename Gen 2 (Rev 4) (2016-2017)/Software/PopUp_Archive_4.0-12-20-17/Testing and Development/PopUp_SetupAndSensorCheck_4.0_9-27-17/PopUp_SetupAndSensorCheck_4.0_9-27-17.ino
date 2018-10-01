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
boolean beginSDsuccess;
//********************************************************************************************************************************************************//
//***SET DATES AND PROGRAM INTERVALS HERE*****************************************************************************************************************//
//********************************************************************************************************************************************************//

DateTime unitStart =  DateTime(17,3,1,11,0,0);             //Date and Time for first sample: 				DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime releaseTime =  DateTime(17,4,1,15,0,0);            //Date and Time for actual release of burn wire: 		DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime sendDataDate = DateTime(17,4,1,18,0,0);            //Date and Time to attempt data transmission twice a day: 	DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime iceFreeDate = DateTime(17,4,1,20,0,0);             //Date and Time to attempt data transmission every hour: 	DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
#define FAILSAFE_TIMER 86400				    //Alarm for Failsafe if program freezes somehow 		(nominal FAILSAFE_TIMER 86400 = 1 day)
#define BOTTOM_SAMPLE_INTERVAL 300			    //Interval between bottom samples 				(nominal BOTTOM_SAMPLE_INTERVAL 21600 = 6 hours)
#define PRE_RELEASE_WAKEUP_TIME 3600			    //How long before planned release to wait for profile 	(nominal PRE_RELEASE_WAKEUP_TIME 21600 = 6 hours)
#define UNDER_ICE_SAMPLE_INTERVAL 7200			    //Interval between samples at the surface (or uncer ice)	(nominal UNDER_ICE_SAMPLE_INTERVAL 3600	= 1 hour)
#define PROFILE_LENGTH_SECONDS 60			    //Length of profile in seconds				(nominal PROFILE_LENGTH_SECONDS 60 = 60 seconds)
#define DEPTH_CHANGE_TO_TRIP_PROFILE 2			    //Depth in meters needed to begin profile mode		(nominal DEPTH_CHANGE_TO_TRIP_PROFILE 3	= 3 meters)
#define GPS_SEARCH_SECONDS 90				    //Number of seconds to search for GPS signal		(nominal GPS_SEARCH_SECONDS 120	= 2 minutes)
#define GPS_SECONDS_BETWEEN_SEARCHES 20		            //Number of seconds between GPS Searches to send data	(nominal GPS_SECONDS_BETWEEN_SEARCHES 180 = 3 minutes)
#define GPS_SUCCESSES_TO_SEND 1				    //Number of successful gps hits before sending data		(nominal GPS_SUCCESSES_TO_SEND 3 = 2 attempts)
#define CHARS_PER_SBDMESSAGE 100		            //Number of bytes to pull from data file and send at once	(nominal CHARS_PER_SBDMESSAGE 50 = 50 bytes)
#define RECEIVE_BUFFER_SIZE 15		                    //Number of bytes to pull from data file and send at once	(nominal CHARS_PER_SBDMESSAGE 50 = 50 bytes)
#define SEND_DATA_INTERVAL 300		                    //Default interval to send data once GPS has been found	(nominal SEND_DATA_INTERVAL 1800 = 30 minutes)

//********************************************************************************************************************************************************//
//***END PROGRAM DATES AND SAMPLE INTERVAL SECTION********************************************************************************************************//
//********************************************************************************************************************************************************//

void setup(){
    initializePins();		                //Initialize Arduino Pins for Output and Set Voltage Levels
    Wire.begin();				//Start I2C Communication
    SPI.begin();                                //Start up SPI Communication (needed for various ICs) 
    SPI.setDataMode(SPI_MODE2);                 //Set SPI Data Mode
    SPI.setBitOrder(MSBFIRST);                  //Configure SPI Communication (needed for various ICs)
    SPI.setClockDivider(SPI_CLOCK_DIV16);       //Set SPI Clock Speed

    setupRTCandSD();			        //Start RTC and SD Card 
    setupAccel();		                //Start Accelerometer

    turnOn3v3SensorPower();		        //turn on 3.3V Power to sensors (PAR, TEMP, PRESSURE)
    turnOn5vSensorPower();		        //turn on 5V Power to sensors (Needed for 5V I2C communication)
    delay(50);				        //Short delay here to start ADCs
    setupTempADC();			        //Start Up the ADC for Temperature Measurement
    setupPARADC();			        //Start Up the ADC for PAR Measurement
    delay(500);				        //Longer delay here to let voltages stabilize
  
    Serial.begin(115200);                       //Turn the Serial Protocol ON
    Serial.println(F("Set up Complete!!"));
    displayStatus();
    delay(2000); 
}

void loop(){
      //"OPTIONS" Menu
      Serial.println(F("-----------------------------------------------"));
      Serial.println(F("'1' - display Status"));
      Serial.println(F("'2' - set RTC time/date"));
      Serial.println(F("'3' - display SD Card Data"));
      Serial.println(F("'4' - check Temperature Sensor"));
      Serial.println(F("'5' - check Pressure Sensors"));
      Serial.println(F("'6' - check Accelerometer (Tilt Sensor)"));
      Serial.println(F("'7' - check PAR"));
      Serial.println(F("'8' - check GPS"));
      Serial.println(F("'9' - check Iridium"));
      Serial.println(F("'P' - collect PAR calibration data"));
      Serial.println(F("'X' - delete all SD Card Data"));
      Serial.println(F("-----------------------------------------------"));
      Serial.println();
      
      DateTime now = RTC.now();
      byte byteRead;   //variable to read Serial input from user
      boolean waiting=true;  //variable to wait for user input

      while(waiting==true){        //Run loop to set unit status as long as serial connection exists
        byteRead = Serial.read();   //read serial input (keystroke from user)
        switch (byteRead){   
          case  49 : //if user enters '1', display unit Status
            displayStatus();
            waiting=false;
            break;
          case  50 : //if user enters '2', display dialog to have user set RTC
            setRTC();
            waiting=false;
            break;
          case  51 : //if user enters '3', check SD Card Data
            sdDataCheck();
            waiting=false;
            break;
          case  52 : //if user enters '4', check Temperature
          case  53 : //if user enters '5', check Pressure
          case  54 : //if user enters '6', check Accelerometer
          case  55 : //if user enters '7', check PAR
            displaySensor(byteRead);
            waiting=false;
            break;
          case  56 : //if user enters '8', check GPS
            checkGPS();
            waiting=false;
            break;
          case  57 : //if user enters '9', check Iridium
            checkIridium();
            waiting=false;
            break;
          case  80 : //if user enters 'P', enter PAR calibration mode
            calibratePAR();
            waiting=false;
            break;
          case  88 : //if user enters 'X', delete files from SD Card
            deleteFiles();
            waiting=false;
            break;
        }
      }
      delay(2000);   //wait a few seconds before running the loop again
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

void setupRTCandSD(){
  SPI.setDataMode(SPI_MODE1);		//Set SPI Data Mode to 1 for RTC
  RTC.begin();                          //Start RTC (defined by RTC instance in global declarations)
  wakeupTime = RTC.now();               //Get time from RTC and store into global variable for when unit awoke
  SPI.setDataMode(SPI_MODE0);           //Set SPI Data Mode to 0 for SDCard
  beginSDsuccess = SD.begin(SDCARD_CS); //Start SD Card so it is available for other function calls 
  SPI.setDataMode(SPI_MODE2);           //Set SPI Data Mode to default
}

void displayStatus(){
  
  Serial.println(F("Unit Status:"));
  if(beginSDsuccess){
    Serial.println(F("  SD Card Initialized Successfully."));
  }
  else{
    Serial.println(F("  **SD Card error.  Check if SD Card is present and re-start program.**"));
  }
  displayRTCTime();
  displayTemperature();
  displayPAR();
  displayPressure();
  displayAccelerometer();
  Serial.println();
}

void displayRTCTime(){
  SPI.setDataMode(SPI_MODE1);                         	                                //Set SPI mode to 1 for RTC 
  const int len = 32;                                                                   //buffer length for RTC display on Serial comm
  static char buf[len];                                                                 //string for RTC display on Serial comm
  DateTime now = RTC.now();
  Serial.print(F("  Current RTC Time/Date: "));
  Serial.println(now.toString(buf,len));  
}

void displaySensor(char menuInput){
  Serial.println(F("Checking Sensor..."));
  Serial.println(F("Press 'Q' to stop\n"));
  boolean waiting2=true;                                //another variable to wait for user input
  byte byteRead=0;
  while(waiting2==true){
    if(Serial.available()){
        byteRead = Serial.read();
    }
    if(byteRead==81){
        waiting2=false;
    }
    if(menuInput==52){
      displayTemperature();
      Serial.println();
    }
    else if(menuInput==53){
      displayPressure();
      Serial.println();
    }
    else if(menuInput==54){
      displayAccelerometer();
    }
    else if(menuInput==55){
      displayPAR();
    }
    else{}
    delay(1000);
  }  
}

void displayAccelerometer(){
  float accelReading = readAccel();
  Serial.print(F("  Accelerometer Reading: "));
  Serial.print(accelReading,1);
  Serial.print(F(" (approx "));
  float tiltAngle = acos(accelReading/1000)/3.14159*180;			    //Get Tilt Angle from Accelerometer
  Serial.print(tiltAngle,1);
  Serial.println(F(" degrees from vertical)"));    
}

void displayPAR(){
  int parVal = readPAR();
  Serial.print(F("  PAR  Sensor ADC Val: ")); 
  Serial.print(parVal);
  float parCalc = (parVal-4842)*0.065753;
  Serial.print(F(" (approx "));
  Serial.print(parCalc);
  Serial.println(F(" umol/(m2s))"));
}

void calibratePAR(){
    Serial.println(F("Collecting PAR data and storing to SD Card..."));
    Serial.println(F("Data Collection will continue if Serial cable is disconnected"));
    Serial.println(F("Press 'Q' to stop\n"));
    boolean waiting2=true;                                //another variable to wait for user input
    byte byteRead=0;
    unsigned long nextSampleMillis=0;
    int PARmeasurement = 0;
    while(waiting2==true){
      nextSampleMillis=millis()+(unsigned long)1000;
      if(Serial.available()){
        byteRead = Serial.read();
      }
      if(byteRead==81){
        waiting2=false;
      }
      PARmeasurement = readPAR();
      storePAR(PARmeasurement);
      while(millis()<nextSampleMillis){}
    }
}


void displayTemperature(){
  int tempVal = readTemp(false);
  Serial.print(F("  Temp Sensor ADC Val: "));
  Serial.print(tempVal);
  Serial.print(F(" (approx "));
  float tempCalc = (1/(.0012768+.00053964*log10(tempVal)+.0000011763*pow(log10(tempVal),3)))-273.15;
  Serial.print(tempCalc,2);
  Serial.println(F(" C)"));
  int tempRefVal = readTemp(true);
  Serial.print(F("  Temp Ref    ADC Val: "));
  Serial.print(tempRefVal);
  float tempRefCalc = (1/(.0012768+.00053964*log10(tempRefVal)+.0000011763*pow(log10(tempRefVal),3)))-273.15;
  Serial.print(F(" (approx "));
  Serial.print(tempRefCalc,2);
  Serial.println(F(" C)")); 
}

void displayPressure(){
  int pressureVal100 = readPressure(true);
  int pressureVal30 = readPressure(false);
  Serial.print(F("  100m Pressure Sensor Val: "));
  Serial.print(pressureVal100);
  float pressureCalc100 = (pressureVal100-16384.0)/327.68;
  Serial.print(F(" (approx "));
  Serial.print(pressureCalc100);
  Serial.println(F(" m)"));
  Serial.print(F("  30m  Pressure Sensor Val: "));
  Serial.print(pressureVal30);
  float pressureCalc30 = (pressureVal30-16384.0)/1092.3;
  Serial.print(F(" (approx "));
  Serial.print(pressureCalc30);
  Serial.println(F(" m)"));
}

void checkGPS(){
}

void checkIridium(){
}

void storePAR(int PARmeasurement){
  SPI.setDataMode(SPI_MODE1);		                                  //Set SPI Data Mode to 1 for RTC
  DateTime timeStamp = RTC.now();
  SPI.setDataMode(SPI_MODE0);		                                  //Set SPI Data Mode to 1 for RTC
  File PARcalFile = SD.open("parcal.txt", FILE_WRITE);
  PARcalFile.print(timeStamp.unixtime());            
  PARcalFile.print(F(","));                      	                  //,
  PARcalFile.println(PARmeasurement);                     	                  //ADC Value from PAR Sensor
  PARcalFile.close();
  Serial.print(timeStamp.unixtime());
  Serial.print(F(","));                      	                  //,
  Serial.print(PARmeasurement);                     	                  //ADC Value from PAR Sensor
  float parCalc = (PARmeasurement-4842)*0.065753;
  Serial.print(F(", (approx "));
  Serial.print(parCalc);
  Serial.println(F(" umol/(m2s))"));


}

void setRTC(){
    Serial.println(F("Enter Current Date:"));
    Serial.println(F("MMDDYY"));
    char inputbuffer[6];
    boolean checkdigit = true;    //variable to check if user input is actually digits
    boolean waiting2=true;        //another variable to wait for user input
    byte readLen =0;
    byte MM;        //variables to store time and date values
    byte DD;
    byte YY;
    byte hh;
    byte mm;
    byte ss;
    while(waiting2==true){
      readLen = Serial.readBytes(inputbuffer, 6);   //read serial input (6 keystrokes from user)
      if (readLen==6){      //check if user has put in 6 digits correctly
        waiting2=false;
        for(int i = 0; i<6;i++){
          if(inputbuffer[i]<48||inputbuffer[i]>57){
            checkdigit=false;
          }
        }
      }
    }
    if(checkdigit==true){
      MM=10*(inputbuffer[0]-48)+(inputbuffer[1]-48);     //parse input data into vaiables
      DD=10*(inputbuffer[2]-48)+(inputbuffer[3]-48);
      YY=10*(inputbuffer[4]-48)+(inputbuffer[5]-48);
      Serial.print(MM);
      Serial.print(F("/")); 
      Serial.print(DD); 
      Serial.print(F("/")); 
      Serial.println(YY);
      Serial.println();      
    }
    else{
     Serial.println(F("**Input Error"));
    }
    waiting2=true;                                        //reset variable waiting for user input
    if(checkdigit==true){
      Serial.println(F("Enter Current Time:"));
      Serial.println(F("hhmmss"));
      while(waiting2==true){
        readLen = Serial.readBytes(inputbuffer, 6);       //read serial input (6 keystrokes from user)
        if (readLen==6){                                  //check if user has put in 6 digits correctly
          waiting2=false;
          for(int i = 0; i<6;i++){
            if(inputbuffer[i]<48||inputbuffer[i]>57){
              checkdigit=false;
            }
          }
        }
      }
      if(checkdigit==true){
        hh=10*(inputbuffer[0]-48)+(inputbuffer[1]-48);    //parse input data into vaiables
        mm=10*(inputbuffer[2]-48)+(inputbuffer[3]-48);
        ss=10*(inputbuffer[4]-48)+(inputbuffer[5]-48);
      Serial.print(hh); 
      Serial.print(F(":")); 
      Serial.print(mm); 
      Serial.print(F(":")); 
      Serial.println(ss); 
      Serial.println();
   }
      else{
       Serial.println(F("**Input Error"));
      }
    }
    if(checkdigit==true){                                    //if both inputs were six digits, then reset the alarm with the right values
      DateTime settime = DateTime(YY,MM,DD,hh,mm,ss);
      SPI.setDataMode(SPI_MODE1);                            //Set SPI Data Mode
      RTC.adjust(settime);
      SPI.setDataMode(SPI_MODE2);                            //Set SPI Data Mode
      Serial.println(F("**Time set successfully!**"));
      displayRTCTime();
    }
}

void sdDataCheck(){
    Serial.println(F("Checking SD Card Files...")); 
    SPI.setDataMode(SPI_MODE0);                        //Set SPI Data Mode
    File myFile = SD.open("botdat.txt", FILE_READ);    //open the file for reading:
    if (myFile) {                                      //if the file opened okay, write to it:
      Serial.println();                                //write to serial window
      Serial.println(F("botdat.txt:"));                //write to serial window
      while (myFile.available()) {                     //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                   //write output from file to serial window
      }
      Serial.println();                                //write to serial window
      myFile.close();                                  //close the file
    } 
    else {
      Serial.println(F("error opening botdat.txt"));   // if the file didn't open, print an error:
    }
    
    myFile = SD.open("icedat.txt", FILE_READ);         //open the file for reading:
    if (myFile) {                                      //if the file opened okay, write to it:
      Serial.println();                                //write to serial window
      Serial.println(F("icedat.txt:"));                //write to serial window
      while (myFile.available()) {                     //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                   //write output from file to serial window
      }
      Serial.println();                                //write to serial window
      myFile.close();                                  //close the file
    } 
    else {
      Serial.println(F("error opening icedat.txt"));   // if the file didn't open, print an error:
    }
    
    myFile = SD.open("prodat.txt", FILE_READ);         //open the file for reading:
    if (myFile) {                                      //if the file opened okay, write to it:
      Serial.println();                                //write to serial window
      Serial.println(F("prodat.txt:"));                //write to serial window
      while (myFile.available()) {                     //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                   //write output from file to serial window
      }
      Serial.println();                                //write to serial window
      myFile.close();                                  //close the file
    } 
    else {
      Serial.println("error opening prodat.txt");      
    }
  
  myFile = SD.open("summary.txt", FILE_READ);          //open the file for reading:
    if (myFile) {                                      //if the file opened okay, write to it:
      Serial.println();                                //write to serial window
      Serial.println(F("summary.txt:"));               //write to serial window
      while (myFile.available()) {                     //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                   //write output from file to serial window
      }
      Serial.println();                                //write to serial window
      myFile.close();                                  //close the file
    } 
    else {
      Serial.println(F("error opening summary.txt"));  // if the file didn't open, print an error:
    } 
    
  myFile = SD.open("filepos.txt", FILE_READ);          //open the file for reading:
    if (myFile) {                                      //if the file opened okay, write to it:
      Serial.println();                                //write to serial window
      Serial.println(F("filepos.txt:"));               //write to serial window
      while (myFile.available()) {                     //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                   //write output from file to serial window
      }
      Serial.println();                                //write to serial window
      myFile.close();                                  //close the file
    } 
    else {
      Serial.println(F("error opening filepos.txt"));  // if the file didn't open, print an error:
    }
    SPI.setDataMode(SPI_MODE2);                        //Set SPI Data Mode
    Serial.println();                                  //write to serial window
 }

void deleteFiles(){
  SPI.setDataMode(SPI_MODE0);                            //Set SPI Data Mode
  SD.remove("prodat.txt");
    Serial.println(F("Deleted prodat.txt"));      
  SD.remove("icedat.txt");
    Serial.println(F("Deleted icedat.txt"));      
  SD.remove("botdat.txt");
    Serial.println(F("Deleted botdat.txt"));      
  SD.remove("summary.txt");
    Serial.println(F("Deleted summary.txt"));      
  SD.remove("filepos.txt");   
    Serial.println(F("Deleted filepos.txt"));      
  SPI.setDataMode(SPI_MODE2);                            //Set SPI Data Mode
    Serial.println();
}

void setAlarmTime(DateTime alarmtime){
  SPI.setDataMode(SPI_MODE1);                //Set SPI Data Mode
  RTCWrite(0x87,alarmtime.second() & 0x7F);  //set alarm time: seconds     //87=write to location for alarm seconds    //binary & second with 0x7F required to turn alarm second "on"
  RTCWrite(0x88,alarmtime.minute() & 0x7F);  //set alarm time: minutes     //88=write to location for alarm minutes    //binary & minute with 0x7F required to turn alarm minute "on"
  RTCWrite(0x89,alarmtime.hour() & 0x7F);    //set alarm time: hour        //89=write to location for alarm hour       //binary & hour with 0x7F required to turn alarm hour "on"
  RTCWrite(0x8A,alarmtime.day() & 0x3F);     //set alarm time: day         //8A=write to location for alarm day        //binary & day with 0x3F required to turn alarm day "on" (not dayofWeek) 
  RTCWrite(0x8B,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8C,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8D,0);                          //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8F,B00000000);                  //reset flags                //8F=write to location for control/status flags    //B00000000=Ocillator Stop Flag 0, No Batt Backed 32 KHz Output, Keep Temp CONV Rate at 64 sec (may change later), disable 32 KHz output, temp Not Busy, alarm 2 not tripped, alarm 1 not tripped
  RTCWrite(0x8E,B00000101);                  //set control register       //8E=write to location for control register        //B01100101=Oscillator always on, SQW on, Convert Temp off, SQW freq@ 1Hz, Interrupt enabled, Alarm 2 off, Alarm 1 on
  SPI.setDataMode(SPI_MODE2);                //Set SPI Data Mode
}

void RTCWrite(char reg, char val){
  SPI.setClockDivider(SPI_CLOCK_DIV16);      //Set SPI Clock Speed
  digitalWrite(RTC_CS, LOW);                 //enable SPI read/write for chip
  SPI.transfer(reg);                         //define memory register location
  SPI.transfer(bin2bcd(val));                //write value
  digitalWrite(RTC_CS, HIGH);                //disable SPI read/write for chip
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
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
void turnOnGPSPower(){
  digitalWrite(GPS_POWER,LOW);  		//Pin LOW=Power ON (P-Channel switch)
}
void turnOffGPSPower(){
  digitalWrite(GPS_POWER,HIGH);  		//Pin HIGH=Power OFF (P-Channel switch)
}	
void turnOnIridiumPower(){
  digitalWrite(IRIDIUM_POWER,LOW);  		//Pin LOW=Power ON (P-Channel switch)
  digitalWrite(IRIDIUM_ON_OFF,HIGH);  		//Digital Pin for Iridium module sleep.  HIGH = wake from sleep
}
void turnOffIridiumPower(){
  digitalWrite(IRIDIUM_POWER,HIGH);  		//Pin HIGH=Power OFF (P-Channel switch)
  digitalWrite(IRIDIUM_ON_OFF,LOW);  		//Digital Pin for Iridium module sleep.  LOW = go to sleep
}



void setupPARADC(){
  Wire.beginTransmission(ads1100A1);		//Begin I2C comms with Temp ADC 
  Wire.write(B10001100); 			//set ADC gain to 1, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();			//End I2C Comms
  delay(50);					//Short delay to let ADC initialize
}

int readPAR(){
  byte controlRegister = 0;			//initialize control register variable 
  int adcVal = 0;				//initialize ADC Value variable
  Wire.requestFrom(ads1100A1, 3);  	        //request 3 bytes from appropriate ADC using I2C
  while(Wire.available()){ 			//ensure all the data comes in
    adcVal = Wire.read(); 			//first byte is MSB 
    adcVal = ((unsigned int)adcVal << 8);	//shift first byte 8 bits to the left to move it into MSB
    adcVal += Wire.read(); 			//second byte is LSB.  add this to the MSB
    controlRegister = Wire.read();	        //third byte is control register
  }
  return adcVal;				//return single value for ADC reading
}


void setupTempADC(){
  Wire.beginTransmission(ads1100A0);		//Begin I2C comms with Temp ADC 
  Wire.write(B10001101); 			//set ADC gain to 2, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();			//End I2C Comms
  delay(50);					//Short delay to let ADC initialize
}

int readTemp(boolean ref){			//function to read temperature ADC (for thermistor and reference resistor)
  int delaytime=500;				//milliseconds to delay between reference resistor and thermistor reading (needed for voltage to stabilize and geet a good reading)
  if(ref){					//if reference is true (i.e. we want to read the reference resistor)
    digitalWrite(TEMP_SWITCH,LOW);		//send signal to switch to reference reading
    delay(delaytime);				//delay to let voltage stabilize
  }
  int measurement = readTempADC();		//read the ADC (Same for thermistor or reference)
  if(ref){					//if reference is true (i.e. we have just read the reference resistor)
    digitalWrite(TEMP_SWITCH,HIGH);		//send signal to switch back to thermistor reading
  }
  return measurement;				//return the measured value from the ADC
}

int readTempADC(){							
  byte controlRegister = 0;			//initialize control registoer variable 
  int adcVal = 0;				//initialize ADC Value variable
  Wire.requestFrom(ads1100A0, 3);  		//request 3 bytes from appropriate ADC using I2C
  while(Wire.available()){ 			//ensure all the data comes in
    adcVal = Wire.read(); 		        //first byte is MSB 
    adcVal = ((unsigned int)adcVal << 8);	//shift first byte 8 bits to the left to move it into MSB
    adcVal += Wire.read(); 			//second byte is LSB.  add this to the MSB
    controlRegister = Wire.read();		//third byte is control register
  }
  return adcVal;				//return single value for ADC reading
}


unsigned int readPressure(boolean deepsensor){	//depth in m = [(reading-16384)*maxbar/32768]*10
  byte p3status;				//initialize variable for sensor status
  unsigned int p3;				//initialize variable for pressure reading 
  byte eocDelay=8;				//variable for conversion delay (8 ms defined in Keller communication protocol)
  char kellerADDR = keller3bar;			//initialize I2C address 
  if(deepsensor){				//if reading deep pressure sensor (10 bar),
    kellerADDR = keller10bar;			//then change I2C address
  }
  Wire.beginTransmission(kellerADDR);  		//Begin I2C comms with pressure sensor
  Wire.write(checkpressure);  			//Send write command to start pressure conversion 
  Wire.endTransmission();			//End command for pressure conversion 
  delay(eocDelay);                       	//Delay for conversion (8 ms defined in Keller communication protocol)
  Wire.requestFrom(kellerADDR,3);        	//Read data from pressure sensor
  while(Wire.available()){               	//Ensure all the data comes in
    p3status = Wire.read(); 			//First byte is Sensor Status  (Can possibly be used for error checking)
    p3 = Wire.read(); 				//Second byte is Pressure reading MSB
    p3 = ((unsigned int)p3 << 8);		//Shift second byte to MSB 
    p3 += Wire.read(); 				//Third byte is LSB, add to pressure reading
  }
  Wire.endTransmission();			//End I2C communication for reading pressure
  return p3;					//Return value for pressure only
}


void setupAccel(){				//Setup ADXL345 Chip 
  SPI.setDataMode(SPI_MODE3);                   //Set SPI Data Mode to 3 for Accelerometer
  sendSPIdata(ACCEL_CS,0x2C,B00001010);		//0x2C=data rate and power mode control, B00001010=normal power mode, SPI data rate =100HZ
  sendSPIdata(ACCEL_CS,0x31,B00001011);		//0x31=data format control, B00001011=no self test, SPI mode, Active LOW, full resolution mode, left justified (MSB), +/-16g range 
  sendSPIdata(ACCEL_CS,0x2D,B00001000);		//0x2D=power control, B00001000=measurement mode
  SPI.setDataMode(SPI_MODE2);                   //Set SPI Data Mode to default
}

float readAccel(){				//Read tilt angle from ADXL345 Chip
  SPI.setDataMode(SPI_MODE3);		        //Set SPI Data Mode to 3 for Accelerometer
  digitalWrite(ACCEL_CS, LOW);			//Select Accelerometer for SPI Communication	
  SPI.transfer(0x32 | 0xC0);			//Request x,y,and z Acceleration Values from Accelerometer 
  byte x0 = SPI.transfer(-1);			//x0=Accel LSB
  byte x1 = SPI.transfer(-1);			//x1=Accel MSB
  byte y0 = SPI.transfer(-1);			//y0=Accel LSB
  byte y1 = SPI.transfer(-1);			//y1=Accel MSB
  byte z0 = SPI.transfer(-1);			//z0=Accel LSB
  byte z1 = SPI.transfer(-1);			//z1=Accel MSB
  digitalWrite(ACCEL_CS, HIGH);			//De-Select Accelerometer for SPI Communication
  SPI.setDataMode(SPI_MODE2);           	//Set SPI Data Mode to default

  float x = x1<<8 | x0;				//Combine x1(MSB) and x0(LSB) into x
  float y = y1<<8 | y0;				//Combine y1(MSB) and y0(LSB) into y
  float z = z1<<8 | z0;				//Combine z1(MSB) and z0(LSB) into z
  float xg = x*3.9/1000;			//Convert x accel value to g force units
  float yg = y*3.9/1000;			//Convert x accel value to g force units
  float zg = z*3.9/1000;			//Convert x accel value to g force units
  float gForce=sqrt(sq(xg)+sq(yg)+sq(zg));	//Find Total G force (3 dim Pythagorean)
  float tiltCos=yg/gForce;			//Find y component of G force (equal to cosine of angle from vertical)
  tiltCos=tiltCos*1000;				//Multiply by 1000 to get desired resolution and save 2 characters (0 and decimal point) for data output
  return tiltCos;				//Return the cosine of the angle from vertical (Arduino doesn't know ArcCos)
}

