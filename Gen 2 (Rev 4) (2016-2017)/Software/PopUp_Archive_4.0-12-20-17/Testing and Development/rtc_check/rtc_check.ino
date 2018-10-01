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
//********************************************************************************************************************************************************//
//***SET DATES AND PROGRAM INTERVALS HERE*****************************************************************************************************************//
//********************************************************************************************************************************************************//

DateTime unitStart =  DateTime(16,12,5,8,9,0);            //Date and Time for first sample: 				DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime releaseTime =  DateTime(16,12,5,8,15,0);          //Date and Time for actual release of burn wire: 		DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime sendDataDate = DateTime(16,12,5,8,21,0);          //Date and Time to attempt data transmission twice a day: 	DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime iceFreeDate = DateTime(16,12,5,8,21,0);           //Date and Time to attempt data transmission every hour: 	DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
#define FAILSAFE_TIMER 86400				    //Alarm for Failsafe if program freezes somehow 		(nominal FAILSAFE_TIMER 86400 = 1 day)
#define BOTTOM_SAMPLE_INTERVAL 180			    //Interval between bottom samples 				(nominal BOTTOM_SAMPLE_INTERVAL 21600 = 6 hours)
#define PRE_RELEASE_WAKEUP_TIME 180			    //How long before planned release to wait for profile 	(nominal PRE_RELEASE_WAKEUP_TIME 21600 = 6 hours)
#define UNDER_ICE_SAMPLE_INTERVAL 180			    //Interval between samples at the surface (or uncer ice)	(nominal UNDER_ICE_SAMPLE_INTERVAL 3600	= 1 hour)
#define PROFILE_LENGTH_SECONDS 60			    //Length of profile in seconds				(nominal PROFILE_LENGTH_SECONDS 60 = 60 seconds)
#define DEPTH_CHANGE_TO_TRIP_PROFILE 1			    //Depth in meters needed to begin profile mode		(nominal DEPTH_CHANGE_TO_TRIP_PROFILE 3	= 3 meters)
#define GPS_SEARCH_SECONDS 180				    //Number of seconds to search for GPS signal		(nominal GPS_SEARCH_SECONDS 120	= 2 minutes)
#define GPS_SECONDS_BETWEEN_SEARCHES 20		            //Number of seconds between GPS Searches to send data	(nominal GPS_SECONDS_BETWEEN_SEARCHES 180 = 3 minutes)
#define GPS_SUCCESSES_TO_SEND 1				    //Number of successful gps hits before sending data		(nominal GPS_SUCCESSES_TO_SEND 3 = 2 attempts)
#define CHARS_PER_SBDMESSAGE 100		            //Number of bytes to pull from data file and send at once	(nominal CHARS_PER_SBDMESSAGE 50 = 50 bytes)


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
    Serial.begin(115200);                       //Turn the Serial Protocol ON
    setupRTCandSD();			        //Start RTC and SD Card 
    Serial.println(F("Set up Complete!!"));
    sdCardCheck();
    rtcCheck();
    delay(2000); 
}


void loop(){
      //"OPTIONS" Menu
      Serial.println(F("-----------------------------------------------"));
      Serial.println(F("'S' to display Status"));
      Serial.println(F("'C' to set time to program compile time"));
      Serial.println(F("'T' to set time to custom time/date"));
      Serial.println(F("'D' to display SD Card Data"));
      Serial.println(F("'X' to delete all SD Card Data"));
      Serial.println(F("-----------------------------------------------"));
      Serial.println();
      
      DateTime now = RTC.now();
      byte byteRead;   //variable to read Serial input from user
      boolean waiting=true;  //variable to wait for user input

      while(waiting==true){        //Run loop to set unit status as long as serial connection exists
        byteRead = Serial.read();   //read serial input (keystroke from user)
        switch (byteRead){   
          case  67 : //'C'
            rtcSystemSync();
            waiting=false;
            break;
          case  68 : //'D'
            sdDataCheck();
            waiting=false;
            break;
          case  83 : //if user enters 'S', display logger "Status"
            rtcCheck();
            waiting=false;
            break;
          case  84 : //if user enters 'T', display dialog to have user set RTC
            setRTC();
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
  SD.begin(SDCARD_CS);                  //Start SD Card so it is available for other function calls 
  SPI.setDataMode(SPI_MODE2);           //Set SPI Data Mode to default
}

void rtcCheck(){
    SPI.setClockDivider(SPI_CLOCK_DIV16);        //Set SPI Clock Speed
    SPI.setDataMode(SPI_MODE2);                  //Set SPI Data Mode
    const int len = 32;                          //buffer length for RTC display on Serial comm
    static char buf[len];                        //string for RTC display on Serial comm
    DateTime now = RTC.now();                    // Get time from RTC
    Serial.print(F("***Current RTC Time: "));
    Serial.println(now.toString(buf,len));        
    Serial.print(F("---Unit Start Time:  "));
    Serial.print(unitStart.toString(buf,len));        
    Serial.println(F("   (Unit will take its first sample at this time)"));
    Serial.println(); 
}


void rtcSystemSync(){
    SPI.setDataMode(SPI_MODE1);		                                  //Set SPI Data Mode to 1 for RTC
    RTC.adjust(DateTime(__DATE__, __TIME__));                             //Sync RTC with System Clock
    SPI.setDataMode(SPI_MODE2);		                                  //Set SPI Data Mode to default
    Serial.println(F("***RTC Time Set to program compile date/time!"));   
    rtcCheck();                                                           //Display the RTC date/time
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
      Serial.println();
    }
}

void sdCardCheck(){
    SPI.setDataMode(SPI_MODE0);                            //Set SPI Data Mode
    Sd2Card card;
    if (!card.init(SPI_HALF_SPEED, SDCARD_CS)) {
      Serial.println();
      Serial.println(F("***Initialization FAILED. Is a card is inserted? "));
      return;
    } 
    else {
      Serial.println();
      Serial.println(F("***SD Card Initialized successfully.  Card is present and communicating properly.")); 
    }        
    SPI.setDataMode(SPI_MODE2);                            //Set SPI Data Mode
}

void sdDataCheck(){
    Serial.println(F("***Checking SD Card Files..."));      // if the file didn't open, print an error:
    SPI.setDataMode(SPI_MODE0);                            //Set SPI Data Mode
    File myFile = SD.open("botdat.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println(F("botdat.txt:"));                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println(F("error opening botdat.txt"));      // if the file didn't open, print an error:
    }
    
    myFile = SD.open("icedat.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                              //write to serial window
      Serial.println(F("icedat.txt:"));                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println(F("error opening icedat.txt"));      // if the file didn't open, print an error:
    }
    
    myFile = SD.open("prodat.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println(F("prodat.txt:"));                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println("error opening prodat.txt");      // if the file didn't open, print an error:
    }
  
  myFile = SD.open("summary.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println(F("summary.txt:"));                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println(F("error opening summary.txt"));      // if the file didn't open, print an error:
    } 
    
  myFile = SD.open("filepos.txt", FILE_READ);                    //re-open the file for reading:
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.println();                   //write to serial window
      Serial.println(F("filepos.txt:"));                   //write to serial window
      while (myFile.available()) {                   //read from the file until there's nothing else in it:
      	Serial.write(myFile.read());                 //write output from file to serial window
      }
      Serial.println();                              //write to serial window
      myFile.close();                                //close the file
    } 
    else {
      Serial.println(F("error opening filepos.txt"));      // if the file didn't open, print an error:
    }
    SPI.setDataMode(SPI_MODE2);                            //Set SPI Data Mode
    Serial.println();                              //write to serial window
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


