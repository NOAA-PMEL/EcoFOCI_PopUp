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
//#define IRIDIUM_RXD A14
//#define IRIDIUM_TXD A15

#define keller3bar 0x40       		//Define pressure sensor control register
#define keller10bar 0x41      		//Define pressure sensor control register
#define checkpressure 0xAC    		//Define pressure sensor control register
#define ads1100A0 0x48        		//Define tempADC control register
#define ads1100A1 0x49        		//Define parADC control register

SoftwareSerial ss(GPS_TX,GPS_RX);		//Software Serial Object for GPS Communication 
//SoftwareSerial nss(IRIDIUM_TXD,IRIDIUM_RXD);	//Software Serial Object for Iridium Communication
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

DateTime unitStart =  DateTime(16,11,9,12,15,0);            //Date and Time for first sample: 				DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime releaseTime =  DateTime(16,11,9,12,25,0);          //Date and Time for actual release of burn wire: 		DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime sendDataDate = DateTime(16,11,9,12,30,0);          //Date and Time to attempt data transmission twice a day: 	DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime iceFreeDate = DateTime(16,11,9,12,30,0);           //Date and Time to attempt data transmission every hour: 	DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
#define FAILSAFE_TIMER 86400				    //Alarm for Failsafe if program freezes somehow 		(nominal FAILSAFE_TIMER 86400 = 1 day)
#define BOTTOM_SAMPLE_INTERVAL 180			    //Interval between bottom samples 				(nominal BOTTOM_SAMPLE_INTERVAL 21600 = 6 hours)
#define PRE_RELEASE_WAKEUP_TIME 180			    //How long before planned release to wait for profile 	(nominal PRE_RELEASE_WAKEUP_TIME 21600 = 6 hours)
#define UNDER_ICE_SAMPLE_INTERVAL 180			    //Interval between samples at the surface (or uncer ice)	(nominal UNDER_ICE_SAMPLE_INTERVAL 3600	= 1 hour)
#define PROFILE_LENGTH_SECONDS 30			    //Length of profile in seconds				(nominal PROFILE_LENGTH_SECONDS 60 = 60 seconds)
#define DEPTH_CHANGE_TO_TRIP_PROFILE 1			    //Depth in meters needed to begin profile mode		(nominal DEPTH_CHANGE_TO_TRIP_PROFILE 3	= 3 meters)
#define GPS_SEARCH_SECONDS 180				    //Number of seconds to search for GPS signal		(nominal GPS_SEARCH_SECONDS 120	= 2 minutes)
#define GPS_SECONDS_BETWEEN_SEARCHES 20		            //Number of seconds between GPS Searches to send data	(nominal GPS_SECONDS_BETWEEN_SEARCHES 180 = 3 minutes)
#define GPS_SUCCESSES_TO_SEND 1				    //Number of successful gps hits before sending data		(nominal GPS_SUCCESSES_TO_SEND 3 = 2 attempts)
#define CHARS_PER_SBDMESSAGE 100		            //Number of bytes to pull from data file and send at once	(nominal CHARS_PER_SBDMESSAGE 50 = 50 bytes)

void setup()
{
  
  initializePins();		            //Initialize Arduino Pins for Output and Set Voltage Levels
  Wire.begin();				    //Start I2C Communication
  SPI.begin();                              //Start up SPI Communication (needed for various ICs) 
  SPI.setDataMode(SPI_MODE2);               //Set SPI Data Mode
  SPI.setBitOrder(MSBFIRST);                //Configure SPI Communication (needed for various ICs)
  SPI.setClockDivider(SPI_CLOCK_DIV16);     //Set SPI Clock Speed

  setupRTCandSD();			    //Start RTC and SD Card 
  Serial.begin(115200);
  turnOnIridiumPower();					    						//switch on voltage to Iridium module 
  delay(50);						    						//short delay to let voltage stabilize 
  int signalQuality = -1;				    						//Initialize signal quality variable (0-5)
  Serial1.begin(19200);  					    						//start serial comms on nss pins defined above
  isbd.attachConsole(Serial);
  //isbd.attachDiags(Serial);

  isbd.setPowerProfile(0); 				    						//1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.setMinimumSignalQuality(2);			    						//minimum signal quality (0-5) needed before sending message.  normally set to 2 (by isbd recommendations)
  isbd.begin();  					    						//wake up the Iridium module and prepare it to communicate

  const int sendbuffer_size = 101;	    			    //calculate size of array for send message buffer
  char iridiumSendBuffer[sendbuffer_size];		    						//initialize array for send message buffer (reserve adequate space in memory)
  iridiumSendBuffer[0] = 'H';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[1] = 'e';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[2] = 'l';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[3] = 'l';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[4] = 'o';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  for(int y=5;y<50;y++){
    iridiumSendBuffer[y] = '!';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)  
  }
  iridiumSendBuffer[50] = '\0';		            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)  
  
  uint8_t iridiumReceiveBuffer[15];			    						//Initialize array for received message (will only send short messages of 12 chars to change filename/position)
  size_t receiveBufferSize = sizeof(iridiumReceiveBuffer);  						//Define size of array for received message 
  int err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, receiveBufferSize);	//Attempt to send first message ("I am alive!")
  if (err != 0){
    return;
  }										//If sending message fails, then exit routine and shutdown unit 
  
  iridiumSendBuffer[0] = 'e';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[1] = 'm';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[2] = 'p';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[3] = 't';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[4] = 'y';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[5] = '\0';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
  
  err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, receiveBufferSize);	//Attempt to send first message ("I am alive!")
  if (err != 0){
    return;
  }

  SPI.setDataMode(SPI_MODE0);                                                           //Set SPI Data Mode to 0 for sd card
  File currentFile=SD.open("icedat.txt", FILE_READ);									//Initialize file variable
  if(currentFile){									//If file is open for reading
    for(int i=0;i<CHARS_PER_SBDMESSAGE;i++){			                //parse one char at a time for length of message
      if(currentFile.available()){						        //If not yet at end of the file
        iridiumSendBuffer[i]=char(currentFile.read());  			                //put the next char into the send message buffer
      }      
      else{										//If end of file has been reached
        iridiumSendBuffer[i]='\0';							//Mark the end of the message will null character
        currentFile.close();							//close the file
        SPI.setDataMode(SPI_MODE2);                      		                        //Set SPI Data Mode back to default
        return;
      }
    }
    iridiumSendBuffer[CHARS_PER_SBDMESSAGE]='\0';				                //if send message buffer is filled to max size, mark the end with null character
    currentFile.close();
  }  
  else{
    iridiumSendBuffer[0] = '?';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
    iridiumSendBuffer[1] = ',';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
    iridiumSendBuffer[2] = 'n';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
    iridiumSendBuffer[3] = 'o';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
    iridiumSendBuffer[4] = 'd';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
    iridiumSendBuffer[5] = 'a';			            						//First message to send if unit wake up is "Hello!"  (preferred to not send data in case of file corruption - unit will still respond)
    iridiumSendBuffer[6] = 't';
    iridiumSendBuffer[7] = 't';
    iridiumSendBuffer[8] = '\0';
  }
  SPI.setDataMode(SPI_MODE2);                      		                        //Set SPI Data Mode back to default


  err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, receiveBufferSize);	//attempt to send message
  if (err != 0){
    return;
  }										//If sending message fails, then exit routine and shutdown unit 
  turnOffIridiumPower();										//Once unit is done sending messages, turn off power to the iridium module 
}

void loop()
{
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

void turnOnIridiumPower(){
  digitalWrite(IRIDIUM_POWER,LOW);  		//Pin LOW=Power ON (P-Channel switch)
  digitalWrite(IRIDIUM_ON_OFF,HIGH);  		//Digital Pin for Iridium module sleep.  HIGH = wake from sleep
}
void turnOffIridiumPower(){
  digitalWrite(IRIDIUM_POWER,HIGH);  		//Pin HIGH=Power OFF (P-Channel switch)
  digitalWrite(IRIDIUM_ON_OFF,LOW);  		//Digital Pin for Iridium module sleep.  LOW = go to sleep
}
