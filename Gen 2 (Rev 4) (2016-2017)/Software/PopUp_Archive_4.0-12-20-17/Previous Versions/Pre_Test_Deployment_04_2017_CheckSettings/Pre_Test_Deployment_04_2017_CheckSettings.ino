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
#define RECEIVE_BUFFER_SIZE 15		//Size of buffer to receive Iridium Messages
#define FAILSAFE_TIMER 86400            //Alarm for Failsafe if program freezes somehow 		(nominal FAILSAFE_TIMER 86400 = 1 day)
#define CHARS_PER_SBDMESSAGE 100        //Number of bytes to pull from data file and send at once	(nominal CHARS_PER_SBDMESSAGE 100 = 100 bytes)
#define GPS_SEARCH_SECONDS 120		//Number of seconds to search for GPS signal		        (nominal GPS_SEARCH_SECONDS 120	= 2 minutes)
#define GPS_SECONDS_BETWEEN_SEARCHES 20	//Number of seconds between GPS Searches to send data	        (nominal GPS_SECONDS_BETWEEN_SEARCHES 180 = 3 minutes)
#define GPS_SUCCESSES_TO_SEND 1		//Number of successful gps hits before sending data	        (nominal GPS_SUCCESSES_TO_SEND 3 = 3 attempts)
#define SEND_DATA_INTERVAL 300		//Default interval to send data once GPS has been found	        (nominal SEND_DATA_INTERVAL 300 = 5 minutes)

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
unsigned int pressureOffset = 16384;
int tempOffset = 21000;
int tempRefOffset = 5600;
int PAROffset = 4700;
//********************************************************************************************************************************************************//
//***SET DATES AND PROGRAM INTERVALS HERE*****************************************************************************************************************//
//********************************************************************************************************************************************************//

DateTime unitStart =    DateTime(17,9,27,0,0,0);           //Date and Time for first sample: 				DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime releaseTime =  DateTime(18,3,15,15,0,0);           //Date and Time for actual release of burn wire: 		DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
DateTime iceFreeDate =  DateTime(18,3,15,18,0,0);           //Date and Time to search for GPS every hour: 	        DateTime(YEAR,MON,DAY,HOUR,MIN,SEC);
#define BOTTOM_SAMPLE_INTERVAL 21600			    //Interval between bottom samples 				(nominal BOTTOM_SAMPLE_INTERVAL 21600 = 6 hours)
#define PRE_RELEASE_WAKEUP_TIME 7200			    //How long before planned release to wait for profile 	(nominal PRE_RELEASE_WAKEUP_TIME 7200 = 2 hours)
#define UNDER_ICE_SAMPLE_INTERVAL 3600			    //Interval between samples at the surface (or uncer ice)	(nominal UNDER_ICE_SAMPLE_INTERVAL 3600	= 1 hour)
#define PROFILE_LENGTH_SECONDS 90			    //Length of profile in seconds				(nominal PROFILE_LENGTH_SECONDS 60 = 60 seconds)
#define DEPTH_CHANGE_TO_TRIP_PROFILE 3			    //Depth in meters needed to begin profile mode		(nominal DEPTH_CHANGE_TO_TRIP_PROFILE 3	= 3 meters)

//********************************************************************************************************************************************************//
//***END PROGRAM DATES AND SAMPLE INTERVAL SECTION********************************************************************************************************//
//********************************************************************************************************************************************************//

void setup(){
  Serial.begin(115200);
  initializePins();		            //Initialize Arduino Pins for Output and Set Voltage Levels
  Wire.begin();				    //Start I2C Communication
  SPI.begin();                              //Start up SPI Communication (needed for various ICs) 
  SPI.setDataMode(SPI_MODE2);               //Set SPI Data Mode
  SPI.setBitOrder(MSBFIRST);                //Configure SPI Communication (needed for various ICs)
  SPI.setClockDivider(SPI_CLOCK_DIV16);     //Set SPI Clock Speed

  setupRTCandSD();			    //Start RTC and SD Card 
  setupAccel();				    //Start Accelerometer

  turnOn3v3SensorPower();		    //turn on 3.3V Power to sensors (PAR, TEMP, PRESSURE)
  turnOn5vSensorPower();		    //turn on 5V Power to sensors (Needed for 5V I2C communication)
  delay(50);				    //Short delay here to start ADCs
  setupTempADC();			    //Start Up the ADC for Temperature Measurement
  setupPARADC();			    //Start Up the ADC for PAR Measurement
  delay(500);				    //Longer delay here to let voltages stabilize
  
  char mode = selectModeandSetAlarm();	    //Select mode based on current time and programmed dates and set the alarm
  if(mode>0){
    sampleData(mode);    	            //Sample sensors and store data set 
  }
  
  if(mode==0){			            //Mode 0 = Before Initial sample date
    displayDeploymentParameters();          //Display Deployment Parameters to Serial window for user on initial setup
    shutdownUnit();			    //Turn off power and wait
  }
  else if(mode==1){			    //Mode 1 = On Bottom, Not waiting for release 
    shutdownUnit();			    //Turn off power and wait
  }
  else if(mode==2){			    //Mode 2 = On Bottom, waiting for profile to begin
    waitForProfile();  			    //continuously monitor depth waiting for change to signal start of profile
    collectProfile();			    //collect samples and store data at high sample rate for pre-determined length of time
    shutdownUnit();			    //Turn off power and wait
  }
  else if(mode==4){			    //Mode 4 = Under Ice, too early to attempt data transmission
    underIceMode();  		            //Look for GPS twice a day, Attempt to Send Iridium Data if position found
    shutdownUnit();			    //Turn off power and wait
  }
  else if(mode==5){			    //Mode 5 = Possibly Free of ice, attempt to transmit data
    iceFreeMode(); 		            //Look for GPS, Attempt to Send Iridium Data if position found
    shutdownUnit();			    //Turn off power and wait
  }    
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
  beginSDsuccess = SD.begin(SDCARD_CS);                  //Start SD Card so it is available for other function calls 
  SPI.setDataMode(SPI_MODE2);           //Set SPI Data Mode to default
}

char selectModeandSetAlarm(){
  alarmtime = wakeupTime.unixtime() + (long)FAILSAFE_TIMER;  		//failsafe if program freezes (t+1day)																				
  char modeSelect=0;  						        //initialize variable to select mode	

  if(wakeupTime.unixtime()<unitStart.unixtime()){               	//if time is before first sample time, set alarm for first sample time and go to sleep																		
    modeSelect=0;
    alarmtime = unitStart.unixtime();
  }	
  else if((wakeupTime.unixtime()<(releaseTime.unixtime()-(long)PRE_RELEASE_WAKEUP_TIME))&&wakeupTime.unixtime()>=unitStart.unixtime()){      	  //if time is earlier than pre-release wakeup, just take bottom sample and go to sleep																								
    modeSelect=1;	
    unsigned long preReleaseWakeUpBuffer=(long)PRE_RELEASE_WAKEUP_TIME+(long)BOTTOM_SAMPLE_INTERVAL+10;					          //must be at least this many seconds from release to take normal bottom sample.  10s buffer added so unit does not skip the correct wake up time
    if(wakeupTime.unixtime()<(releaseTime.unixtime()-preReleaseWakeUpBuffer)){          						          //If still far enough from release for a normal bottom sample, set alarm for time + BOTTOM_SAMPLE_INTERVAL     
      alarmtime = wakeupTime.unixtime()+(long)BOTTOM_SAMPLE_INTERVAL;            				
    }   	
    else{    																          //If within 1 BOTTOM_SAMPLE_INTERVAL of pre release wake up, then set alarm for pre release wake up time                                                            
      alarmtime = releaseTime.unixtime()-(long)PRE_RELEASE_WAKEUP_TIME;          
    } 
  }
  else if((wakeupTime.unixtime()>=(releaseTime.unixtime()-(long)PRE_RELEASE_WAKEUP_TIME))&&(wakeupTime.unixtime()<=(releaseTime.unixtime()))){    //if time is equal or after pre-release wakeup but before or equal to release, then wait for profile
    modeSelect=2;
    unsigned long postReleaseWakeUp=(long)PRE_RELEASE_WAKEUP_TIME+(long)UNDER_ICE_SAMPLE_INTERVAL;
    alarmtime = releaseTime.unixtime()+postReleaseWakeUp;       
  }  
  else if((wakeupTime.unixtime()>releaseTime.unixtime())&&(wakeupTime.unixtime()<iceFreeDate.unixtime())){     		                  //if after Release Date and Before Send Data Date then sample data and go to sleep. (Look for GPS once a day and Don't attempt to transmit data)
    modeSelect=4;
    alarmtime = wakeupTime.unixtime()+(long)UNDER_ICE_SAMPLE_INTERVAL;     	
  }  
  else if(wakeupTime.unixtime()>=iceFreeDate.unixtime()){        									          //if after Send Data Date, look for GPS and send Iridium Attempt if GPS lock found								                                                        
    modeSelect=5;
    alarmtime = wakeupTime.unixtime()+(long)UNDER_ICE_SAMPLE_INTERVAL;     	
  }  
  
  setAlarmTime();		//Set the Alarm by writing to RTC registers
  return modeSelect; 		//return mode for program flow
}

void displayDeploymentParameters(){
  Serial.begin(115200);
  const int len = 32;                                                                   //buffer length for RTC display on Serial comm
  static char buf[len];                                                                 //string for RTC display on Serial comm
  
  Serial.println(F("Unit Status:"));
  if(beginSDsuccess){
    Serial.println(F("  SD Card Initialized Successfully."));
  }
  else{
    Serial.print(F("  **SD Card error.  Check if SD Card is present."));
  }
  SPI.setDataMode(SPI_MODE1);                         	                                //Set SPI mode to 1 for RTC 
  DateTime now = RTC.now();
  Serial.print(F("  Current RTC Time/Date: "));
  Serial.println(now.toString(buf,len));

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
  
  int parVal = readPAR();
  Serial.print(F("  PAR  Sensor ADC Val: ")); 
  Serial.print(parVal);
  float parCalc = (parVal-4842)*0.065753;
  Serial.print(F(" (approx "));
  Serial.print(parCalc);
  Serial.println(F(" umol/(m2s))"));
  
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
  
  float accelReading = readAccel();
  Serial.print(F("  Accelerometer Reading: "));
  Serial.print(accelReading,1);
  Serial.print(F(" (approx "));
  float tiltAngle = acos(accelReading/1000)/3.14159*180;			    //Get Tilt Angle from Accelerometer
  Serial.print(tiltAngle,1);
  Serial.println(F(" degrees from vertical)")); 
      
  Serial.println(F("\nOffset values for post-calibration:"));
  Serial.print(F("  Pressure Offset: "));
  Serial.println(pressureOffset);
  Serial.print(F("  Temperature Offset: "));
  Serial.println(tempOffset);
  Serial.print(F("  Temperature Reference Offset: "));
  Serial.println(tempRefOffset);
  Serial.print(F("  PAR Offset: "));
  Serial.println(PAROffset);
 
  Serial.println(F("\nDeployment Schedule:"));
  Serial.println(F("--PHASE 1--"));  
  Serial.print(F("  Unit will start sampling ["));
  Serial.print(unitStart.toString(buf,len));
  Serial.print(F("] and take a sample every "));
  displayHoursMinsSecs(BOTTOM_SAMPLE_INTERVAL);
  
  Serial.println(F("\n--PHASE 2--"));  
  Serial.print(F("  Release Expected ["));
  Serial.print(releaseTime.toString(buf,len));
  Serial.println(F("] (<--Set Burn Wire Release for this exact time/date.)"));
  Serial.print(F("  Unit will wake up  "));
  displayHoursMinsSecs(PRE_RELEASE_WAKEUP_TIME);  
  Serial.println(F(" before scheduled release and wait for profile."));
  Serial.print(F("  Start of profile will be triggered by a change in depth of ("));
  Serial.print(DEPTH_CHANGE_TO_TRIP_PROFILE);
  Serial.println(F(" meters)."));
  Serial.print(F("  Profile will sample at 4 Hz for "));
  displayHoursMinsSecs(PROFILE_LENGTH_SECONDS);  
  Serial.println(F(" seconds before proceeding to phase 3. "));
  Serial.print(F("  If unit does not sense the release within  "));
  displayHoursMinsSecs(PRE_RELEASE_WAKEUP_TIME*(unsigned long)2);  
  Serial.println(F(" of waking it will assume it missed the profile and proceed to phase 3."));

  Serial.println(F("--PHASE 3--"));  
  Serial.print(F("  Unit will assume it is under ice until ["));
  Serial.print(iceFreeDate.toString(buf,len));
  Serial.print(F("]\n  Unit will take a sample every "));
  displayHoursMinsSecs(UNDER_ICE_SAMPLE_INTERVAL);  
  Serial.println(F(" and search for GPS location only once a day to conserve battery power."));

  Serial.println(F("--PHASE 4--"));  
  Serial.print(F("  Unit will no longer assume it is under ice after ["));
  Serial.print(iceFreeDate.toString(buf,len));
  Serial.print(F("]\n  Unit will take a sample and search for GPS location every "));
  displayHoursMinsSecs(UNDER_ICE_SAMPLE_INTERVAL);  
  Serial.print(F("\n  Unit will search for GPS ("));
  Serial.print(GPS_SUCCESSES_TO_SEND);
  Serial.print(F(" times) for "));
  displayHoursMinsSecs(GPS_SEARCH_SECONDS);  
  Serial.print(F(" with "));
  displayHoursMinsSecs(GPS_SECONDS_BETWEEN_SEARCHES);  
  Serial.println(F("  between each attempt to confirm a lock."));
  Serial.println(F("  If unit does successfully find a GPS lock, it will attempt to send data back in the following order:"));
  Serial.println(F("  1-File Summary, 2-Profile Data, 3-Under Ice Data, 4-Bottom Data"));  
  Serial.print(F("  If any messages send successfully, the unit will automatically make another attempt in "));  
  displayHoursMinsSecs(SEND_DATA_INTERVAL);  
  Serial.print(F("\n  Otherwise, the unit will return to sampling and searching for GPS every "));
  displayHoursMinsSecs(UNDER_ICE_SAMPLE_INTERVAL);  

  Serial.println(F("\n\n**Check all settings, take a screenshot of this dialog, and disconnect from USB when ready.**"));
  Serial.print(F("**Set Burn Wire Release for ["));
  Serial.print(releaseTime.toString(buf,len));
  Serial.println(F("]**\n "));
  
  if(!beginSDsuccess){
    Serial.println(F("**WARNING!!! SD Card error.  Check if SD Card is present.**"));
  }
  if(now.unixtime()>unitStart.unixtime()){
    Serial.println(F("**WARNING!!! Start time/Date is before current time. Set start time/date to after current time to prevent errors.**"));
  }
  if(unitStart.unixtime()>releaseTime.unixtime()){
    Serial.println(F("**WARNING!!! Expected release is before start time/date. Check all time/date settings.**"));
  }
  if(releaseTime.unixtime()>iceFreeDate.unixtime()){
    Serial.println(F("**WARNING!!! Ice Free time/date is before expected release. Check all time/date settings.**"));
  }
}

void displayHoursMinsSecs(unsigned long numSeconds){
  unsigned long displayIntervalHours=numSeconds/(unsigned long)3600;
  unsigned long displayIntervalMinutes=(numSeconds%(unsigned long)3600)/(unsigned long)60;
  unsigned long displayIntervalSeconds=numSeconds%(unsigned long)60;
  
  Serial.print(F("(")); 
  Serial.print(displayIntervalHours);
  Serial.print(F("h,")); 
  Serial.print(displayIntervalMinutes); 
  Serial.print(F("m,")); 
  Serial.print(displayIntervalSeconds);
  Serial.print(F("s)"));    
}


void sampleData(char modeSelected){
  unsigned int pressureVal10 = readPressure(true) - pressureOffset;   //Read 10 bar Pressure Sensor
  unsigned int pressureVal3 = readPressure(false) - pressureOffset;   //Read 3 Bar Pressure Sensor
  float tiltInvCos = 1000 - readAccel();		              //Get Tilt Angle from Accelerometer
  int tempVal = tempOffset - readTemp(false);		              //Read Thermistor Resistor Value
  int tempRefVal = 0;					              //Initialize Temperature Reference Value
  if(modeSelected!=3){					              //If not profiling, then check and store the temperature reference value (This takes ~1 second for voltage to stabilize on switch. Profile must sample much faster and reference value is not needed at every sample for data quality)
    tempRefVal = tempRefOffset - readTemp(true);	              //Read Temperature Reference Resistor Value
  }
  int parVal = readPAR() - PAROffset;		            //Read PAR Sensor
  SPI.setDataMode(SPI_MODE0);                               //Set SPI Data Mode to 0 for SDCard 
  File dataFile;					    //Initialize dataFile
  if(modeSelected==1){                                      //If just a bottom sample
    dataFile = SD.open("botdat.txt", FILE_WRITE);           //open file with write permissions
  }
  else if(modeSelected==2||modeSelected==3){		    //if profile or waiting for profile
    dataFile = SD.open("prodat.txt", FILE_WRITE);           //open file with write permissions      
  }
  else if(modeSelected==4||modeSelected==5){		    //If under ice or at surface
    dataFile = SD.open("icedat.txt", FILE_WRITE);           //open file with write permissions
  }
  if (dataFile) {                                           //if file is available, write to it:
    dataFile.print(F("/"));                                 //Mark the start of a new dataset
    if(modeSelected==3){				    //If in profiling mode, then we want a new timestamp for each sample
      SPI.setDataMode(SPI_MODE1);                           //Set SPI Data Mode to 2 for reading RTC
      DateTime now = RTC.now();				    //Get Timestamp from RTC
      SPI.setDataMode(SPI_MODE0);                           //Set SPI Data Mode to 0 for SD Card 
      dataFile.print(now.unixtime(),HEX);                   //write timestamp to file
    }
    else{
      dataFile.print(wakeupTime.unixtime(),HEX);            //If not in profiling mode, use the timestamp for when the unit woke up
    }
    dataFile.print(F(","));                                 //,
    dataFile.print(pressureVal10,HEX);                      //ADC Value from 10 bar pressure sensor
    dataFile.print(F(","));                                 //,
    dataFile.print(pressureVal3,HEX);                       //ADC Value from 3 bar pressure sensor
    dataFile.print(F(","));                                 //,
    dataFile.print(tempRefVal,HEX);                         //ADC Value from Temperature Reference Resistor
    dataFile.print(F(","));                        	    //,
    dataFile.print(tempVal,HEX);                   	    //ADC Value from Temperature Thermistor
    dataFile.print(F(","));                      	    //,
    dataFile.print(parVal,HEX);                     	    //ADC Value from PAR Sensor
    dataFile.print(F(","));                      	    //,
    dataFile.print(tiltInvCos,0);                 	    //1000*Cosine of angle from vertical (4 decimal places gives <<1 degree resolution, factor of 1000 removes decimal place and 0)
    dataFile.close();					    //Close data file (Also flushes datastream to make sure all data is written)
  }
  SPI.setDataMode(SPI_MODE2);                         	    //Set SPI mode to default
}

void waitForProfile(){
  unsigned int pressureAverageLong = averagePressureLong();			//store baseline reading for average pressure (20 readings over 20 seconds)
  unsigned int pressureAverageShort = averagePressureShort();			//store reading for current pressure (3 readings over 300 ms)
  unsigned int pressureChangeVal = 328*(int)DEPTH_CHANGE_TO_TRIP_PROFILE;	//calculate ADC reading change needed for desired depth change 
  SPI.setDataMode(SPI_MODE1);                                     		//Set SPI Data Mode to 1 for reading RTC
  DateTime now = RTC.now();							//Get current time from RTC
  while(pressureAverageShort>(pressureAverageLong-pressureChangeVal)){          //while short average of last 3 depth readings is less still deeper than cutoff depth to trigger profile
    now = RTC.now();                                     			//update the RTC time
    if(now.unixtime()>(releaseTime.unixtime()+(long)PRE_RELEASE_WAKEUP_TIME)){  //if past expected release window, shut the unit down (alarm already set for releaseTime+Wait Period+Sample Interval)
      missedProfile();						                //store message on data file indicating unit missed the profile
      shutdownUnit();							        //immediately just shut down unit and don't continue with collect profile function
    }
    delay(500);								        //wait half a second between sample bins
    pressureAverageShort=averagePressureShort();                  		//update the short depth average
  }
}

unsigned int averagePressureLong(){						//function to get baseline depth reading (used to establish depth before while waiting for profile)
  long depthAverage=0;								//initialize depth average (long datatype so we can sum 20 samples without overflow)
  for(char sample=0;sample<20;sample++){					//do this 20 times
    depthAverage=depthAverage+readPressure(true); 				//running sum of depth reading from 10 bar sensor
    delay(1000);								//delay 1 second between samples
  }
  depthAverage=depthAverage/20;							//divide sum by 20 to get an average (will truncate any decimals)
  return depthAverage;								//return average value, will automatically convert to unsigned int type
}

unsigned int averagePressureShort(){						//function to get an average of 3 depth readings to see if unit is ascending and profile should begin
  long depthAverage=0;								//initialize depth average (long datatype so we can sum 3 samples without overflow)
  for(char sample=0;sample<3;sample++){						//do this 3 times
    depthAverage=depthAverage+readPressure(true); 				//running sum of depth reading from 10 bar sensor
    delay(100);									//delay 100ms between samples
  }
  depthAverage=depthAverage/3;							//divide sum by 3 to get an average (will truncate any decimals)
  return depthAverage;								//return average value, will automatically convert to unsigned int type 
}

void collectProfile(){								//function to collect data once profile has been triggered by depth change 
  long profileLengthMillis=(long)PROFILE_LENGTH_SECONDS*(long)1000;	        //calculate length of profile in milliseconds
  long endProfileTime=millis()+(long)profileLengthMillis;        		//timestamp for end of profile
  long quarterSecond=millis()+(long)250;  					//timestamp to trigger next data sample
  while(millis()<endProfileTime){						//while current timestamp is less than profile cutoff timestamp, keep taking samples
    while(millis()<quarterSecond){
    }						                                //loop to take sample every 250 ms (4Hz)
    sampleData(3);								//function call to sample sensors (3=profile mode, don't take reference resistor reading)
    quarterSecond=millis()+250;						        //timestamp to trigger next data sample
  }
}

void missedProfile(){						        //function to write message to sd card if profile was never triggered at appropriate time 
  SPI.setDataMode(SPI_MODE0);                                     	//Set SPI Data Mode to 0 for SDCard
  File dataFile=SD.open("prodat.txt", FILE_WRITE);			//open file with write permissions
  if (dataFile) {                                           	        //if the file is open
    dataFile.print(F(",MISSED_PROFILE")); 				//print message saying profile has been missed
    dataFile.close(); 						        //close the file
  }
  SPI.setDataMode(SPI_MODE2);                               	        //Set SPI Data Mode back to normal
}

void underIceMode(){							//mode for when unit is expected to be locked under ice, don't waste time or battery trying to send data
  turnOff3v3SensorPower();						//turn off 3.3V power for sensors (no longer needed)
  turnOff5vSensorPower();						//turn off 5V power for sensors (no longer needed)
  if(wakeupTime.hour()==15){		                                //if hour==15 (3pm, usually warmest time of day...most likely to be ice free)
    lookForGPS(1);						        //look for GPS once and store position if found
  }
  if(foundGPS==true){							//if GPS is found successfully n times in a row
    sendIridiumData();							//try to send data over iridium
  }
}									//other than looking for GPS once a day, unit will just sample data and go back to sleep in this mode

void iceFreeMode(){							                                //mode for when unit is expected to be possibly free of ice
  turnOff3v3SensorPower();										//turn off 3.3V power for sensors (no longer needed)
  turnOff5vSensorPower();										//turn off 5V power for sensors (no longer needed)
  lookForGPS(GPS_SUCCESSES_TO_SEND);									//look for GPS n times.  send data if found successfully n times in a row
  if(foundGPS==true){											//if GPS is found successfully n times in a row
    sendIridiumData();											//try to send data over iridium
  }													//if GPS is not found n times successfully, then shut down unit 
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

void lookForGPS(byte gpsTries){										//function to search for GPS - looks for GPS n times and returns foundGPS=true if found successfully n times
  float maxtiltInvCos = readAccel();									//initialize max tilt angle variable
  float tiltInvCos=maxtiltInvCos;									//initialize tilt angle reading, set to same as max reading
  for(int n=0;n<gpsTries;n++){										//loop n times
    Serial.print(F("gpsTries:"));
    Serial.println(gpsTries);
    Serial.print(F("n:"));
    Serial.println(n);
    foundGPS=false;										//start each loop assuming GPS has not been found
    turnOnGPSPower();										//turn on power to GPS chip
    delay(100);											//short delay to let GPS chip initialize and begin streaming data
    ss.begin(9600);										//begin software serial comms at 9600 baud with GPS chip
    unsigned long gpsSearchMillis = (long)1000*(long)60;					//calculate number of milliseconds to search for GPS
    unsigned long startSearchMillis=millis();							//timestamp for when GPS search started  (we want to log the time to fix)
    unsigned long endSearchMillis=(long)startSearchMillis+(long)gpsSearchMillis;		//timestamp for when to end GPS search
    while((millis()<endSearchMillis)&&(foundGPS==false)){					//while time to search is not over and gps hasn't been foundGPS
      while ((ss.available() > 0)&&(foundGPS==false)){						//while data stream from GPS chip is incoming and gps hasn't been found	
        if (gps.encode(ss.read())){								//parse incoming data and everytime a full message is encoded, execute the following
          displayInfo();
          if(n==0){  									        //if this is the first time searching, we want to record the max tilt angle during search (some indication of sea state)
            tiltInvCos = readAccel();							        //read instantaneous tilt angle
            if(tiltInvCos<maxtiltInvCos){							//if new tilt angle is more than old tilt angle (current cosine value is less than old cosine value)
              maxtiltInvCos=tiltInvCos;						                //store the current cosine value into the max angle
            }
          }				
          if (gps.location.isValid()&&gps.date.isValid()&&gps.time.isValid()){		        //if a valid fix is found (location, time, and date all valid)
            foundGPS=true;								        //flag GPS fix has been found
            if(n==0){  									        //if this is the first time searching, then record the fix, time to fix, and max tilt angle during search (some indication of sea state) 
              unsigned long timetofix=millis()-startSearchMillis;
              storeGPSFix(gps.location.lat(), gps.location.lng(), gps.time.value(), gps.date.value(), timetofix, maxtiltInvCos);    //send data from GPS position to SD File
            }
          }
        }
        if((foundGPS==true)||(millis()>=endSearchMillis)){                                      //if GPS position has been found or loop has timed out
          ss.end();                                                                             //end the serial comms with the GPS 
        }
      }
    }												//keep searching and parsing data until GPS has been found or search timeout is exceeded
    turnOffGPSPower();										//turn off GPS power in between searches (last position stored in chip and has coin cell for hot start)
    if(foundGPS==true&&(n<gpsTries-1)){								//if unit still needs to search for GPS again
      long gpsDelaySearchMillis = (long)GPS_SECONDS_BETWEEN_SEARCHES*(long)1000;		//calculate the delay needed between searches in milliseconds
      delay(gpsDelaySearchMillis);								//delay between searches
    }
    else{											//if this is the last time to search
      n=gpsTries;										//increment n so search isn't executed again
    }
  }
}

void storeGPSFix(double gpsLat, double gpsLon, unsigned long gpsTimeVal, unsigned long gpsDateVal, unsigned long timeToFix, float maxtiltInvCos){
  SPI.setDataMode(SPI_MODE1);                           //Set SPI Data Mode to 1 for reading RTC
  DateTime now = RTC.now();				//Get current time to compare to GPS time for clock drift
  SPI.setDataMode(SPI_MODE0);                        	//Set SPI Data Mode to 0 for SD Card Use
  File dataFile = SD.open("icedat.txt", FILE_WRITE); 	//open the file with write permissions
  timeToFix=timeToFix/1000;				//convert timetofix from ms to seconds
  Serial.println(F("timeToFix="));
  Serial.println(timeToFix);
  if (dataFile) {                                    	//if the file is available, write to it:
    dataFile.print(F(","));                          	//, 	
    dataFile.print(gpsLat,4);                     	//GPS Latitude with 4 decimal places (~11m accuracy, best possibly without DGPS)   		
    dataFile.print(F(","));                             //,	
    dataFile.print(gpsLon,4);                           //GPS Latitude with 4 decimal places (~11m accuracy, best possibly without DGPS) 
    dataFile.print(F(","));                          	//,	
    dataFile.print(gpsDateVal);                         //GPS Date (DDMMYY format)
    dataFile.print(F(","));                             //,	
    dataFile.print(gpsTimeVal);                         //GPS Time (HHMMSSCC format)
    dataFile.print(F(","));                             //,	
    dataFile.print(timeToFix);		                //Seconds to acquire fix (helps with diagnostics)
    dataFile.print(F(","));                          	//,	
    dataFile.print(maxtiltInvCos,0);			//maxTilt while searching for GPS	//1000*Cosine of angle from vertical (4 decimal places gives <<1 degree resolution, factor of 1000 removes decimal place and 0) 
    dataFile.print(F(","));                          	//,	
    dataFile.print(now.unixtime(),HEX);			//RTC Time in UNIXTIME
    dataFile.close();				        //Close data file (Also flushes datastream to make sure all data is written)
  }
  SPI.setDataMode(SPI_MODE2);                         	//Set SPI Data Mode back to default
}

void sendIridiumData(){
  turnOnIridiumPower();					    						//switch on voltage to Iridium module 
  delay(50);						    						//short delay to let voltage stabilize 
  int signalQuality = -1;				    						//Initialize signal quality variable (0-5)
  Serial1.begin(19200);  					    					//start serial comms on nss pins defined above
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);

  isbd.setPowerProfile(0); 				    						//1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.setMinimumSignalQuality(2);			    						//minimum signal quality (0-5) needed before sending message.  normally set to 2 (by isbd recommendations)
  isbd.begin();  					    						//wake up the Iridium module and prepare it to communicate

  readFilePositionSD();                                                                                 //Read the SD Card to determine last sent message if applicable
  setAlarmNextSendAttempt();                                                                            //reset alarm for next send attempt
  const int sendbuffer_size = CHARS_PER_SBDMESSAGE+1;	    			                        //calculate size of array for send message buffer
  char iridiumSendBuffer[sendbuffer_size];		    						//initialize array for send message buffer (reserve adequate space in memory)
  iridiumSendBuffer[0] = 'H';			            						//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[1] = 'e';			            						//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[2] = 'l';			            						//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[3] = 'l';			            						//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[4] = 'o';			            						//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)
  iridiumSendBuffer[5] = '!';			            						//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)  
  iridiumSendBuffer[6] = '\0';		            						        //First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)  
  
  uint8_t iridiumReceiveBuffer[RECEIVE_BUFFER_SIZE];			    						//Initialize array for received message (will only send short messages of 12 chars to change filename/position)
  emptyReceiveBuffer(iridiumReceiveBuffer);
  size_t receiveBufferSize = RECEIVE_BUFFER_SIZE;  						//Define size of array for received message 
  int err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, receiveBufferSize);	//Attempt to send first message ("I am alive!")
  if (err != 0){
      setAlarmNextSendAttempt();                                                                        //reset alarm for next send attempt
      return;
  }										                        //If sending message fails, then exit routine and shutdown unit  
  readReceivedMessage(iridiumReceiveBuffer);                                                            //read the received message and if it contains info on where to start parsing data, then override the info from the SDCard

  if(whichFile=='s'){                                                                                   //if whichFile still='s', then this is the very first message (or we are re-sending it) so we want to get a summary of the data files
    writeFileSummary();                                                                                 //function to collect file size info 
  }

  while(whichFile!='x'){										//'x' is marker for all files being completely sent; as long as this is not true, keep parsing and sending data
    writeFilePosition();                                                                                //record current file position
    setAlarmNextSendAttempt();                                                                          //reset alarm for 1 hour from current time (Failsafe)
    emptyReceiveBuffer(iridiumReceiveBuffer);
    receiveBufferSize = RECEIVE_BUFFER_SIZE; 
    fillIridiumBuffer(iridiumSendBuffer);					                        //Fill the send message buffer based on file and file position 
Serial.println(F("**Sending Message"));
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);
Serial.print(F("  iridiumSendBuffer:"));
Serial.println(iridiumSendBuffer);
Serial.print(F("  freeMemory()="));
Serial.println(freeMemory());
    if(iridiumSendBuffer[0]!=0){									//As long as the message contains data
      err = isbd.sendReceiveSBDText(iridiumSendBuffer, iridiumReceiveBuffer, receiveBufferSize);	//attempt to send message
      if (err != 0){                                                                                    //If sending message fails
Serial.println();
Serial.print(F("err="));
Serial.print(err);
        setAlarmNextSendAttempt();                                                                              //reset alarm for next send attempt
        return;                                                                                         //then exit routine and shutdown unit
      }
      else{                                                                                             //if message is successful
        if(newFile==false){
          filePosition=filePosition+(long)sizeof(iridiumSendBuffer)-1;                                  //increment file position accordingly
        }
      }									                          
    }
    readReceivedMessage(iridiumReceiveBuffer);                                                          //check if received message contains any info on where to go in file structure next
  }
  turnOffIridiumPower();										//Once unit is done sending messages, turn off power to the iridium module 
  setAlarmNextSendAttempt();                                                                            //reset alarm for 1 hour from current time
}

void readFilePositionSD(){
Serial.println(F("**Reading File Position"));
  SPI.setDataMode(SPI_MODE0);                        	                                //Set SPI Data Mode to 0 for SD Card Use
  File dataFile = SD.open("filepos.txt", FILE_READ); 	                                //open the file with read permissions
  if(dataFile){                                                                         //if the data file exists (contains info)
    char readChar=dataFile.read();                                                      //variable for parsing chars
    if(readChar=='s'||readChar=='p'||readChar=='b'||readChar=='i'||readChar=='z'){      //if first char is a valid file designator
      whichFile=readChar;                                                               //first character should be file designator ('p','i','b')
      filePosition=0;                                                                   //Reset the value for file Position (Start of the file)
      readChar=dataFile.read();                                                         //second character in the line should be comma (',') - do nothing with this
      while(dataFile.available()){                                                      //only one line in the file, so just read until the end
        readChar=dataFile.read();                                                       //keep reading characters
        if(readChar>47&&readChar<58){                                                   //if there is a digit in the next slot
          filePosition=filePosition*10+readChar-'0';                                    //multiply the previous value by 10 and add the new digit value
        }
      }
    }
  }
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);  
  SPI.setDataMode(SPI_MODE2);                         	                                //Set SPI Data Mode back to default
}

void emptyReceiveBuffer(uint8_t receiveBuffer[]){
  for(int y=0;y<RECEIVE_BUFFER_SIZE;y++){
    receiveBuffer[y] = '\0';			            					//First message to send if unit wake up is "Hello!!!..."  (preferred to not send data in case of file corruption - unit will still respond)  
  }
}

void readReceivedMessage(uint8_t receiveBuffer[]){                                                                              //Function to parse received message and get file position (example message = 'p,1234567890' or 'p,95')
Serial.println(F("**Reading Received Message:"));
  if(receiveBuffer[0]=='s'||receiveBuffer[0]=='p'||receiveBuffer[0]=='b'||receiveBuffer[0]=='i'||receiveBuffer[0]=='z'){	//if message specifically designates icedat,botdat, prodat, or sleep mode then use that designation
    whichFile=receiveBuffer[0];                                                                                                 //set file designator appropriately
    filePosition=0;					                                                                        //reset value for file position unsigned long is a maximum of 10 digits (4,294,967,295)
    for(int d=2;d<12;d++){					                                                                //for chars 2-11 in the message
      if(receiveBuffer[d]>47&&receiveBuffer[d]<58){				                                                //if there is a digit in the next slot
        filePosition=filePosition*10+receiveBuffer[d]-'0';		                                                        //multiply the previous value by 10 and add the new digit value
      }
    }
  }
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);  
}

void fillIridiumBuffer(char sendbuffer[]){	                                        //function to fill message based on file and file position
Serial.println(F("**Filling Send Buffer:"));
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);  
  SPI.setDataMode(SPI_MODE0);                                                           //Set SPI Data Mode to 0 for sd card
  File currentFile;									//Initialize file variable
  if(whichFile=='s'){									//'s' designates summary data (summary.txt)
    currentFile=SD.open("summary.txt", FILE_READ);		                        //open file with read access
  }
  else if(whichFile=='p'){								//'p' designates profile data (prodat.txt)
    currentFile=SD.open("prodat.txt", FILE_READ);		                        //open file with read access
  }
  else if(whichFile=='i'){								//'i' designates under ice data (icedat.txt)
    currentFile=SD.open("icedat.txt", FILE_READ);  		                        //open file with read access
  }
  else if(whichFile=='b'){								//'p' designates bottom data (botdat.txt)
    currentFile=SD.open("botdat.txt", FILE_READ);  		                        //open file with read access
  }	
  else if(whichFile=='z'){								//'z' designates sleep mode
    currentFile=SD.open("icedat.txt", FILE_READ);					//Initialize file variable
    filePosition=currentFile.available()-(unsigned long)CHARS_PER_SBDMESSAGE+1;         //send the most recent ice data (i.e. position) and go to sleep
    if(filePosition<0){                                                                 //if icedat is short and file position is negatie for some reason
      filePosition=0;                                                                   //set file position to 0
    }
  }
  else if(whichFile=='x'){								//'p' designates bottom data (botdat.txt)
    currentFile=SD.open("filepos.txt", FILE_READ);  		                        //open file with read access
  }
  newFile=false;                                                                        //set the newfile variable so it is a known value	
  if(currentFile){									//If file is open for reading
    currentFile.seek(filePosition);							//move forward to proper file position
    for(int i=0;i<CHARS_PER_SBDMESSAGE;i++){			                        //parse one char at a time for length of message
      if(currentFile.available()){						        //If not yet at end of the file
        sendbuffer[i]=byte(currentFile.read());  			                //put the next char into the send message buffer
      }      
      else{										//If end of file has been reached
        sendbuffer[i]='\0';							        //Mark the end of the message will null character
        currentFile.close();							        //close the file
        nextFile();					                                //Get marker for the next file to read
        filePosition=0;								        //Reset File position to start of next file
        SPI.setDataMode(SPI_MODE2);                      		                //Set SPI Data Mode back to default
        return;
      }
    }
    sendbuffer[CHARS_PER_SBDMESSAGE]='\0';				                //if send message buffer is filled to max size, mark the end with null character
    currentFile.close();                                                                //close the file
    newFile=false;									//flag to tell main send loop not to move on to the next file 
  }  
  else{
    sendbuffer[0] = whichFile;			            				//first character in message is file identifier
    sendbuffer[1] = ',';			            				//rest of message says there is no data in the identified file
    sendbuffer[2] = 'n';			            				
    sendbuffer[3] = 'o';			            				
    sendbuffer[4] = 'd';			            				
    sendbuffer[5] = 'a';			            				
    sendbuffer[6] = 't';
    sendbuffer[7] = 'a';
    sendbuffer[8] = '\0';
    nextFile();					                                        //go to the next file
    filePosition=0;	                                                                
  }
  SPI.setDataMode(SPI_MODE2);                      		                        //Set SPI Data Mode back to default
Serial.print(F("**Send Buffer="));
Serial.println(sendbuffer);
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);  
}

void writeFileSummary(){
Serial.println(F("**Writing File Summary"));
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);  
  SPI.setDataMode(SPI_MODE0);                                                           //Set SPI Data Mode to 0 for sd card
  unsigned long prodatSize=0;                                                           //variable for prodat.txt file size
  unsigned long icedatSize=0;                                                           //variable for icedat.txt file size
  unsigned long botdatSize=0;                                                           //variable for botdat.txt file size
  File dataFile=SD.open("prodat.txt", FILE_READ);			                //Initialize File variable and Open Prodat.txt to get file size
  if(dataFile){                                                                         //if file exists
    prodatSize=dataFile.available();                                                         //get file size (length in bytes)
    dataFile.close();                                                                   //Close the File
  }
  dataFile=SD.open("icedat.txt", FILE_READ);					        //Open Icedat.txt to get file size
  if(dataFile){                                                                         //if file exists
    icedatSize=dataFile.available();                                                         //get file size (length in bytes)
    dataFile.close();                                                                   //Close the File
  }
  dataFile=SD.open("botdat.txt", FILE_READ);					        //Open Botdat.txt to get file size
  if(dataFile){                                                                         //if file exists
    botdatSize=dataFile.available();                                                         //get file size (length in bytes)
    dataFile.close();                                                                   //Close the File
  }
  if(SD.exists("summary.txt")){
    SD.remove("summary.txt");
  }  
  dataFile=SD.open("summary.txt", FILE_WRITE);					        //Create and Open summary.txt 
  dataFile.print(F("prodat:"));                                                          //,
  dataFile.print(prodatSize);                                                           //File Position
  dataFile.print(F(",icedat:"));                                                               //,
  dataFile.print(icedatSize);                                                           //File Position
  dataFile.print(F(",botdat:"));                                                         //,
  dataFile.print(botdatSize);                                                           //File Position
  dataFile.print(F("!"));                                                               //,
  dataFile.close();                                                                     //close the file
  SPI.setDataMode(SPI_MODE2);                                                           //Set SPI Data Mode to 0 for sd card
}

void writeFilePosition(){
Serial.println(("**Writing File Position"));
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
Serial.print(F("  filePosition:"));
Serial.println(filePosition);  
  SPI.setDataMode(SPI_MODE0);                                                           //Set SPI Data Mode to 0 for sd card
  if(SD.exists("filepos.txt")){
    SD.remove("filepos.txt");
  }  
  File dataFile=SD.open("filepos.txt", FILE_WRITE);					//Initialize file variable
  dataFile.print(whichFile);                                                            //File Designator
  dataFile.print(F(","));                                                               //,
  dataFile.print(filePosition);                                                         //File Position
  dataFile.close();
  SPI.setDataMode(SPI_MODE2);                                                           //Set SPI Data Mode to 0 for sd card
}

void nextFile(){			                //function for selecting the next file based on the current file (when file does not open or end of file has been reached)
  newFile=true;
Serial.println(F("**Moving to next file"));
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
  if(whichFile=='s'){				        //send summary of data first.  if summary.txt is finished
    whichFile='p';					//move to icedat.txt
  }
  else if(whichFile=='p'){				//send prodat.txt.  if prodat.txt is finished
    whichFile='i';					//move to icedat.txt
  }
  else if(whichFile=='i'){			        //if icedat.txt has finished
    whichFile='b';					//move to botdat.txt
  }
  else if(whichFile=='b'){			        //if botdat.txt has finished, then all files are done
    whichFile='z';				        //'z' to designate sleep mode.  Send a message indicating data is done and sleep for 24 hours
  }
  else if(whichFile=='z'){			        //if done.txt has finished, then all files are done
    whichFile='x';				        //'x' for exit.  all files complete so stop sending data	
  }
  else{
    whichFile='x';	  				//failsafe if something strange happens, exit and stop sending data
  }
Serial.println(F("**Moved to next file"));
Serial.print(F("  whichFile:"));
Serial.println(whichFile);
}

void loop(){  				
  shutdownUnit();				//If program exits routine unexpectedly (all contained in 'setup'), then shut down						
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

void sendSPIdata(char pin, byte b1, byte b2){	//generic function for sending 2 bytes of SPI data
  digitalWrite(pin, LOW);			//Select designated chip for SPI Communication
  SPI.transfer(b1);				//Transfer first byte
  SPI.transfer(b2);				//Transfer second byte
  digitalWrite(pin, HIGH);			//De-Select designated chip for SPI Communication
}

void setAlarmNextSendAttempt(){
  SPI.setDataMode(SPI_MODE1);		                //Set SPI Data Mode to 1 for RTC
  DateTime now = RTC.now();                             //Get time from RTC and store into global variable for when unit awoke
  if(whichFile=='z'){                                   //if All the data has finished sending, go into sleep mode, set by designated sleep interval in seconds (stored in filePosition variable)
    if(filePosition==0){          
      filePosition=3600;                               //if a designated sleep interval is not set from previous SD write or received message, then default to +1 day
    }
    alarmtime=now.unixtime()+filePosition;              //Add NNNNNNNNNN seconds to the current time 
    Serial.print(F("Alarm Interval="));
    Serial.print(filePosition);
  }
  else{                                                           //if whichFile does not equal z, then data has not finished sending.  if we have reached this point in the code, then we have found GPS successfully so attempt to send iridium data every hour from this point onward
    //alarmtime=now.unixtime()+(unsigned long)300;                //Add 1 hour to the current time 
    alarmtime=now.unixtime()+(unsigned long)SEND_DATA_INTERVAL;        //Add 1 hour to the current time 
    Serial.print(F("Alarm Interval="));
    Serial.print(F("3600"));
  }
  if(whichFile!='x'){
    Serial.println(F("Alarm Set"));
    setAlarmTime();                                       //Set The Alarm 
  }
  else{
    Serial.println(F("Alarm NOT Set"));
  }
  
  SPI.setDataMode(SPI_MODE0);                           //Set SPI Data Mode to 2 for default
}

void setAlarmTime(){
  SPI.setDataMode(SPI_MODE1);                   //Set SPI Data Mode to 1 for RTC
  RTCWrite(0x87,alarmtime.second() & 0x7F);     //set alarm time: seconds     //87=write to location for alarm seconds    //binary & second with 0x7F required to turn alarm second "on"
  RTCWrite(0x88,alarmtime.minute() & 0x7F);     //set alarm time: minutes     //88=write to location for alarm minutes    //binary & minute with 0x7F required to turn alarm minute "on"
  RTCWrite(0x89,alarmtime.hour() & 0x7F);       //set alarm time: hour        //89=write to location for alarm hour       //binary & hour with 0x7F required to turn alarm hour "on"
  RTCWrite(0x8A,alarmtime.day() & 0x3F);        //set alarm time: day         //8A=write to location for alarm day        //binary & day with 0x3F required to turn alarm day "on" (not dayofWeek) 
  RTCWrite(0x8B,0);                             //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8C,0);                             //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8D,0);                             //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8F,B00000000);                     //reset flags                //8F=write to location for control/status flags    //B00000000=Ocillator Stop Flag 0, No Batt Backed 32 KHz Output, Keep Temp CONV Rate at 64 sec (may change later), disable 32 KHz output, temp Not Busy, alarm 2 not tripped, alarm 1 not tripped
  RTCWrite(0x8E,B00000101);                     //set control register       //8E=write to location for control register        //B01100101=Oscillator always on, SQW on, Convert Temp off, SQW freq@ 1Hz, Interrupt enabled, Alarm 2 off, Alarm 1 on
  SPI.setDataMode(SPI_MODE2);                   //Set SPI Data Mode to default
}

void RTCWrite(char reg, char val){
  digitalWrite(RTC_CS, LOW);                    //Select RTC for SPI Communication
  SPI.transfer(reg);                            //Send RTC register 
  SPI.transfer(bin2bcd(val));                   //Send value to RTC register in proper format
  digitalWrite(RTC_CS, HIGH);                   //De-Select RTC for SPI Communication
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

void shutdownUnit(){
  digitalWrite(SHUTDOWN,LOW);			//Pin Low=Power to entire unit off
  delay(100000);				//long delay to ensure nothing else happens and unit shuts down
}

bool ISBDCallback()				//This function executes in the background while doing certain Iridium functions, we don't need it to do anything here
{
  return true;					//Must return true to continue using ISBD protocol
}
