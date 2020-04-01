
//INPUT for POP-UP S/N
#define SN 2018                       //INPUT POP-UP S/N as SN = ####

//INPUT for Topside Temperature Probe (TTP)
float At = 0.00117611211;  //INPUT Steinhart-Hart Coeficient A for probes from unit of interest
float Bt = 0.000231799033;  //INPUT Steinhart-Hart Coeficient B for probes from unit of interest
float Ct = 0.000000106066762;  //INPUT Steinhart-Hart Coeficient C for probes from unit of interest

//INPUT for Sea Surface Temperature (SST) Probe
float As = 0.00105471332;  //INPUT Steinhart-Hart Coeficient A for probes from unit of interest
float Bs = 0.00024840849;   //INPUT Steinhart-Hart Coeficient B for probes from unit of interest
float Cs = 0.0000000603057951;  //INPUT Steinhart-Hart Coeficient C for probes from unit of interest


//include various libraries
#include <ctype.h>
#include <math.h>                      //Library for math functions used in calculating sensor values
#include <Wire.h>                      //Library for I2C Comms
#include <SPI.h>                       //Library for SPI Comms
#include <SD.h>                        //Library for SD Card
#include <RTClib.h>                    //Library for RTC Functions
#include <RTC_DS3234.h>                //Library for DS3234 RTC
#include <MemoryFree.h>                //Library to determine amount of free RAM while program is running
#include <TinyGPS++.h>                 //Library for interfacing with GPS module
#include <IridiumSBD.h>                //Library for Iridium interface - rockBlock Mk3

#define TEMP_SWITCH_1 3                //Define pin for switching between temp sensors and reference thermistors
#define TEMP_SWITCH_2 4                //Define pin for switching between temp sensors and reference thermistors
#define SENSOR_POWER_5V 6              //Define pin for switching on power to 5V sensors
#define SENSOR_POWER_3V3 32            //Define pin for switching on power to 3.3V sensors
#define GPS_POWER 9                    //Define pin for switching on GPS
#define IRIDIUM_POWER 8                //Define pin for switching on Iridium power
#define IRIDIUM_ON_OFF 11              //Define pin for switching on Iridium module
#define ACCEL_CS A8                    //Define pin for accelerometer SPI chip select
#define SDCARD_CS A5                   //Define pin for sdcard SPI chip select
#define RTC_CS A2                      //Define pin for RTC SPI chip select
#define SHUTDOWN A0                    //Define pin for shutting down unit
#define FLUOROMETER_X10 24             //Define pin for setting Fluorometer Sensitivity to X10
#define FLUOROMETER_X100 26            //Define pin for setting Fluorometer Sensitivity to X100
#define FLUOROMETER_ON_OFF 28          //Define pin for switching on Fluorometer
#define CAMERA_RESET 13                //Define pin for switching on Camera reset function
#define CAMERA_ON_OFF 12               //Define pin for switching on Camera module
#define SERIAL_COMMS_ON_OFF 2          //Define pin for switching on Serial Comms chip (5V to 3.3V translator)

#define kellerADDR 0x41       	       	        //Define pressure sensor control register
#define kellerCheckPressure 0xAC       	        //Define pressure sensor check pressure command
#define ads1100A0 B1001000              //Define temperature ADC control register
#define ads1100A1 B1001001                 	        //Define PAR ADC control register
#define ads1100A2 B1001010                 	        //Define Fluorometer ADC control register
#define RECEIVE_BUFFER_SIZE 15		   	          //Size of buffer to receive Iridium Messages
#define CHARS_PER_SBDMESSAGE 120       	        //Number of bytes to pull from data file and send at once   (nominal CHARS_PER_SBDMESSAGE 120 = 120 bytes)
#define BYTES_PER_SBDMESSAGE 338       	        //Number of bytes to pull from data file and send at once   (nominal BYTES_PER_SBDMESSAGE 338 =  338 bytes)
#define GPS_FIX_SEARCH_SECONDS 120              //Number of seconds to search for GPS fix                   (nominal GPS_FIX_SEARCH_SECONDS 120 = 2 minutes)
#define GPS_SATELLITE_SEARCH_SECONDS 10	        //Number of seconds to search for GPS satellite             (nominal GPS_SATELLITE_SEARCH_SECONDS 10 = 10 seconds)
#define SEND_DATA_INTERVAL 1200                 //Default interval to send data once GPS has been found     (nominal SEND_DATA_INTERVAL 1200 = 20 minutes)
#define BOTTOM_SAMPLE_INTERVAL 3600			    //Interval between bottom samples 			                	  (nominal BOTTOM_SAMPLE_INTERVAL 3600 = 1 hour)
#define PRE_RELEASE_WAKEUP_INTERVAL 7200		  //How long before planned release to wait for profile 	    (nominal PRE_RELEASE_WAKEUP_INTERVAL 7200 = 2 hours)
#define UNDER_ICE_SAMPLE_INTERVAL 3600			  //Interval between samples at the surface (or uncer ice)	  (nominal UNDER_ICE_SAMPLE_INTERVAL 3600	= 1 hour)
#define DRIFT_MODE_SAMPLE_INTERVAL 10800			//Interval between samples at the surface (or uncer ice)	  (nominal DRIFT_MODE_SAMPLE_INTERVAL 10800	= 3 hours)
#define PROFILE_LENGTH_SECONDS 90			      //Length of profile in seconds				                      (nominal PROFILE_LENGTH_SECONDS 90 = 90 seconds)
#define TIMEZONE 0                            //
#define DEPTH_CHANGE_TO_TRIP_PROFILE 2           //Depth in meters needed to begin profile mode            (nominal DEPTH_CHANGE_TO_TRIP_PROFILE 2 = 2 meters)

HardwareSerial &IridiumSerial = Serial1;        //Define Alias for Iridium Serial Comms (Arduino MEGA Serial1)
HardwareSerial &CameraSerial = Serial2;         //Define Alias for Iridium Serial Comms (Arduino MEGA Serial2)
HardwareSerial &GPSSerial = Serial3;            //Define Alias for Iridium Serial Comms (Arduino MEGA Serial3)

IridiumSBD isbd(IridiumSerial, IRIDIUM_ON_OFF);	//Object for Iridium Module
TinyGPSPlus gps;			                      	  //Object for GPS Module
RTC_DS3234 RTC(RTC_CS);			                	  //Object for RTC

DateTime wakeupTime;				                    //Global Variable for when unit first wakes
DateTime alarmtime;			                    	  //Global Variable for setting RTC Alarm
boolean foundGPS = false; 		                	 //Global Variable for GPS lock success
uint16_t fileSelection = 0;                     //Global Variable for marking which file to send data from
uint32_t filePosition = 0;		       		      	 //Global variable to designate position in file when sending data
boolean newFile = true;                     		 //Global variable for marking when a file has completed transmitting
boolean beginSDsuccess;                   		  //Global variable for informing user if SDCard initialized sucessfully

static const int MAX_SATELLITES = 73;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} 

sats[MAX_SATELLITES];

void setup() {
  Serial.begin(115200);                       //Turn the Serial Protocol ON
  initializePins();                           //Initialize Arduino Pins for Output and Set Voltage Levels
  turnOn3v3SensorPower();                     //turn on 3.3V Power to sensors (PAR, TEMP, PRESSURE)
  turnOn5vSensorPower();                      //turn on 5V Power to sensors
  delay(500);
  Wire.begin();				                        //Start I2C Communication
  SPI.begin();                                //Start up SPI Communication (needed for various ICs)
  SPI.setBitOrder(MSBFIRST);                	//Configure SPI Communication to Most Significant Bit First
  SPI.setClockDivider(SPI_CLOCK_DIV4);      	//Set SPI Clock Speed

  setupRTCandSD();			                      //Start and configure RTC and SD Card
  setupAccel();		                            //Start and configure Accelerometer

  setupTempADC();
  setupPARADC();			                        //Set Up the ADC for PAR Measurement
  setupFluorADC();			                      //Set Up the ADC for Fluorometer Measurement
  delay(500);				                          //Longer delay here to let voltages stabilize

  Serial.println(F("Unit set up Complete"));     //Print to user interface
  displayStatus();                            //Display summary of sensors and status
}

void loop() {                 //continue running loop to display options to user and check functions
  //"OPTIONS" Menu
  Serial.println(F("-----------------------------------------------"));     //Print to user interface
  Serial.println(F("'1' - display Status"));                                //Print to user interface
  Serial.println(F("'2' - set RTC time/date"));                             //Print to user interface
  Serial.println(F("'3' - check Temperature Sensors"));                     //Print to user interface
  Serial.println(F("'4' - check Pressure Sensor"));                         //Print to user interface
  Serial.println(F("'5' - check Tilt Sensor"));                             //Print to user interface
  Serial.println(F("'6' - check PAR Sensor"));                              //Print to user interface
  Serial.println(F("'7' - check Fluorometer"));                             //Print to user interface
  Serial.println(F("'S' - check GPS Satellites"));                          //Print to user interface
  Serial.println(F("'F' - check GPS Fix"));                                 //Print to user interface
  Serial.println(F("'I' - check Iridium"));                                 //Print to user interface
  Serial.println(F("'C' - check Camera"));                                  //Print to user interface
  Serial.println(F("'D' - display SD Card Data"));                          //Print to user interface
  Serial.println(F("'X' - delete all SD Card Data"));                       //Print to user interface
  Serial.println(F("'8' - conduct SST temp response test at 4Hz"));         //Print to user interface
  Serial.println(F("'9' - conduct TTP temp response test at 4Hz"));         //Print to user interface  
  Serial.println(F("-----------------------------------------------"));     //Print to user interface
  Serial.println();                                                         //Print to user interface

  DateTime now = RTC.now();             //Set system time to time from RTC
  byte byteRead;                        //variable to read Serial input from user
  boolean waitingforInput = true;       //variable to wait for user input

  while (waitingforInput == true) {     //Until the user inputs a proper command...
    byteRead = Serial.read();           //Read any input into the Serial Monitor
    switch (byteRead) {                 //What did the user input? Check ASCII values
      case  49 :                        //if user enters '1', display unit Status
        displayStatus();                //Display summary of sensors and status
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  50 :                        //if user enters '2', display dialog to have user set RTC
        setRTC();                       //display dialog to have user set RTC
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  51 :                        //if user enters '3', check Temperature
      //break intentionally omitted, go to displaySensor() function
      case  52 :                        //if user enters '4', check Pressure
      //break intentionally omitted, go to displaySensor() function
      case  53 :                        //if user enters '5', check Accelerometer
      //break intentionally omitted, go to displaySensor() function
      case  54 :                        //if user enters '6', check PAR
      //break intentionally omitted, go to displaySensor() function
      case  56 :                        //if user enters '8', test SST response time
      //break intentionally omitted, go to displaySensor() function
      case  57 :                        //if user enters '9', test TTP response time
      //break intentionally omitted, go to displaySensor() function
      case  55 :                        //if user enters '7', check Fluorometer
        displaySensor(byteRead);        //display measurements from selected sensor
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  83 :                        //if user enters 'S', check GPS
        lookforGPSSatellites();         //function to check GPS functionality
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  70 :                        //if user enters 'F', check GPS
        lookforGPSFix();                //function to check GPS functionality
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  73 :                        //if user enters 'I', check Iridium
        checkIridium();                 //function to check Iridium functionality
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  67 :                        //if user enters 'C', check Camera
        checkCamera();                  //function to check Camera functionality
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  68 :                        //if user enters 'D', check SD Card Data
        sdDataCheck();                  //function to check SDCard functionality
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
      case  88 :                        //if user enters 'X', delete files from SD Card (check with user first)
        boolean fileDelete = deleteFilesPrompt();
        if (fileDelete == true) {      //If user confirms that they want to delete files
          deleteFiles();                //delete files from SDCard
        }
        Serial.println();
        waitingforInput = false;        //Stop reading user Input
        break;                          //exit switch case
    }
  }
  delay(1000);                          //wait 1 second before running the loop again
}

void initializePins() {
  pinMode(SHUTDOWN, OUTPUT);		         //SHUTDOWN PIN for turning unit off (Sleep mode)
  digitalWrite(SHUTDOWN, HIGH);			     //SHUTDOWN PIN HIGH keeps unit's power on
  pinMode(TEMP_SWITCH_1, OUTPUT);			   //TEMP_SWITCH PIN for switching between thermistors and reference resistor
  digitalWrite(TEMP_SWITCH_1, LOW);		   //TEMP_SWITCH PIN HIGH for reading reference resistor
  pinMode(TEMP_SWITCH_2, OUTPUT);			   //TEMP_SWITCH PIN for switching between thermistors and reference resistor
  digitalWrite(TEMP_SWITCH_2, LOW);		   //TEMP_SWITCH PIN HIGH (Doesn't matter when reference resistor is selected)
  pinMode(SENSOR_POWER_5V, OUTPUT);		   //SENSOR_POWER_5V PIN for turning 5V sensor power on and off
  digitalWrite(SENSOR_POWER_5V, HIGH);	   //SENSOR_POWER_5V PIN HIGH for 5V sensor power off (P-Channel Transistor)
  pinMode(SENSOR_POWER_3V3, OUTPUT);		   //SENSOR_POWER_3V3 PIN for turning 3.3V power on and off
  digitalWrite(SENSOR_POWER_3V3, HIGH);	 //SENSOR_POWER_3V3 PIN HIGH for 3.3V sensor power off (P-Channel Transistor)
  pinMode(GPS_POWER, OUTPUT);			       //GPS_POWER PIN for turning power to GPS on and off
  digitalWrite(GPS_POWER, HIGH);			     //GPS_POWER PIN HIGH for GPS power off (P-Channel Transistor)
  pinMode(IRIDIUM_POWER, OUTPUT);		     //IRIDIUM_POWER PIN for turning power to Iridium on and off
  digitalWrite(IRIDIUM_POWER, HIGH);		   //IRIDIUM_POWER PIN HIGH for Iridium power off (P-Channel Transistor)
  pinMode(IRIDIUM_ON_OFF, OUTPUT);		     //IRIDIUM_ON_OFF PIN for Iridium sleep or awake
  digitalWrite(IRIDIUM_ON_OFF, LOW);		   //IRIDIUM_ON_OFF PIN LOW for Iridium sleep mode
  pinMode(ACCEL_CS, OUTPUT);				       //ACCEL_CS PIN for SPI communication to Accelerometer
  digitalWrite(ACCEL_CS, HIGH);			     //ACCEL_CS PIN HIGH for SPI communication to Accelerometer inactive
  pinMode(SDCARD_CS, OUTPUT);			       //SDCARD_CS PIN for SPI communication to SDCard
  digitalWrite(SDCARD_CS, HIGH);    		   //ACCEL_CS PIN HIGH for SPI communication to SDCard inactive
  pinMode(RTC_CS, OUTPUT);                //RTC_CS PIN for SPI communication to RTC
  digitalWrite(RTC_CS, HIGH);             //ACCEL_CS PIN HIGH for SPI communication to RTC inactive
  pinMode(FLUOROMETER_X10, OUTPUT);				//FLUOROMETER_X10 PIN for Fluorometer sensitivity x10 enable and disable
  digitalWrite(FLUOROMETER_X10, LOW);			//FLUOROMETER_X10 PIN LOW for Fluorometer sensitivity x0
  pinMode(FLUOROMETER_X100, OUTPUT);				//FLUOROMETER_X100 PIN for Fluorometer sensitivity x10 enable and disable
  digitalWrite(FLUOROMETER_X100, LOW);			//FLUOROMETER_X100 PIN LOW for Fluorometer sensitivity x0
  pinMode(FLUOROMETER_ON_OFF, OUTPUT);			//FLUOROMETER_ON_OFF PIN for Fluorometer on and off
  digitalWrite(FLUOROMETER_ON_OFF, HIGH);	//FLUOROMETER_X10 PIN HIGH for Fluorometer power off (P-Channel Transistor)
  pinMode(CAMERA_RESET, OUTPUT);				   //CAMERA_RESET PIN for camera hardware reset
  digitalWrite(CAMERA_RESET, LOW);			   //CAMERA_RESET PIN for LOW camera hardware reset inactive
  pinMode(CAMERA_ON_OFF, OUTPUT);				 //CAMERA_ON_OFF PIN for camera on and off
  digitalWrite(CAMERA_ON_OFF, HIGH);			 //CAMERA_ON_OFF PIN HIGH for Fluorometer power off (P-Channel Transistor)
  pinMode(SERIAL_COMMS_ON_OFF, OUTPUT);   //SERIAL_COMMS_ON_OFF PIN for SPI communication to SDCard
  digitalWrite(SERIAL_COMMS_ON_OFF, LOW); //SERIAL_COMMS_ON_OFF PIN HIGH for Serial Comms off
}

void setupRTCandSD() {
  SPI.setDataMode(SPI_MODE1);		        //Set SPI Data Mode to 1 for RTC
  RTC.begin();                          //Start RTC (defined by RTC instance in global declarations)
  wakeupTime = RTC.now();               //Get time from RTC and store into global variable for when unit awoke
  SPI.setDataMode(SPI_MODE0);           //Set SPI Data Mode to 0 for SDCard
  beginSDsuccess = SD.begin(SDCARD_CS); //Start SD Card so it is available for other function calls
}

void displayStatus() {
  Serial.println(F("Unit Status:"));
  if (beginSDsuccess) {                                                                               //if SDCard Starts successfully
    Serial.println(F("  SD Card Initialized Successfully."));
  }
  else {                                                                                              //if SDCard does not start successfully
    Serial.println(F("  **SD Card error.  Check if SD Card is present and re-start program. If SD Card is present conduct loopback test to reset Arduino.**"));
  }
  displayRTCTime();                     //Display RTC Time on Serial Monitor
  displayTemperatures();                //Display Temperature Sensor measurement and Reference Resistor Value on Serial Monitor
  displayPAR();                         //Display PAR Sensor measurement on Serial Monitor
  turnOnFluorometer();                  //Turn on Fluorometer
  displayFluor();                       //Display Fluorometer measurement on Serial Monitor
  turnOffFluorometer();                 //Turn Off Fluorometer
  displayPressure();                    //Display Pressure Sensor measurement on Serial Monitor
  displayAccelerometer();               //Display Accelerometer measurement (Tilt Angle) on Serial Monitor
  Serial.println();
}

void displayRTCTime() {                           //display RTC Time on Serial Monitor
  SPI.setDataMode(SPI_MODE1);                     //Set SPI Data Mode to 1 for RTC
  const int len = 32;                             //buffer length for printing RTC Time
  static char buf[len];                           //buffer for printing RTC Time
  DateTime now = RTC.now();                       //Set system time to time from RTC
  Serial.print(F("  Current RTC Time/Date: "));
  Serial.println(now.toString(buf, len));         //Print formatted date/time to serial monitor
}

void displaySensor(char menuInput) {                  //function to display measurement from sensors once every 1/4 second, menuInput defines which sensor to display
  if (menuInput == 55) {                              //if user entered '7'
    Serial.println(F("Turning on Fluorometer..."));
    turnOnFluorometer();                              //Turn on the fluorometer
    delay(2000);                                      //wait 2 seconds for the sensor to startup on
  }
  Serial.println(F("Checking Sensor..."));
  Serial.println(F("Enter 'Q' to stop\n"));
  boolean waitingforInput2 = true;                    //variable to wait for user input
  byte byteRead = 0;                                  //variable to read user input from Serial Monitor
  while (waitingforInput2 == true) {                  //until the user Enters 'Q' to stop...
    
    if (Serial.available()) {                         //if anything has been input into the Serial Monitor
      byteRead = Serial.read();                       //read serial monitor input
    }
    
    if (byteRead == 81) {                             //if user Enters 'Q'
      waitingforInput2 = false;                       //stop waiting for user input and exit this dialog
    }
    
    else if (menuInput == 51) {                            //If user entered '3'
      displayTemperatures();                          //display temperature measurements and reference resistor reading to serial monitor
      Serial.println();
    }
    
    else if (menuInput == 52) {                       //If user entered '4'
      displayPressure();                              //display pressure measurement on serial monitor
      Serial.println();
    }
    
    else if (menuInput == 53) {                       //If user entered '5'
      displayAccelerometer();                         //display accelerometer (tilt sensor) measurement on serial monitor
    }
    
    else if (menuInput == 54) {                       //If user entered '6'
      displayPAR();                                   //display PAR measurement on serial monitor
    }
    
    else if (menuInput == 55) {                       //If user entered '7'
      displayFluor();                                 //display fluorometer measurement on serial monitor
    }

    else if (menuInput == 56) {                       //If user entered '8'
      TestSSTResponse();                             //display SST temp response test on serial monitor
    }
    
    else if (menuInput == 57) {                       //If user entered '9'
      TestTTPResponse();                             //display TTP temp response test on serial monitor
    }
    
    else {}
    delay(250);                                      //take 4 sensor readings per second, once every 250 milliseconds
  }
  
  if (menuInput == 55) {                              //If user entered '7'
    Serial.println(F("Turning off Fluorometer..."));
    turnOffFluorometer();                             //Turn off power to fluorometer
    delay(50);                                        //short delay to turn off sensor
  }
}

void displayAccelerometer() {                         //function to display accelerometer reading (tilt angle) to serial monitor
  uint8_t tiltAngle = readAccel();                    //read the accelerometer
  Serial.print(F("  Tilt Angle: "));
  Serial.print(tiltAngle);                            //display the tilt Angle
  Serial.println(F(" degrees from horizontal"));
}

void displayFluor() {                                 //function to display fluorometer reading to serial monitor
  int fluorVal = readFluor();                     //read the fluorometer ADC
  Serial.print(F("  Fluorometer ADC Val: "));
  Serial.print(fluorVal);                           //display the ADC reading
  float fluorCalc = 0;                                //calculate estimated fluorometer value
  Serial.print(F(" (approx "));
  //Serial.print(fluorCalc);                          //display the estimated fluorometer value
  Serial.println(F("? ug/L)"));
}

void displayPAR() {
  int parVal = readPAR();
  Serial.print(F("  PAR  Sensor ADC Val: "));
  Serial.print(parVal);
  float parCalc = (parVal - 5460) * 0.065753;
  Serial.print(F(" (approx "));
  Serial.print(parCalc);
  Serial.println(F(" umol/(m2s))"));
}

void displayTemperatures() {                                                                                //function to display temperature reading to serial monitor
  int tempValTop = readTopTemp();                                                                           //read voltage from top probe thermistor
  Serial.print(F("  Temp Sensor ADC Val: "));
  Serial.print(tempValTop);
  Serial.print(F(" (approx "));
  float tempCalc = (1 / (At + Bt * log(tempValTop) + Ct * pow(log(tempValTop), 3))) - 273.15;               //calculate estimated temperature from Steinhart-Hart equation
  Serial.print(tempCalc, 2);
  Serial.println(F(" C)"));

  int tempValSST = readSST();                                                                               //read voltage from sst probe thermistor
  Serial.print(F("  SST (Sea Surface Temp) Sensor ADC Val: "));
  Serial.print(tempValSST);
  Serial.print(F(" (approx "));
  float SSTCalc = (1 / (As + Bs* log(tempValSST) + Cs * pow(log(tempValSST), 3))) - 273.15;                  //calculate estimated temperature from Steinhart-Hart equation
  Serial.print(SSTCalc, 2);
  Serial.println(F(" C)"));

  int tempRefVal = readTempRef();                                                                            //read voltage from reference thermistor
  Serial.print(F("  Temp Ref    ADC Val: "));
  Serial.print(tempRefVal);
  float tempRefCalc = (tempRefVal / 16384.0 * 3.0 / 4.0) * 39.872;                                            //calculate estimated resistance based on voltage output curve from LTSpice model
  Serial.print(F(" (approx "));
  Serial.print(tempRefCalc, 2);
  Serial.println(F(" kOhms)"));
}

void TestSSTResponse() {
  int tempValSST = testSST();
  float SSTCalc = (1 / (As + Bs * log(tempValSST) + Cs * pow(log(tempValSST), 3))) - 273.15;                  //calculate calibrated temperature from Steinhart-Hart equation
  Serial.print(SSTCalc, 2);                                                                                   //the '2' after SSTCalc, '2' designates taking temp measurement to the nearest 100th of a degree
  Serial.println(F(" C"));
}

void TestTTPResponse() {
  int tempValTop = testTopTemp();
  float TTPCalc = (1 / (As + Bs * log(tempValTop) + Cs * pow(log(tempValTop), 3))) - 273.15;                  //calculate calibrated temperature from Steinhart-Hart equation
  Serial.print(TTPCalc, 2);                                                                                   //the '2' after TTPCalc, '2' designates taking temp measurement to the nearest 100th of a degree
  Serial.println(F(" C"));
}

void displayPressure() {                                    //function to display pressure reading to serial monitor
  uint16_t pressureVal100 = readPressure();                 //read the pressure sensor
  Serial.print(F("  Pressure Sensor Val: "));
  Serial.print(pressureVal100);                             //display the sensor reading
  float pressureCalc100 = (pressureVal100 - 16384.0) / 327.68; //calculate estimated pressure
  Serial.print(F(" (approx "));
  Serial.print(pressureCalc100);                            //display the estimated pressure
  Serial.println(F(" m)"));
}

void lookforGPSSatellites() {
  Serial.println(F("Turning on GPS Module..."));
  turnOnGPSPower();                                                      //turn on power to GPS Module
  turnOnSerialComms();                                                  //turn on Serial Comms Chip
  for (int i = 0; i < 4; ++i) {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }
  delay(100);                                                           //short delay to let GPS Module initialize and begin streaming data
  GPSSerial.begin(9600);                                                //begin serial comms at 9600 baud with GPS Module
  boolean waitingforInput2 = true;                                      //variable to wait for user input
  byte byteRead = 0;                                                    //variable to read input from Serial Monitor
  Serial.println(F("\nStreaming NMEA Data, Enter 'Q' to stop\n"));
  Serial.println(F("Looking for GPS Satellites:\n"));
  uint32_t timerStart = millis();                                       //timestamp for when start of GPS communication was attempted
  while (waitingforInput2 == true) {                                    //until user enters 'Q' to exit the dialog...
    while (GPSSerial.available() > 0) {                                 //if any data is available on the GPS Serial Buffer
      gps.encode(GPSSerial.read());
      if (totalGPGSVMessages.isUpdated()) {
        for (int i = 0; i < 4; ++i) {
          int no = atoi(satNumber[i].value());
          Serial.println(no);

          if (no >= 1 && no <= MAX_SATELLITES) {
            sats[no - 1].elevation = atoi(elevation[i].value());
            sats[no - 1].azimuth = atoi(azimuth[i].value());
            sats[no - 1].snr = atoi(snr[i].value());
            sats[no - 1].active = true;
            Serial.println(F("Active"));

          }
        }
        int totalMessages = atoi(totalGPGSVMessages.value());
        int currentMessage = atoi(messageNumber.value());
        uint8_t satsInView =0;
        uint32_t searchSeconds=0;
        if (totalMessages == currentMessage) {
          for (int i = 0; i < MAX_SATELLITES; ++i) {
            if (sats[i].active) {
              satsInView++;
            }
          }
          Serial.print(F("Satellites in view:"));
          Serial.print(satsInView);
          Serial.print(F(", Number of seconds searching:"));
          searchSeconds=(millis()-timerStart)/1000;
          Serial.println(searchSeconds);
          
          for (int i = 0; i < MAX_SATELLITES; ++i)
            sats[i].active = false;
        }
      }
    }
    if (Serial.available()) {                                         //if any data has been input on the Serial Monitor
      byteRead = Serial.read();                                     //read data from Serial Monitor
    }
    if (byteRead == 81) {                                             //if user enters 'Q'
      waitingforInput2 = false;                                     //stop waiting for user input and exit the dialog
    }
  }
  turnOffGPSPower();                                                    //turn off power to GPS Module
  turnOffSerialComms();                                                 //turn off Serial Comms Chip
  GPSSerial.end();                                                      //end Serial comms with GPS Module
}


void lookforGPSFix() {
  Serial.println(F("Turning on GPS Module..."));
  turnOnGPSPower();										                                  //turn on power to GPS Module
  turnOnSerialComms();                                                  //turn on Serial Comms Chip
  delay(100);											                                      //short delay to let GPS Module initialize and begin streaming data
  GPSSerial.begin(9600);									                              //begin serial comms at 9600 baud with GPS Module
  boolean waitingforInput2 = true;                                      //variable to wait for user input
  byte byteRead = 0;                                                    //variable to read input from Serial Monitor
  Serial.println(F("\nStreaming NMEA Data, Enter 'Q' to stop\n"));
  uint32_t timerStart = millis();                                       //timestamp for when start of GPS communication was attempted
  while (waitingforInput2 == true) {                                    //until user enters 'Q' to exit the dialog...
    while (GPSSerial.available() > 0)                                   //if any data is available on the GPS Serial Buffer
      if (gps.encode(GPSSerial.read()))                                 //translate available GPS Serial data into NMEA data
        displayGPSInfo();                                               //display any GPS data which has been translated successfully on the serial monitor
    if ((millis() - timerStart > 5000) && gps.charsProcessed() < 10) { //if comms have been attempted for 5 seconds, and no data has come in...
      Serial.println(F("No NMEA data detected: check power supply and connections."));
      waitingforInput2 = false;                                       //stop waiting for user input and exit the dialog
    }
    if (Serial.available()) {                                         //if any data has been input on the Serial Monitor
      byteRead = Serial.read();                                     //read data from Serial Monitor
    }
    if (byteRead == 81) {                                             //if user enters 'Q'
      waitingforInput2 = false;                                     //stop waiting for user input and exit the dialog
    }
  }
  turnOffGPSPower();										                                //turn off power to GPS Module
  turnOffSerialComms();                                                 //turn off Serial Comms Chip
  GPSSerial.end();										                                  //end Serial comms with GPS Module
}

void displayGPSInfo()                                             //function to display incoming GPS data to serial monitor
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())  {                                  //If location data from GPS is valid, display latitude and longitude to 6 decimal places on serial monitor
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else  {
    Serial.print(F("INVALID"));                                   //If location data is not valid, display INVALID
  }

  Serial.print(F("  Date/Time: "));                               //If date from GPS is valid, display  it on serial monitor
  if (gps.date.isValid())  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else  {
    Serial.print(F("INVALID"));                                   //If date is not valid, display INVALID
  }

  Serial.print(F(" "));
  if (gps.time.isValid())  {                                      //If Time from GPS is valid, display it on serial monitor
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
  else  {
    Serial.print(F("INVALID"));                                   //If time is not valid, display INVALID
  }
  Serial.print(F("  Satellites in Use:"));                               //display the number of visible satellites, regardless of validity of other data
  Serial.print(gps.satellites.value());
  Serial.println();
}

void checkCamera() {                                                                                  //function to check camera functionality by taking a picture
  Serial.println(F("Turning on Camera Power..."));
  turnOnCameraPower();                                                                                //turn on power to camera
  turnOnSerialComms();                                                                                //turn on serial comms chip
  delay(800);                                                                                         //wait 800 ms for camera startup (per datasheet)
  digitalWrite(CAMERA_RESET, LOW);
  delay(50);
  digitalWrite(CAMERA_RESET, HIGH);
  delay(800);                                                                                         //wait 800 ms for camera startup (per datasheet)
  CameraSerial.begin(9600);                                                                           //start serial comms with camera at 9600 baud
  if (syncCamera()) {                                                                                 //if camera sync is successful
    Serial.println(F("Camera Sync successful!\nWaiting 2 seconds for circuits to stabilize..."));
    delay(2000);                                                                                      //wait 2000 ms to let camera circuits stabilize (per datasheet)
    if (setupCamera()) {                                                                              //if camera setup is successful
      if (takeCameraSnapshot()) {                                                                     //if commands to take camera snapshot is successful
        delay(200);                                                                                   //wait 200 ms for picture to be available
        if (getImageFromCamera()) {                                                                   //if all image data can be downloaded successfully
          Serial.println(F("Image Capture Successful!"));
          Serial.println(F("Image Saved on SDcard as camtest.jpg"));
          Serial.println(F("Transfer File to PC and open to check image"));                         //let user know how to verify image
        }
        else {
          Serial.println(F("FAILED to Get/Save Image"));
        }
      }
      else {
        Serial.println(F("Take Snapshot FAILED"));
      }
    }
    else {
      Serial.println(F("Setup FAILED"));
    }
  }
  else {
    Serial.println(F("Sync FAILED"));
  }
  turnOffCameraPower();                                                                               //turn off power to the camera
  CameraSerial.end();                                                                                 //end serial comms with camera
  turnOffSerialComms();                                                                               //turn off power to serial comms chip
  Serial.println();
}

boolean syncCamera() {                                                //function to sync serial comms with camera
  Serial.println(F("Starting Camera Sync..."));
  uint8_t sync_attempts = 1;                                          //variable for number of sync commands sent
  boolean syncStatus = false;                                         //variable for camera sync status
  uint8_t sync_delay_ms = 5;                                          //variable for milliseconds to delay between sync commands
  uint8_t SYNC_COMMAND[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};     //6 byte command to sync camera
  uint8_t SYNC_RESPONSE[6] = {0xAA, 0x0E, 0x0D, 0x00, 0x00, 0x00};    //6 byte sync response
  while (sync_attempts <= 60 && syncStatus == false) {                //while camera has not synced and we have sent less than 60 sync commands (per datasheet)
    sendCameraCommand(SYNC_COMMAND);                                  //send sync command over serial comms
    delay(sync_delay_ms);                                             //delay defined number of ms (per datasheet)
    syncStatus = readack(SYNC_RESPONSE);                              //listen for a responce, return true if it is the expected sync response
    sync_attempts++;                                                  //increment sync attempt
    sync_delay_ms = 5 + sync_attempts;                                //define number of ms to wait for next attempt (per datasheet)
  }
  if (syncStatus == true) {                                           //if camera has synced properly
    syncStatus = readack(SYNC_COMMAND);                               //listen for the next 6 byte response from the camera
    uint8_t SYNC_ACK_COMMAND[6] = {0xAA, 0x0E, 0x0D, 0x00, 0x00, 0x00};
    sendCameraCommand(SYNC_ACK_COMMAND);                              //send the final sync ack command
  }
  return syncStatus;                                                  //return true if sync was successful
}

boolean waitForCameraSerialData(uint16_t bytesNeeded) {               //function to wait for incoming serial data from camera (at 9600 baud this can take some time)
  uint32_t cameraSerialTimeout = millis() + 3000UL;                //set timeout for serial data to 3 seconds
  uint16_t bytesAvailable = 0;
  while (millis() < cameraSerialTimeout) {                            //wait for serial data until we have the proper number of bytes
    bytesAvailable = CameraSerial.available();
    //Serial.print(bytesAvailable);
    //Serial.print(F(" out of "));
    //Serial.println(bytesNeeded);
    if (bytesAvailable >= bytesNeeded) {                             //if not enough data is received before the timeout
      return true;                                                   //return false
    }
    //delay(10);
  }
  return false;                                                        //if we have enough bytes and serial comms did not timeout, then return true
}

boolean setupCamera() {                                                     //function to setup camera
  boolean setupStatus = false;                                              //variable for setup status
  uint8_t INITIAL_COMMAND[6] = {0xAA, 0x01, 0x00, 0x07, 0x00, 0x07};        //6 byte command to configure camera settings (resolution + format)
  sendCameraCommand(INITIAL_COMMAND);                                       //send the 6 byte command
  setupStatus = waitForCameraSerialData(6);                                 //wait for a response of 6 bytes
  uint8_t ACK_RESPONSE_INITIAL[6] = {0xAA, 0x0E, 0x01, 0x00, 0x00, 0x00};   //6 byte response expected for command
  if (setupStatus) {                                                        //if we have 6 bytes of data to read
    setupStatus = readack(ACK_RESPONSE_INITIAL);                            //read the response from the camera
  }
  if (setupStatus) {                                                        //if no errors yet
    uint8_t PACKAGESIZE_COMMAND[6] = {0xAA, 0x06, 0x08, 0x00, 0x02, 0x00};  //6 byte command to configure package size for image data
    sendCameraCommand(PACKAGESIZE_COMMAND);                                 //send the 6 byte command
    setupStatus = waitForCameraSerialData(6);                               //wait for a response of 6 bytes
    uint8_t ACK_RESPONSE_PACKAGESIZE[6] = {0xAA, 0x0E, 0x06, 0x00, 0x00, 0x00}; //6 byte response expected for command
    setupStatus = readack(ACK_RESPONSE_PACKAGESIZE);                        //read the response from the camera
  }
  return setupStatus;                                                       //if setup commands were sent and acked successfully, return true
}

boolean takeCameraSnapshot() {                                              //function to send command to camera to take a snapshot (can be retreived from memory multiple times)
  Serial.println(F("Taking Snapshot..."));
  boolean takeSnapshotStatus = false;                                       //variable for snapshot status
  uint8_t SNAPSHOT_COMMAND[6] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00};       //6 byte command to take a snapshot
  sendCameraCommand(SNAPSHOT_COMMAND);                                      //send the 6 byte command
  takeSnapshotStatus = waitForCameraSerialData(6);                          //wait for a response of 6 bytes
  uint8_t ACK_RESPONSE_SNAPSHOT[6] = {0xAA, 0x0E, 0x05, 0x00, 0x00, 0x00};  //6 byte response expected for command
  if (takeSnapshotStatus) {                                                 //if we have 6 bytes of data to read
    takeSnapshotStatus = readack(ACK_RESPONSE_SNAPSHOT);                    //read the response from the camera
  }
  return takeSnapshotStatus;                                                //if snapshot command was sent and acked successfully, return true
}

boolean getImageFromCamera() {                                              //function to get image from the camera
  boolean getImageStatus = false;                                           //variable for get Image status
  uint8_t GET_PICTURE_COMMAND[6] = {0xAA, 0x04, 0x01, 0x00, 0x00, 0x00};    //6 byte command to get image
  sendCameraCommand(GET_PICTURE_COMMAND);                                   //send the 6 byte command
  getImageStatus = waitForCameraSerialData(6);                              //wait for a response of 6 bytes
  uint8_t ACK_RESPONSE_GET_PICTURE[6] = {0xAA, 0x0E, 0x04, 0x00, 0x00, 0x00}; //6 byte response expected for command
  if (getImageStatus) {                                                     //if we have 6 bytes of data to read
    getImageStatus = readack(ACK_RESPONSE_GET_PICTURE);                     //read the response from the camera
  }
  if (getImageStatus) {                                                     //if no errors yet
    uint32_t imageSize = readImageSize();                                   //get the size of the image from the next message
    Serial.print(F("IMAGE SIZE="));
    Serial.print(imageSize);                                                //print the image size to the serial monitor
    Serial.println(F(" bytes"));
    getImageStatus = getImageData(imageSize);                               //get actual image data from the camera
  }
  return getImageStatus;
}

boolean getImageData(uint32_t imageSize) {                                  //function to get image data from the camera
  boolean getImageDataStatus = true;                                        //variable for if getting image data if successful
  uint8_t numPackages = imageSize / 506;                                    //calculate the number of packages to read from the camera
  SPI.setDataMode(SPI_MODE0);		                                            //Set SPI Data Mode to 0 for SD Card
  if (SD.exists("camtest.jpg")) {                                           //if camtest.jpg exists
    SD.remove("camtest.jpg");                                               //delete the existing file
  }
  for (uint8_t packageNum = 0; packageNum <= numPackages; packageNum++) {   //for the required number of packages
    if (getImageDataStatus) {                                               //if there are no errors
      getImageDataStatus = readPackage(packageNum);                         //read the next package, store it to the SDCard, and output if it was successful
    }
  }

  if (getImageDataStatus) {                                                 //if there were no errors
    uint8_t ACK_COMMAND[6] = {0xAA, 0x0E, 0x00, 0x00, 0xF0, 0xF0};          //6 byte command to ack all data was received
    sendCameraCommand(ACK_COMMAND);                                         //send the 6 byte command
  }
  return getImageDataStatus;                                                //if all data was collected successfully, return true
}

boolean readPackage(uint8_t packageNum) {                                   //function to read the next package of data from the camera
  boolean packageStatus = true;                                             //variable to tell if reading package was successful
  uint8_t readbyte;                                                         //variable to read incoming bytes
  uint16_t package_ID;                                                      //variable for incoming packageID
  uint8_t ACK_COMMAND[6] = {0xAA, 0x0E, 0x00, 0x00, packageNum, 0x00};      //6 byte command to get the next package
  sendCameraCommand(ACK_COMMAND);                                           //send the command
  
  packageStatus = waitForCameraSerialData(1);                               //wait for 1 bytes
  if (packageStatus) {                                                      //if we have 1 byte of data
    readbyte = CameraSerial.read();
    package_ID = readbyte;
  }
  if (packageStatus) {                                                      //if no errors so far  
    packageStatus = waitForCameraSerialData(1);                               //wait for 1 bytes
  }
  if (packageStatus) {                                                      //if we have 1 bytes of data
    readbyte = CameraSerial.read();
    package_ID = package_ID | readbyte << 8;                                //read the first 2 bytes and store data into Package ID variable
  }

  uint16_t datasize;                                                        //variable for size of incoming data
  if (packageStatus) {                                                      //if no errors so far
    packageStatus = waitForCameraSerialData(1);                               //wait for 1 bytes
  }
  if (packageStatus) {                                                      //if no errors so far and we have 2 bytes of data
    readbyte = CameraSerial.read();
    datasize = readbyte;
  }
  if (packageStatus) {                                                      //if no errors so far
    packageStatus = waitForCameraSerialData(1);                               //wait for 1 bytes
  }
  if (packageStatus) {                                                      //if we have 1 bytes of data
    readbyte = CameraSerial.read();
    datasize = datasize | readbyte << 8;                                    //read the next 2 bytes and store data into datasize variable
  }

  uint16_t bytesRead = 0;                                                   //variable for the number of bytes read
  uint16_t bytesAvailable = 0;                                                   //variable for the number of bytes read

  if (packageStatus) {                                                      //if no errors so far
    SPI.setDataMode(SPI_MODE0);		                                          //Set SPI Data Mode to 0 for SD Card
    File imagefile = SD.open("camtest.jpg", FILE_WRITE);                    //open camtest.jpg
    if (imagefile) {                                                        //if file is open
      uint32_t cameraSerialTimeout = millis() + 5000UL;                     //set timeout to 5 seconds from now
      while (bytesRead < datasize) {                                        //as long as we are still expecting more bytes to come in
        if (millis() > cameraSerialTimeout) {                               //if timeout has elapsed, return false and exit
          imagefile.close();                                                //close the file
          return false;
        }
        bytesAvailable = CameraSerial.available();
        if (bytesAvailable) {                                     //if any data is available on camera serial buffer
          readbyte = CameraSerial.read();                                   //read the next byte of data
          imagefile.write(readbyte);                                        //store the byte to the SD Card
          bytesRead++;
        }
      }
      imagefile.close();                                                    //close the file
    }
    else {
      Serial.println(F("Could not open file"));              //if file couldn't be opened
      packageStatus = false;                                                //return false
    }
  }

  uint16_t verifycode;                                                      //variable for checksum
  if (packageStatus) {                                                      //if no errors so far
    packageStatus = waitForCameraSerialData(2);                             //wait for 2 last bytes of data
  }
  if (packageStatus) {                                                      //if no errors so far and we have 2 more bytes of data
    readbyte = CameraSerial.read();
    verifycode = readbyte;
    readbyte = CameraSerial.read();
    verifycode = verifycode | readbyte << 8;                                //read the last 2 bytes and put them in the checksum variable
  }
  return packageStatus;                                                     //if there were no errors in getting image data packages, return true
}

boolean readack(uint8_t response[6]) {                                      //function to read and verify 6 bytes of data from camera
  uint8_t readbyte;                                                         //variable to read bytes of data
  boolean acked = false;                                                    //variable for if ack has been read successfully
  delay(50);
  for (uint8_t i = 0; i < 6; i++) {                                         //for each of the 6 bytes of data
    readbyte = CameraSerial.read();                                         //read the next byte coming from the camera
    if (readbyte == response[i] || response[i] == 0x00) {                   //if the byte from the camera matches the reponse we expected
      acked = true;                                                         //byte has been read successfully
    }
    else {                                                                  //if any byte is incorrect
      acked = false;                                                        //ack response was not read successfully
      return acked;                                                         //return false and exit
    }
  }
  return acked;                                                             //return status of reading ack response
}

uint32_t readImageSize() {                          //function to read the size of the image from camera response
  uint8_t readbuf[6];                               //buffer to read 6 bytes of data
  uint32_t imageSize = 0;                           //variable for image size
  uint32_t sizeByte = 0;                            //variable for calculating image size with individual bytes
  if (waitForCameraSerialData(6)) {                 //if 6 bytes of data are available from the camera
    for (uint8_t k = 0; k < 6; k++) {               //for each of the 6 bytes
      readbuf[k] = CameraSerial.read();             //read each byte from the camera
    }
    sizeByte = readbuf[5];                          //these 5 lines calculate the image size using last 3 bytes of data in buffer
    imageSize = sizeByte << 16;
    sizeByte = readbuf[4];
    imageSize = imageSize + sizeByte << 8;
    sizeByte = readbuf[3];
  }
  return imageSize;                                 //return the calculated size of the image
}

void sendCameraCommand(uint8_t command[6]) {        //generic function to send a 6 byte command to the camers
  while (CameraSerial.available()) {                //if any data is available in the Camera Serial buffer
    CameraSerial.read();                            //read and flush data in the buffer before sending the command (prevents errors in case all data is not read)
  }
  CameraSerial.write(command, 6);                   //send each of the 6 bytes to the camera
}

void checkIridium() {
  Serial.println(F("This program will attempt to send message and receive the next message in the queue if one exists"));
  Serial.println(F("If not successful for any reason, this attempt will timeout after a maximum of 5 minutes"));
  Serial.println(F("Otherwise, this attempt can be stopped manually by closing this program\n"));
  Serial.println(F("Turning On Iridium Module...\n"));

  turnOnIridiumPower();					    						            //turn on power to Iridium module
  delay(100);						    						                    //short delay to let voltage stabilize
  int16_t signalQuality = -1;				    						        //Initialize signal quality variable

  IridiumSerial.begin(19200);  					    		            //start serial comms with Iridium module at 19200 baud
  isbd.attachConsole(Serial);                               //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only)
  isbd.attachDiags(Serial);                               //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only)
  isbd.setPowerProfile(0);                                  //set power profile to 0 for high power application(500mA); 1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.setMinimumSignalQuality(2);			    			          //minimum signal quality (0-5) needed before sending message.  normally set to 2 (by isbd recommendations)
  isbd.adjustSendReceiveTimeout(300);
  isbd.begin();                                             //wake up the 9603 and prepare it to communicate

  int16_t err = isbd.getSignalQuality(signalQuality);       //get the signal quality from the Iridium module
  if (err != 0)                                             //if there is an error getting signal quality
  {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(err);                                    //print error to serial monitor
    Serial.println(F("\nTurning Off Iridium Module..."));
    turnOffIridiumPower();					    						        //switch off voltage to Iridium module
    IridiumSerial.end();                                    //end serial comms with Iridium module
    return;                                                 //end Iridium test and go back to main menu
  }

  Serial.print(F("Signal quality is "));
  Serial.println(signalQuality);                            //print signal quality to serial monitor

  const uint16_t sendbuffer_size = BYTES_PER_SBDMESSAGE + 1; //calculate max size of array for send message buffer
  uint8_t iridiumSendBuffer[BYTES_PER_SBDMESSAGE+1];		    			  //initialize array for send message buffer (reserve adequate space in memory)

  size_t sendMessageSize = fillSendBufferFirstMessage(iridiumSendBuffer);
  Serial.print(F("Message Size:"));
  Serial.println(sendMessageSize);
  uint8_t iridiumReceiveBuffer[RECEIVE_BUFFER_SIZE];			  //Initialize array for received message (received messages are only 6 bytes to change filename/position)
  emptyReceiveBuffer(iridiumReceiveBuffer);                 //Empty the contents of the receive buffer array
  size_t receiveBufferSize = RECEIVE_BUFFER_SIZE;           //Define size of array for received message
  
  //err = isbd.sendSBDText("Hello, world!");
  err = isbd.sendReceiveSBDBinary(iridiumSendBuffer, sendMessageSize, iridiumReceiveBuffer, receiveBufferSize);	    //Attempt to send first message ("0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF")
//    uint8_t buffer[200] = 
//  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
//  size_t bufferSize = sizeof(buffer);
//  err = isbd.sendReceiveSBDBinary(buffer, 20, buffer, bufferSize);

  if (err != 0) {                                           //if there are any errors when sending
    Serial.print(F("Message Send Attempt failed: error code = "));
    Serial.println(err);                                    //print error to serial monitor
    Serial.println(F("\nTurning Off Iridium Module..."));
    turnOffIridiumPower();                                  //switch off voltage to Iridium module
    IridiumSerial.end();                                    //end serial comms with Iridium module
    return;                                                 //end Iridium test and go back to main menu
  }

  Serial.println(F("Message Sent Successfully!"));
  if (iridiumReceiveBuffer[0] != 0) {                       //if there is any data in the recieved message
    Serial.println(F("Received Message: "));
    printReceivedMessage(iridiumReceiveBuffer);             //print the received message to the serial monitor
  }
  Serial.print(F("Messages Waiting: "));
  Serial.println(isbd.getWaitingMessageCount());          //print the number of remining messages to the serial monitor

  Serial.println(F("\nTurning Off Iridium Module..."));
  turnOffIridiumPower();                                    //switch off voltage to Iridium module
  IridiumSerial.end();                                      //end serial comms with Iridium module
}

uint16_t fillSendBufferFirstMessage(uint8_t sendBuffer[]) {
  for (uint8_t k = 0; k < 10; k++) {
    sendBuffer[k] = 0x0F;			            						//First message to send if unit wake up is "0xFF(1)0xFF(2)...0xFF(10)"  (preferred to not send data in case of file corruption - unit will still respond if data won't send)
  }
  return 10;
}

void emptyReceiveBuffer(uint8_t receiveBuffer[]) {        //function to empty the contents of the received message
  for (int y = 0; y < RECEIVE_BUFFER_SIZE; y++) {         //for each element of the received message array
    receiveBuffer[y] = 0;			            					      //set the element to 0
  }
}

void printReceivedMessage(uint8_t receiveBuffer[]) {      //Function to parse received message and print contents in readable format to serial monitor
  for (int y = 0; y < RECEIVE_BUFFER_SIZE; y++) {         //for each element of the received message array
    Serial.print(receiveBuffer[y], HEX);                  //print the element in hex format
    Serial.print(F(","));                                 //print delimiter
  }
}

void setRTC() {
  Serial.println(F("Enter Current Date:"));             //prompt user for input
  Serial.println(F("MMDDYY"));                          //prompt user for input
  char inputbuffer[6];                                  //buffer for user input of time/date
  boolean checkdigit = true;                            //variable to check if user input is actually digits
  boolean waitingforInput2 = true;                      //variable to wait for user input
  byte readLen = 0;                                     //variable for how many bytes have been read
  byte MM;                                              //variables to store time and date values
  byte DD;                                              //variables to store time and date values
  byte YY;                                              //variables to store time and date values
  byte hh;                                              //variables to store time and date values
  byte mm;                                              //variables to store time and date values
  byte ss;                                              //variables to store time and date values
  while (waitingforInput2 == true) {                    //until the user inputs 6 characters, keep reading serial data
    readLen = Serial.readBytes(inputbuffer, 6);         //read serial monitor input (6 characters)
    if (readLen == 6) {                                 //if user has put in 6 characters
      for (int i = 0; i < 6; i++) {                     //go through each character in the buffer
        if (inputbuffer[i] < 48 || inputbuffer[i] > 57) { //if any character is not a digit (ASCII value of 48-57)
          checkdigit = false;                           //flag that the input is wrong
        }
      }
      waitingforInput2 = false;                         //stop waiting for input and exit the dialog
    }
  }
  if (checkdigit == true) {                             //if data input so far is valid
    MM = 10 * (inputbuffer[0] - 48) + (inputbuffer[1] - 48); //parse input data into date variables
    DD = 10 * (inputbuffer[2] - 48) + (inputbuffer[3] - 48); //parse input data into date variables
    YY = 10 * (inputbuffer[4] - 48) + (inputbuffer[5] - 48); //parse input data into date variables
    Serial.print(MM);
    Serial.print(F("/"));
    Serial.print(DD);
    Serial.print(F("/"));
    Serial.println(YY);
    Serial.println();
  }
  else {                                                //if data input is not valid
    Serial.println(F("**Input Error"));                  //print an error to the serial monitor
  }
  waitingforInput2 = true;                              //reset variable waiting for user input
  if (checkdigit == true) {                             //if data input so far is valid
    Serial.println(F("Enter Current Time:"));           //prompt user for input
    Serial.println(F("hhmmss"));                        //prompt user for input
    while (waitingforInput2 == true) {                  //until the user inputs 6 characters, keep reading serial data
      readLen = Serial.readBytes(inputbuffer, 6);       //read serial monitor input (6 characters)
      if (readLen == 6) {                               //check if user has put in 6 characters
        for (int i = 0; i < 6; i++) {                   //go through each character in the buffer
          if (inputbuffer[i] < 48 || inputbuffer[i] > 57) { //if any character is not a digit (ASCII value of 48-57)
            checkdigit = false;                         //flag that the input is wrong
          }
        }
        waitingforInput2 = false;                       //stop waiting for input and exit the dialog
      }
    }
    if (checkdigit == true) {                           //if data input so far is valid
      hh = 10 * (inputbuffer[0] - 48) + (inputbuffer[1] - 48); //parse input data into variables
      mm = 10 * (inputbuffer[2] - 48) + (inputbuffer[3] - 48); //parse input data into variables
      ss = 10 * (inputbuffer[4] - 48) + (inputbuffer[5] - 48); //parse input data into variables
      Serial.print(hh);
      Serial.print(F(":"));
      Serial.print(mm);
      Serial.print(F(":"));
      Serial.println(ss);
      Serial.println();
    }
    else {                                              //if data input is not valid
      Serial.println(F("**Input Error"));                //print an error to the serial monitor
    }
  }
  if (checkdigit == true) {                             //if data input was valid for time and date, then reset the alarm with the right values
    DateTime settime = DateTime(YY, MM, DD, hh, mm, ss); //make a DateTime variable using the input the user gave
    SPI.setDataMode(SPI_MODE1);                         //Set SPI Data Mode to 1 for RTC
    RTC.adjust(settime);                                //set the RTC time/date
    SPI.setDataMode(SPI_MODE2);                         //Set SPI Data Mode
    Serial.println(F("**Time set successfully!**"));
    displayRTCTime();                                   //display new RTC time/date on serial monitor
  }
}

void sdDataCheck() {
  Serial.println(F("Checking SD Card Files..."));
  SPI.setDataMode(SPI_MODE0);                        //Set SPI Data Mode
  File myFile = SD.open("botdat.txt", FILE_READ);    //open the file for reading:
  if (myFile) {                                      //if the file opened okay, write to it:
    Serial.println();                                //write to serial window
    Serial.println(F("botdat.txt:"));                //write to serial window
    while (myFile.available()) {                     //read from the file until there's nothing else in it:
      Serial.print(myFile.read(),HEX);                   //write output from file to serial window
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
      Serial.print(myFile.read(), HEX);                   //write output from file to serial window
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
      Serial.print(myFile.read(),HEX);                   //write output from file to serial window
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
      Serial.print(myFile.read(),HEX);                   //write output from file to serial window
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

boolean deleteFilesPrompt() {
  Serial.println(F("This will delete all files on the SD Card."));
  Serial.println(F("Are you sure you want to delete? (Enter 'Y' to delete, 'N' to cancel)"));
  boolean waitingforInput2 = true;                                              //variable to wait for user input
  byte readByte = 0;                                                            //variable to read input from Serial Monitor
  while (waitingforInput2 == true) {                                            //until the user enters something, keep waiting for input
    if (Serial.available()) {                                                   //if any data is input on the serial monitor
      readByte = Serial.read();                                                 //read the serial input
      if (readByte == 89) {                                                     //if user enters 'Y', then
        return true;                                                            //return true to delete files
      }
      else {                                                                    //for any other input
        return false;                                                           //return false and don't delete files
      }
      waitingforInput2 = false;                                                //stop waiting and exit the dialog
    }
  }
}

void deleteFiles() {
  SPI.setDataMode(SPI_MODE0);                                         //Set SPI Data Mode to 0 for SDCard
  SD.remove("prodat.txt");                                            //delete prodat.txt
  Serial.println(F("Deleted prodat.txt"));
  SD.remove("icedat.txt");                                            //delete icedat.txt
  Serial.println(F("Deleted icedat.txt"));
  SD.remove("botdat.txt");                                            //delete botdat.txt
  Serial.println(F("Deleted sstdat.txt"));
  SD.remove("sstdat.txt");                                            //delete botdat.txt
  Serial.println(F("Deleted botdat.txt"));
  SD.remove("summary.txt");                                           //delete summary.txt
  Serial.println(F("Deleted summary.txt"));
  SD.remove("filepos.txt");                                           //delete filepos.txt
  Serial.println(F("Deleted filepos.txt"));
  SD.remove("jpgorder.txt");                                          //delete jpgorder.txt
  Serial.println(F("Deleted jpgorder.txt"));
  Serial.println(F("**Please remove any jpg image files manually by inserting SD Card into a computer**"));
}

void setAlarmTime(DateTime alarmtime) {      //function to set the alarmtime on the DS3234 RTC
  SPI.setDataMode(SPI_MODE1);                //Set SPI Data Mode to 1 for RTC
  RTCWrite(0x87, alarmtime.second() & 0x7F); //set alarm time: seconds     //87=write to location for alarm seconds    //binary & second with 0x7F required to turn alarm second "on"
  RTCWrite(0x88, alarmtime.minute() & 0x7F); //set alarm time: minutes     //88=write to location for alarm minutes    //binary & minute with 0x7F required to turn alarm minute "on"
  RTCWrite(0x89, alarmtime.hour() & 0x7F);   //set alarm time: hour        //89=write to location for alarm hour       //binary & hour with 0x7F required to turn alarm hour "on"
  RTCWrite(0x8A, alarmtime.day() & 0x3F);    //set alarm time: day         //8A=write to location for alarm day        //binary & day with 0x3F required to turn alarm day "on" (not dayofWeek)
  RTCWrite(0x8B, 0);                         //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8C, 0);                         //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8D, 0);                         //Set Alarm #2 to zll zeroes (disable)
  RTCWrite(0x8F, B00000000);                 //reset flags                //8F=write to location for control/status flags    //B00000000=Ocillator Stop Flag 0, No Batt Backed 32 KHz Output, Keep Temp CONV Rate at 64 sec (may change later), disable 32 KHz output, temp Not Busy, alarm 2 not tripped, alarm 1 not tripped
  RTCWrite(0x8E, B00000101);                 //set control register       //8E=write to location for control register        //B01100101=Oscillator always on, SQW on, Convert Temp off, SQW freq@ 1Hz, Interrupt enabled, Alarm 2 off, Alarm 1 on
  SPI.setDataMode(SPI_MODE2);                //Set SPI Data Mode
}

void RTCWrite(char reg, char val) {          //function to send SPI data to RTC in the proper format
  SPI.setClockDivider(SPI_CLOCK_DIV16);      //Set SPI Clock Speed
  digitalWrite(RTC_CS, LOW);                 //enable SPI read/write for chip
  SPI.transfer(reg);                         //define memory register location
  SPI.transfer(bin2bcd(val));                //write value
  digitalWrite(RTC_CS, HIGH);                //disable SPI read/write for chip
}

void sendSPIdata(char pin, byte b1, byte b2) { //generic function to send SPI data
  digitalWrite(pin, LOW);                     //pull chip select pin low to select
  SPI.transfer(b1);                           //transfer first byte
  SPI.transfer(b2);                           //transfer secons byte
  digitalWrite(pin, HIGH);                    //pull chip select pin high to deselect
}

void turnOn3v3SensorPower() {
  digitalWrite(SENSOR_POWER_3V3, LOW);  		 //Pin LOW=Power ON (P-Channel switch)
}

void turnOff3v3SensorPower() {
  digitalWrite(SENSOR_POWER_3V3, HIGH);    	//Pin HIGH=Power OFF (P-Channel switch)
}

void turnOn5vSensorPower() {
  digitalWrite(SENSOR_POWER_5V, LOW);  		 //Pin LOW=Power ON (P-Channel switch)
}

void turnOff5vSensorPower() {
  digitalWrite(SENSOR_POWER_3V3, HIGH);  	 //Pin HIGH=Power OFF (P-Channel switch)
}

void turnOnFluorometer() {
  digitalWrite(FLUOROMETER_ON_OFF, LOW);  		//Pin HIGH=Power OFF (P-Channel switch)
}
void turnOffFluorometer() {
  digitalWrite(FLUOROMETER_ON_OFF, HIGH);   //Pin HIGH=Power OFF (P-Channel switch)
}

void turnOnGPSPower() {
  digitalWrite(GPS_POWER, LOW);  			     //Pin LOW=Power ON (P-Channel switch)
}
void turnOffGPSPower() {
  digitalWrite(GPS_POWER, HIGH);  			     //Pin HIGH=Power OFF (P-Channel switch)
}

void turnOnIridiumPower() {
  digitalWrite(IRIDIUM_POWER, LOW);  		   //Pin LOW=Power ON (P-Channel switch)
  digitalWrite(IRIDIUM_ON_OFF, HIGH);  		 //Digital Pin for Iridium module sleep.  HIGH = wake from sleep
}
void turnOffIridiumPower() {
  digitalWrite(IRIDIUM_POWER, HIGH);  		   //Pin HIGH=Power OFF (P-Channel switch)
  digitalWrite(IRIDIUM_ON_OFF, LOW);  		   //Digital Pin for Iridium module sleep.  LOW = go to sleep
}

void turnOnCameraPower() {
  digitalWrite(CAMERA_ON_OFF, LOW);  		   //Pin LOW=Power ON (P-Channel switch)
  digitalWrite(CAMERA_RESET, HIGH);  		   //Digital Pin for Camera Reset.  HIGH = not active
}

void turnOffCameraPower() {
  digitalWrite(CAMERA_ON_OFF, HIGH);  		   //Pin HIGH=Power OFF (P-Channel switch)
  digitalWrite(CAMERA_RESET, LOW);  		     //Digital Pin for Iridium module sleep.  LOW = active reset
}

void turnOnSerialComms() {
  digitalWrite(SERIAL_COMMS_ON_OFF, HIGH);  	//Pin LOW=Power ON (P-Channel switch)
}
void turnOffSerialComms() {
  digitalWrite(SERIAL_COMMS_ON_OFF, LOW);  	//Pin HIGH=Power OFF (P-Channel switch)
}

void setupPARADC() {
  Wire.beginTransmission(ads1100A1);    //Begin I2C comms with Temp ADC
  Wire.write(B10001100);      //set ADC gain to 1, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();     //End I2C Comms
  delay(50);          //Short delay to let ADC initialize
}

int16_t readPAR() {
  uint8_t controlRegister = 0;      //initialize control register variable
  int16_t adcVal = 0;       //initialize ADC Value variable
  Wire.requestFrom(ads1100A1, 3);           //request 3 bytes from appropriate ADC using I2C
  while (Wire.available()) {    //ensure all the data comes in
    adcVal = Wire.read();       //first byte is MSB
    adcVal = ((unsigned int)adcVal << 8); //shift first byte 8 bits to the left to move it into MSB
    adcVal += Wire.read();      //second byte is LSB.  add this to the MSB
    controlRegister = Wire.read();          //third byte is control register
  }
  return adcVal;        //return single value for ADC reading
}

void setupFluorADC() {
  Wire.beginTransmission(ads1100A2);		//Begin I2C comms with Fluor ADC
  Wire.write(B10001100); 			          //set ADC gain to 1, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();			          //End I2C Comms
  delay(50);					                  //Short delay to let ADC initialize
}

int16_t readFluor() {
  uint8_t controlRegister = 0;               //initialize control register variable
  int16_t adcVal = 0;                     //initialize ADC Value variable
  Wire.requestFrom(ads1100A2, 3);         //request 3 bytes from appropriate ADC using I2C
  while (Wire.available()) {              //ensure all the data comes in
    adcVal = Wire.read();                 //first byte is MSB
    adcVal = ((unsigned int)adcVal << 8);     //shift first byte 8 bits to the left to move it into MSB
    adcVal += Wire.read();                //second byte is LSB.  add this to the MSB
    controlRegister = Wire.read();        //third byte is control register
  }
  return adcVal;                          //return single value for ADC reading
}

void setupTempADC() {
  Wire.beginTransmission(ads1100A0);		//Begin I2C comms with Temp ADC
  Wire.write(B10001110); 			          //set ADC gain to 4, 8 Samples per Second (16-bit Resolution)
  Wire.endTransmission();			          //End I2C Comms
}

int16_t readTopTemp() {			             //function to read temperature ADC for top thermistor
  digitalWrite(TEMP_SWITCH_1, HIGH);		 //send signal to switch to top thermistor reading
  digitalWrite(TEMP_SWITCH_2, LOW);		   //send signal to switch to top thermistor reading
  delay(1000);						               //delay to let voltage stabilize
  int16_t measurement = readTempADC();	 //read the ADC (Same for thermistor or reference)
  digitalWrite(TEMP_SWITCH_1, LOW);		   //send signal to switch back to reference resistor 1
  digitalWrite(TEMP_SWITCH_2, LOW);		   //send signal to switch back to reference resistor 2
  return measurement;					           //return the measured value from the ADC
}

int16_t readSST() {			                 //function to read temperature ADC for SST thermistor
  digitalWrite(TEMP_SWITCH_1, HIGH);     //send signal to switch to top thermistor reading
  digitalWrite(TEMP_SWITCH_2, HIGH);     //send signal to switch to top thermistor reading
  delay(1000);                           //delay to let voltage stabilize
  int16_t measurement = readTempADC();   //read the ADC (Same for thermistor or reference)
  digitalWrite(TEMP_SWITCH_1, LOW);      //send signal to switch back to reference resistor 1
  digitalWrite(TEMP_SWITCH_2, LOW);      //send signal to switch back to reference resistor 2
  return measurement;                    //return the measured value from the ADC
}

int16_t testSST() {                      //function to read temperature ADC for SST thermistor 
  digitalWrite(TEMP_SWITCH_1, HIGH);     //send signal to switch to top thermistor reading
  digitalWrite(TEMP_SWITCH_2, HIGH);     //send signal to switch to top thermistor reading
  delay(250);                            //sample ADC 4 times per second (4Hz or once every 250 milliseconds)
  int16_t measurement = readTempADC();   //read the ADC (Same for thermistor or reference)
  digitalWrite(TEMP_SWITCH_1, LOW);      //send signal to switch back to reference resistor 1
  digitalWrite(TEMP_SWITCH_2, LOW);      //send signal to switch back to reference resistor 2
  return measurement;                    //return the measured value from the ADC
}

int16_t testTopTemp() {                  //function to read temperature ADC for top thermistor
  digitalWrite(TEMP_SWITCH_1, HIGH);     //send signal to switch to top thermistor reading
  digitalWrite(TEMP_SWITCH_2, LOW);      //send signal to switch to top thermistor reading
  delay(250);                            //sample ADC 4 times per second (4Hz or once every 250 milliseconds)
  int16_t measurement = readTempADC();   //read the ADC (Same for thermistor or reference)
  digitalWrite(TEMP_SWITCH_1, LOW);      //send signal to switch back to reference resistor 1
  digitalWrite(TEMP_SWITCH_2, LOW);      //send signal to switch back to reference resistor 2
  return measurement;                    //return the measured value from the ADC
}

int16_t readTempRef() {			             //function to read temperature ADC for temp ref
  delay(1000);                           //delay to let voltage stabilize
  int16_t measurement = readTempADC();   //read the ADC (Same for thermistor or reference)
  return measurement;                    //return the measured value from the ADC
}

int16_t readTempADC() {
  uint8_t controlRegister = 0;				    //initialize control registoer variable
  int16_t adcVal = 0;						          //initialize ADC Value variable
  Wire.requestFrom(ads1100A0, 3);  			  //request 3 bytes from appropriate ADC using I2C
  while (Wire.available()) { 					    //ensure all the data comes in
    adcVal = Wire.read(); 		        	  //first byte is MSB
    adcVal = ((unsigned int)adcVal << 8); //shift first byte 8 bits to the left to move it into MSB
    adcVal += Wire.read(); 					      //second byte is LSB.  add this to the MSB
    controlRegister = Wire.read();			  //third byte is control register
  }
  return adcVal;							            //return single value for ADC reading
}

uint16_t readPressure() {	                       //function to read pressure measurement from sensor: depth in m = [(reading-16384)*maxbar/32768]*10
  uint8_t pressureStatus;				                  //initialize variable for sensor status
  uint16_t pressureReading;				                //initialize variable for pressure reading
  uint8_t eocDelay = 8;					                  //variable for conversion delay (8 ms defined in Keller communication protocol)
  Wire.beginTransmission(kellerADDR);  	          //Begin I2C comms with pressure sensor
  Wire.write(kellerCheckPressure);  	            //Send write command to start pressure conversion
  Wire.endTransmission();				                  //End command for pressure conversion
  delay(eocDelay);                                //Delay for conversion (8 ms defined in Keller communication protocol)
  Wire.requestFrom(kellerADDR, 3);        	       //Read data from pressure sensor
  while (Wire.available()) {               	      //Ensure all the data comes in
    pressureStatus = Wire.read(); 			          //First byte is Sensor Status  (Can possibly be used for error checking)
    pressureReading = Wire.read(); 				        //Second byte is Pressure reading MSB
    pressureReading = (pressureReading << 8);		  //Shift second byte to MSB
    pressureReading += Wire.read(); 				      //Third byte is LSB, add to pressure reading
  }
  return pressureReading;					                //Return value for pressure only
}

void setupAccel() {				                     //Function to Setup ADXL345 Chip (accelerometer)
  SPI.setDataMode(SPI_MODE3);                   //Set SPI Data Mode to 3 for Accelerometer
  sendSPIdata(ACCEL_CS, 0x2C, B00001010);		    //0x2C=data rate and power mode control, B00001010=normal power mode, SPI data rate =100HZ
  sendSPIdata(ACCEL_CS, 0x31, B00001011);		    //0x31=data format control, B00001011=no self test, SPI mode, Active LOW, full resolution mode, left justified (MSB), +/-16g range
  sendSPIdata(ACCEL_CS, 0x2D, B00001000);		    //0x2D=power control, B00001000=measurement mode
  SPI.setDataMode(SPI_MODE2);                   //Set SPI Data Mode to default
}

uint8_t readAccel() {						         //function to Read tilt angle from ADXL345 Chip
  SPI.setDataMode(SPI_MODE3);		          //Set SPI Data Mode to 3 for Accelerometer
  digitalWrite(ACCEL_CS, LOW);				    //Select Accelerometer for SPI Communication
  SPI.transfer(0x32 | 0xC0);				      //Request x,y,and z Acceleration Values from Accelerometer
  byte x0 = SPI.transfer(-1);				      //x0=Accel LSB
  byte x1 = SPI.transfer(-1);				      //x1=Accel MSB
  byte y0 = SPI.transfer(-1);				      //y0=Accel LSB
  byte y1 = SPI.transfer(-1);				      //y1=Accel MSB
  byte z0 = SPI.transfer(-1);				      //z0=Accel LSB
  byte z1 = SPI.transfer(-1);				      //z1=Accel MSB
  digitalWrite(ACCEL_CS, HIGH);				    //De-Select Accelerometer for SPI Communication

  float x = x1 << 8 | x0;						                //Combine x1(MSB) and x0(LSB) into x
  float y = y1 << 8 | y0;						                //Combine y1(MSB) and y0(LSB) into y
  float z = z1 << 8 | z0;						                //Combine z1(MSB) and z0(LSB) into z
  float xg = x * 3.9 / 1000.0;					            //Convert x accel value to g force units
  float yg = y * 3.9 / 1000.0;					            //Convert x accel value to g force units
  float zg = z * 3.9 / 1000.0;					            //Convert x accel value to g force units
  float gForce = sqrt(sq(xg) + sq(yg) + sq(zg));	  //Find Total G force (3 dim Pythagorean)
  float tiltAngle = acos(-zg / gForce) / 3.14159 * 180.0; //calculate the angle using vertical component of g-force
  uint8_t truncatedAngle = tiltAngle;               //convert angle to a single byte integer value
  return truncatedAngle;				                    //Return the angle from vertical
}

bool ISBDCallback(){
//   if(Serial.available()){
//     uint8_t byteRead=Serial.read();
//      if(byteRead==81){
//        return false;
//      }
//   }
   return true;
}
