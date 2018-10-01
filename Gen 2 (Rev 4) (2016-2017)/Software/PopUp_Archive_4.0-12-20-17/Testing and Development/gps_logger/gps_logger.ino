#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <ctype.h>
#include <Wire.h>
#include <SPI.h>  


// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software myFile (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware myFile (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
// If using software myFile, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false
#define sdcardPin  8
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
File myFile;
void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
  SPI.setDataMode(SPI_MODE2);              //Configure SPI Communication (needed for various ICs)
  SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)  
  pinMode(10,OUTPUT);
  SD.begin(sdcardPin);                                       //Start SD Card so it is available for other function calls
  Serial.begin(115200);
  myFile = SD.open("gpsdata.txt", FILE_WRITE);   //open test.txt
  if (myFile) {                                    //if the file opened okay, write to it:
      Serial.print("Writing to test.txt...");        //write to serial window 
      myFile.close();
  } 
  Serial.println("Adafruit GPS library basic test!");
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 5 seconds or so, print out the current stats
  if (millis() - timer > 5000) { 
    timer = millis(); // reset the timer
    Serial.print("Test");
    File myFile = SD.open("gpsdata.txt", FILE_WRITE);   //open test.txt
    if (myFile) {                                    //if the file opened okay, write to it:
      Serial.print("Writing to test.txt...");        //write to serial window 
//      myFile.print("\nTime: ");
//      myFile.print(GPS.hour, DEC); myFile.print(':');
//      myFile.print(GPS.minute, DEC); myFile.print(':');
//      myFile.print(GPS.seconds, DEC); myFile.print('.');
//      myFile.println(GPS.milliseconds);
//      myFile.print("Date: ");
//      myFile.print(GPS.day, DEC); myFile.print('/');
//      myFile.print(GPS.month, DEC); myFile.print("/20");
//      myFile.println(GPS.year, DEC);
//      myFile.print("Fix: "); myFile.print((int)GPS.fix);
//      myFile.print(" quality: "); myFile.println((int)GPS.fixquality); 
//      if (GPS.fix) {
//        myFile.print("Location: ");
//        myFile.print(GPS.latitude, 4); myFile.print(GPS.lat);
//        myFile.print(", "); 
//        myFile.print(GPS.longitude, 4); myFile.println(GPS.lon);
//        myFile.print("Location (in degrees, works with Google Maps): ");
//        myFile.print(GPS.latitudeDegrees, 4);
//        myFile.print(", "); 
//        myFile.println(GPS.longitudeDegrees, 4);
//        
//        myFile.print("Speed (knots): "); myFile.println(GPS.speed);
//        myFile.print("Angle: "); myFile.println(GPS.angle);
//        myFile.print("Altitude: "); myFile.println(GPS.altitude);
//        myFile.print("Satellites: "); myFile.println((int)GPS.satellites);
//    }
      myFile.close();                                //close the file
      Serial.println("done.");                       //write to serial window
    } 
    else {                                           //if the file didn't open, print an error:
      Serial.println("error opening test.txt");      //write to serial window
    }
    

  }
}
