#include <SD.h>
#include <ctype.h>
#include <Wire.h>

#define sdcardPin  8

File myFile;

void setup()
{
  pinMode(10,OUTPUT);
  SD.begin(sdcardPin);                                       //Start SD Card so it is available for other function calls
  Serial.begin(115200);
  myFile = SD.open("gpsdata.txt", FILE_READ);   //open test.txt
  if (myFile) {                                    //if the file opened okay, write to it:
    Serial.println("gpsdata.txt:");                   //write to serial window
    while (myFile.available()) {                   //read from the file until there's nothing else in it:
    	Serial.write(myFile.read());                 //write output from file to serial window
    }
    myFile.close();                                //close the file
  } 
  else {
    Serial.println("error opening test.txt");      // if the file didn't open, print an error:
  }
}

void loop()
{

}

