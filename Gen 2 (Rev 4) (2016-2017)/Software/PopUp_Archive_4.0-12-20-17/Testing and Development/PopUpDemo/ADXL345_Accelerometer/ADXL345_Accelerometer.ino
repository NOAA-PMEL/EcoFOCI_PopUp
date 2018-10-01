//include various libraries
#include <ctype.h>
#include <Wire.h>
#include <SPI.h>  
#include <math.h>

#define adxlPin 4                           //Chip Select for ADXL

void setup(){
    SPI.begin();                             //Start up SPI Communication (needed for various ICs) 
    SPI.setDataMode(SPI_MODE3);              //Configure SPI Communication (needed for various ICs)
    SPI.setBitOrder(MSBFIRST);               //Configure SPI Communication (needed for various ICs)
    Serial.begin(9600);                      //Turn the Serial Protocol ON
    Serial.println(F("**Setting Up ADXL345 Accelerometer"));
    setupADXL();
    Serial.println(F("Set up Complete!!"));
    delay(2000);                              
}


void loop(){
      //"OPTIONS" Menu
      readADXL();

      delay(2000);
}

void setupADXL(){
  pinMode(adxlPin,OUTPUT);
  digitalWrite(adxlPin,HIGH);
  SPI.setDataMode(SPI_MODE3);                                //Set SPI Data Mode
  sendSPIdata(adxlPin,0x2C,B00001010);
  sendSPIdata(adxlPin,0x31,B00001011);
  //sendSPIdata(adxlPin,0x38,B10000000);  //FIFO Mode 'Stream'
  sendSPIdata(adxlPin,0x2D,B00001000);
}

void sendSPIdata(char pin, byte b1, byte b2){
  digitalWrite(pin, LOW);
  SPI.transfer(b1);
  SPI.transfer(b2);
  digitalWrite(pin, HIGH);
}

void readADXL(){
  digitalWrite(adxlPin, LOW);
  SPI.transfer(0x32 | 0xC0);
  byte x0 = SPI.transfer(-1);
  byte x1 = SPI.transfer(-1);
  byte y0 = SPI.transfer(-1);
  byte y1 = SPI.transfer(-1);
  byte z0 = SPI.transfer(-1);
  byte z1 = SPI.transfer(-1);
//  Serial.print(x0,BIN);
//  Serial.print("-");
//  Serial.println(x1,BIN);
//  Serial.print(y0,BIN);
//  Serial.print("-");
//  Serial.println(y1,BIN);
//  Serial.print(z0,BIN);
//  Serial.print("-");
//  Serial.println(z1,BIN);
  digitalWrite(adxlPin, HIGH);
  
  float x = x1<<8 | x0;
  float y = y1<<8 | y0;
  float z = z1<<8 | z0;
  Serial.println();
  Serial.print(F("X="));
  Serial.println(x);
  Serial.print(F("Y="));
  Serial.println(y);
  Serial.print(F("Z="));
  Serial.println(z);

  float xg = x*3.9/1000;
  float yg = y*3.9/1000;
  float zg = z*3.9/1000;
  Serial.println();
  Serial.print(F("Xg="));
  Serial.println(xg,4);
  Serial.print(F("Yg="));
  Serial.println(yg,4);
  Serial.print(F("Zg="));
  Serial.println(zg,4);
  
  float gForce=sqrt(sq(xg)+sq(yg)+sq(zg));
  float tiltCos=zg/gForce;
  
  Serial.println(F("G Force="));      
  Serial.println(gForce,4);
  Serial.println(F("Tilt Angle="));      
  Serial.println(tiltCos,4);

  
}
