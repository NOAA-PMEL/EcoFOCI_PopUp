/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  unsigned int depthAve=depthAverage();
  Serial.print("Average=");
  Serial.println(depthAve);
  unsigned int shortAverage = depthAve-1500;
  Serial.print("profilecutoff=");
  Serial.println(shortAverage);
  
//  unsigned long endTime=millis()+300000;
//  Serial.print("Collecting Profile:EndTime=");
//  Serial.println(endTime);
//  while(millis()<endTime){
//    Serial.println(millis());
//    delay(1000);
//  }
//  Serial.print("Profile Complete");
  
  unsigned long gpsDateVal=91233;
  unsigned long gpsTimeVal=12300455;
  double gpsLat=43.9875;
  double gpsLon=-123.3744;


  String sendGPSFix = "";
  sendGPSFix += String(gpsLat,4);
  sendGPSFix += ",";
  sendGPSFix += String(gpsLon,4);
  sendGPSFix += ",";
  sendGPSFix += gpsTimeVal;
  sendGPSFix += ",";
  sendGPSFix += gpsDateVal;
  Serial.println(sendGPSFix);
  
  delay(10000);
  
}

unsigned int depthAverage(){
  long depthAverage=0;
  for(char samples=0;samples<20;samples++){
     depthAverage=depthAverage+40000; 
  }
  Serial.print("Sum=");
  Serial.println(depthAverage);
  depthAverage=depthAverage/20;
  return depthAverage;
}
