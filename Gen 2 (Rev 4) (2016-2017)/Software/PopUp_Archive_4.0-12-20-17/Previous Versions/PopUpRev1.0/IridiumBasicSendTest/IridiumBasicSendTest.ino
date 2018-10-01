#include <IridiumSBD.h>
#include <SoftwareSerial.h>

SoftwareSerial nss(A14, A15);
IridiumSBD isbd(nss, 24);
static const int ledPin = 13;



void setup()
{
  int signalQuality = -1;

  pinMode(ledPin, OUTPUT);
  
  pinMode(A0, OUTPUT);
  digitalWrite(A0,HIGH);
  
  pinMode(22, OUTPUT);
  digitalWrite(22,LOW);
  
  Serial.begin(115200);
  nss.begin(19200);  //start serial comms on nss pins defined above

  isbd.attachConsole(Serial); //Stream object that will monitor serial traffic to/from the RockBlock  (diagnostic only) 
  isbd.setPowerProfile(1); //1 for low power application (USB, limited to ~90mA, interval between transmits = 60s)
  isbd.begin();  //wake up the 9602 and prepare it to communicate
  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

  err = isbd.sendSBDText("Pop Up Rig Test 128");
  if (err != 0)
  {
    Serial.print("sendSBDText failed: error ");
    Serial.println(err);
    return;
  }

  Serial.println("Hey, it worked!");
  Serial.print("Messages left: ");
  Serial.println(isbd.getWaitingMessageCount());
}

void loop()
{
   digitalWrite(ledPin, HIGH);
}

bool ISBDCallback()
{
   digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
   return true;
}
