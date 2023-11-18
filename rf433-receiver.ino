/*
  Simple example for receiving
  https://github.com/sui77/rc-switch/
*/
#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  Serial.println("started");
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (mySwitch.available()) {
    
    // Serial.print("Received ");
    // Serial.print( mySwitch.getReceivedValue() );
    // Serial.print(" / ");
    // Serial.print( mySwitch.getReceivedBitlength() );
    // Serial.print("bit ");
    // Serial.print("Protocol: ");
    // Serial.println( mySwitch.getReceivedProtocol() );
    Serial.println(mySwitch.getReceivedValue());
    delay(1000);
    mySwitch.resetAvailable();
  }

}
