#include <SPI.h>
#include <RH_RF69.h>
#include "MyPacketStruct.h"

//Radio variables
#define RF69_FREQ 915.0
//#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     2  // 
  #define RFM69_CS      10  //
  #define RFM69_RST     6  // "A"
//  #define LED           40  //dont use yet
//#endif
RH_RF69 rf69(RFM69_CS, RFM69_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission

//custom radio packet
MyRadioPacket message;
int remoteHelp = 0;
#define GATEWAYID   1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  //Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    //Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  //pinMode(LED, OUTPUT);
  Serial.println("Done with init.");
  delay(1000);
}

float T = 0;
float beatsPerMinute = 0;
float spo2 = 0;
float px = 0;
float py = 0;
float pz = 0;
float buttonState = 0;

void loop() {
  T += 1;
  beatsPerMinute += 1;
  spo2 += 1;
  px += 1;
  py += 1;
  pz += 1;
  buttonState += 1;
  
  dtostrf(T, 5, 0, message.temp);
  dtostrf(beatsPerMinute, 5, 0, message.bpm);
  dtostrf(spo2, 5, 0, message.spO2);
  dtostrf(px, 5, 0, message.posX);
  dtostrf(py, 5, 0, message.posY);
  dtostrf(pz, 5, 0, message.posZ);
  dtostrf(buttonState, 5, 0, message.helpButton);
  String sT = message.temp;
  //Serial.println(sT);
  //Serial.print(message.temp); 
  String s = " ";
  //String num = "\0";
  //char radiopacket[31] = message.radioID + s + message.temp + s + message.bpm + s + message.spO2 + s + message.posX + s + message.posY + s + message.posZ + s + message.helpButton;
  //String radiopacket = message.temp + s + message.bpm + s + message.spO2 + s + message.posX + s + message.posY + s + message.posZ + s + message.helpButton + s;
  
  //Serial.println(newPacket);
  //int packetLen = strlen(newPacket);
  //.toCharArray(newPacket, packetLen);
  //int newLen = newPacket.length();
  //Serial.println(newLen);
  Serial.println("Format: Temperature  BPM  SPO2  X-Pos  Y-Pos  Z-Pos  DistressSignal MessageNum");
  //Serial.print("Correct Message: ");
  //Serial.println(radiopacket);
  //itoa(packetnum++, newPacket+36, 10);
  //Serial.print("Packet Length: ");
  //Serial.println(packetLen);
  Serial.print("Sending: "); Serial.println(message.temp);

  
  // Send a message!
  //rf69.send((uint8_t *)message.temp, strlen(message.temp));
  //rf69.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      if (strstr((char *)buf, "DISTRESS")) {
        remoteHelp = 1;
      }
      else if(strstr((char*)buf, "REPORT2")){
        // Send a message!
        Serial.println("Format: Temperature  BPM  SPO2  X-Pos  Y-Pos  Z-Pos  DistressSignal MessageNum");
        Serial.print("Sending: "); Serial.println(message.temp);
        rf69.send((uint8_t *)message.temp, strlen(message.temp));
        rf69.waitPacketSent();
      }
    } else {
      Serial.println("Receive failed");
      remoteHelp = 0;
    }
  } else {
    //Serial.println("No reply, is another RFM69 listening?");
  }
  //newPacket[0] = 0;  //reset the message 
  //radiopacket = "";
  Serial.println();
} 
