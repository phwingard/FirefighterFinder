//Copyright 2019 Patrick Wingard
//License Notice at Notice.md

#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0

#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13

RH_RF69 rf69(RFM69_CS, RFM69_INT);


int packet[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
char msgbuf[6];
char* msg;
bool is_ready = true;
int id = 1;
String report;
char* rep;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);     
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
  Serial.println("RFM69 radio init OK!");
  
  // No encryption
  
  rf69.setTxPower(20, true);

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

}

void loop() {
  if (id == 1) {
      report = "REPORT1";
  }
  else {
    report = "REPORT2";
  }
  report.toCharArray(rep, 8);
  rf69.send((uint8_t *)rep, strlen(rep));
  rf69.waitPacketSent();
  if (rf69.available()) {
    is_ready = true;
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;

      msg = (char*)buf;
      String str(msg);
      //Serial.println(msg);
      packet[0] = id;
      for (int i = 1; i < 8; i++) {
        String ms = str.substring((i*5), (i*5) + 5);
        //strncpy(msgbuf, msg + (i*5), 5);
        //msgbuf[6] = '\0';
        ms.trim();
        packet[i] = ms.toInt();
        //Serial.println(ms);
      }

      // Send a reply!
      uint8_t data[] = "Thank you for your service";
      rf69.send(data, sizeof(data));
      rf69.waitPacketSent();
      //Serial.println("Sent a reply");
      //Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
    
    } else {
      Serial.println("Receive failed");
    }

    for (int i = 0; i < 8; i++) {
      Serial.print(packet[i]);
      Serial.print('\t');
    }
    Serial.print('\n');
  }

  
  if (Serial.available()){
    char val = Serial.read();
    if (val == 'D') {
      uint8_t dist[] = "DISTRESS";
      rf69.send(dist, sizeof(dist));
      rf69.waitPacketSent();
    }
    if (val == 'O') {
      //uint8_t dist[] = "DISTRESS2";
      //rf69.send(dist, sizeof(dist));
      //rf69.waitPacketSent();
    }
  }
  if (id == 1) {
    id = 2; 
  }
  else {
    id = 1;
  }

}
