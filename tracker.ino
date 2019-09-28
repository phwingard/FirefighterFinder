#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <SPI.h>
#include <RH_RF69.h>

//Radio variables
#define RF69_FREQ 915.0
//#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     2  // 
  #define RFM69_CS      28  //
  #define RFM69_RST     30  // "A"
  #define LED           13
//#endif
RH_RF69 rf69(RFM69_CS, RFM69_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission

//Thermistor variables
int ThermistorPin = A0;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//Oximeter variables
MAX30105 particleSensor;
const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
float filteredBPM = 0;
float alpha = 0.3;
/*
//Smoke sensor variables
int smokeA1 = A1;
int sensorThres = 400;  //threshold value
int smokeD7 = 7;*/

//IMU variables
/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

//pushbutton variables
int helpButton = 32;


void setup() {
  Serial.begin(115200);

  //help button
  pinMode(helpButton, INPUT);
  
  //Oximeter
  particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running

  /*//smoke sensor
  pinMode(smokeA1, INPUT);
  pinMode(smokeD7, INPUT);
*/
  //IMU
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }
  if(!accelmag.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while(1);
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();

  //radio setup
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
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  
}

void loop() {
  //Thermistor code
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  T = (T * 9.0)/ 5.0 + 32.0; 

  //Serial.print("Temperature: "); 
  //Serial.print(T);
  //Serial.println(" F"); 

  //Oximeter code
  long irValue = particleSensor.getIR(); 
  if(irValue > 7000){           
    if (checkForBeat(irValue) == true){
      long delta = millis() - lastBeat;                   //Measure duration between two beats
      lastBeat = millis();  
      beatsPerMinute = 60 / (delta / 1000.0);           //Calculating the BPM
      filteredBPM = (alpha * beatsPerMinute) + ((1 - alpha) * filteredBPM);
      if (beatsPerMinute < 255 && beatsPerMinute > 20)               //To calculate the average we strore some values (4) then do some math to calculate the average
      {  
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;         
        //Serial.print("BPM: ");
        //Serial.print(beatsPerMinute);
        //Serial.print("   Filtered BPM: ");
        //Serial.print(filteredBPM); 
        //Serial.print("   Avg BPM: ");
        //Serial.print(beatAvg);
      }
    }
  }

  //Smoke sensor code
  int analogSensor = analogRead(smokeA1);
  //Serial.print("Analog Value: ");
  //Serial.println(analogSensor);
  if (analogSensor > sensorThres)
  {
    Serial.println("Level reached.");
  }
  
  //IMU code
  sensors_event_t event;
  gyro.getEvent(&event);

  /* Display the results (speed is measured in rad/s) */
  //Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");
  //Serial.println("rad/s ");
  //delay(500);

  //Accel and Magnetometer Sample code
  sensors_event_t aevent, mevent;

  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);

  /* Display the accel results (acceleration is measured in m/s^2) */
  //Serial.print("A ");
  //Serial.print("X: "); Serial.print(aevent.acceleration.x, 4); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(aevent.acceleration.y, 4); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(aevent.acceleration.z, 4); Serial.print("  ");
  //Serial.println("m/s^2");

  /* Display the mag results (mag data is in uTesla) */
  //Serial.print("M ");
  //Serial.print("X: "); Serial.print(mevent.magnetic.x, 1); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(mevent.magnetic.y, 1); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(mevent.magnetic.z, 1); Serial.print("  ");
  //Serial.println("uT");
  //Serial.println("");

  //radio code
  char radiopacket[20] = "Hello World #";  //modify this to send a packet for each peripheral
    itoa(packetnum++, radiopacket+13, 10);
    Serial.print("Sending "); Serial.println(radiopacket);
    
    // Send a message!
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();
  
    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  
    if (rf69.waitAvailableTimeout(500))  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
        Serial.print("Got a reply: ");
        Serial.println((char*)buf);
        Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
      } else {
        Serial.println("Receive failed");
      }
    } else {
      Serial.println("No reply, is another RFM69 listening?");
    }
}

void displaySensorDetails(void)  //displays IMU details
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    0x"); Serial.println(sensor.sensor_id, HEX);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
