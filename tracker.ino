//#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <SPI.h>
#include <RH_RF69.h>
#include "MyPacketStruct.h"
#include <MadgwickAHRS.h>
#include "spo2_algorithm.h"

//Radio variables
#define RF69_FREQ 915.0
//#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     2  // 
  #define RFM69_CS      30  //
  #define RFM69_RST     31  // "A"
  #define LED           13  //dont use yet
//#endif
RH_RF69 rf69(RFM69_CS, RFM69_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission

//custom radio packet
MyRadioPacket message;

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
float alpha2 = 0.3;
#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

/*
//Smoke sensor variables
int smokeA1 = A1;
int sensorThres = 400;  //threshold value
int smokeD7 = 7;*/

//IMU variables
/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Madgwick filter;
float LowEMA(float Aold, float Anew, float beta);
float HighEMA(float Aold, float Anew, float beta);
// Initialize the EMA filter
float Axnew = 0; float Aynew = 0; float Aznew = 0; //Sets initial acceleration to 0
float Axold = 0; float Ayold = 0; float Azold = 0; //Sets initial acceleration to 0
float Mxnew = 0; float Mynew = 0; float Mznew = 0; //Sets initial Magnetic Reading to 0
float Mxold = 0; float Myold = 0; float Mzold = 0; //Sets initial Magnetic Reading to 0
float Gxnew = 0; float Gynew = 0; float Gznew = 0; //Sets initial Magnetic Reading to 0
float Gxold = 0; float Gyold = 0; float Gzold = 0; //Sets initial Magnetic Reading to 0
float Pnew = 0, Pold = 0, Ynew = 0, Yold = 0, Rnew = 0, Rold = 0;//Sets up filtering for Madgwick Output
float alpha = 0.5; //Sets alpha for EMA filter


//pushbutton variables
int helpButton = 32;
int buttonState = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //help button
  pinMode(helpButton, INPUT);
  
  //Oximeter
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  //Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  //Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings


  /*//smoke sensor
  pinMode(smokeA1, INPUT);
  pinMode(smokeD7, INPUT);
*/
  //IMU
  if(!gyro.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    //while(1);
  }
  if(!accelmag.begin())
  {
    /* There was a problem detecting the FXAS21002C ... check your connections */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    //while(1);
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
    //Serial.println("RFM69 radio init failed");
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
  
  pinMode(LED, OUTPUT);

  //Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  
}

void loop() {

  buttonPushed();
  
  //Thermistor code
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  T = (T * 9.0)/ 5.0 + 32.0; 

  Serial.print("Temperature: "); 
  Serial.print(T);
  Serial.println(" F"); 
  dtostrf(T, 7, 3, message.temp);
  
  //Oximeter code
  bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  

/*
  //Smoke sensor code
  int analogSensor = analogRead(smokeA1);
  //Serial.print("Analog Value: ");
  //Serial.println(analogSensor);
  if (analogSensor > sensorThres)
  {
    Serial.println("Level reached.");
  }*/
  
  //IMU code
float roll, pitch, heading;
  unsigned long microsNow;

  //Code from MadgwickAHRS sample
  microsNow = micros();
    /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);
  /*High Pass Filter for Gyro events Excludes Drift*/
  Gxold = Gxnew; Gyold = Gynew; Gzold = Gznew;
  Gxnew = HighEMA(Gxold, event.gyro.x, 0.9);
  Gynew = HighEMA(Gyold, event.gyro.y, 0.9);
  Gznew = HighEMA(Gzold, event.gyro.z, 0.9);
  
  /* Display the results (speed is measured in rad/s) 
  Serial.print("X: "); Serial.print(Gxnew); Serial.print("  ");
  Serial.print("Y: "); Serial.print(Gynew); Serial.print("  ");
  Serial.print("Z: "); Serial.print(Gznew); Serial.print("  ");
  Serial.println("rad/s ");
  delay(10); */

  //Accel and Magnetometer Sample code
  sensors_event_t aevent, mevent;

  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
  /*Low Pass Filter for Accel events*/
  Axold = Axnew; Ayold = Aynew; Azold = Aznew;
  Axnew = alpha*aevent.acceleration.x + (1-alpha)*Axold;
  Aynew = alpha*aevent.acceleration.y + (1-alpha)*Ayold;
  Aznew = alpha*aevent.acceleration.z + (1-alpha)*Azold;

  /* Display the accel results (acceleration is measured in m/s^2) */
  /*Serial.print("A ");
  Serial.print("X: "); Serial.print(Axnew, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(Aynew, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(Aznew, 4); Serial.print("  ");
  Serial.println("m/s^2");*/
  
  /*Low Pass Filter for magnetic events*/
  Mxold = Mxnew; Myold = Mynew; Mzold = Mznew;
  Mxnew = alpha*aevent.magnetic.x + (1-alpha)*Mxold;
  Mynew = alpha*aevent.magnetic.y + (1-alpha)*Myold;
  Mznew = alpha*aevent.magnetic.z + (1-alpha)*Mzold;
  /* Display the mag results (mag data is in uTesla) */
  /*Serial.print("M ");
  Serial.print("X: "); Serial.print(Mxnew, 1); Serial.print("  ");
  Serial.print("Y: "); Serial.print(Mynew, 1); Serial.print("  ");
  Serial.print("Z: "); Serial.print(Mznew, 1); Serial.print("  ");
  Serial.println("uT");
  Serial.println("");*/
  //Madgwick filter Implementation
  filter.updateIMU(Gxnew, Gynew, Gznew, Axnew, Aynew, Aznew);
  roll = filter.getRollRadians(); pitch = filter.getPitchRadians(); heading = filter.getYawRadians();
  //Filtering Noise off of Madgwick Output
  /*Low Pass Filter for Direction*/
  Yold = Ynew; Pold = Pnew; Rold = Rnew;
  Rnew = LowEMA(Rold, roll, 0.5);
  Ynew = LowEMA(Yold, heading, 0.5);
  Pnew = LowEMA(Pold, pitch, 0.5);
  
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
  Serial.println();
  delay(500);
  

  //Computing Actual Acceleration in regards to orientation
  float R11, R12, R13, R21, R22, R23, R31, R32, R33;
  R11 = cos(Pnew)*cos(Ynew);
  R12 = cos(Pnew)*sin(Ynew);
  R13 = -sin(Ynew);
  R21 = sin(Rnew)*sin(Pnew)*cos(Ynew)-cos(Rnew)*sin(Ynew);
  R22 = sin(Rnew)*sin(Pnew)*sin(Ynew)+cos(Rnew)*cos(Ynew);
  R23 = sin(Rnew)*cos(Ynew);
  R31 = cos(Rnew)*sin(Pnew)*cos(Ynew)-sin(Rnew)*sin(Ynew);
  R32 = cos(Rnew)*sin(Pnew)*sin(Ynew) - sin(Rnew)*cos(Ynew);
  R33 = cos(Rnew)*cos(Ynew);
  float ax, ay, az;
  
  ax = R11*Axnew + R12*Aynew + R13*Aznew;
  ay = R21*Axnew + R22*Aynew + R23*Aznew;
  az = R31*Axnew + R32*Aynew + R33*Aznew;

  /*Serial.print("A ");
  Serial.print("X: "); Serial.print(ax, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ay, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(az, 4); Serial.print("  ");
  Serial.println("m/s^2");
  Serial.println();*/
  
  //radio code
  char radiopacket[8];
  //Serial.println(message.temp);
  memcpy(radiopacket, message.temp, sizeof(radiopacket));  //modify this to send a packet for each peripheral
  //float radiopacket = T;  
    itoa(packetnum++, radiopacket+13, 10);
    //Serial.print("Sending "); Serial.println(radiopacket);
    
    // Send a message!
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();
  
    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  
    if (rf69.waitAvailableTimeout(500))  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
        //Serial.print("Got a reply: ");
        //Serial.println((char*)buf);
        Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
      } else {
        //Serial.println("Receive failed");
      }
    } else {
      //Serial.println("No reply, is another RFM69 listening?");
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

float LowEMA(float Aold, float Anew, float beta){
  float Afil = beta*Anew + (1-beta)*Aold;
  return Afil;
}

float HighEMA(float Aold, float Anew, float beta){
  float Afil = beta*Anew - (beta*Anew + (1-beta)*Aold);
  return Afil;
}

void buttonPushed(){
  buttonState = digitalRead(helpButton);
  if(buttonState == HIGH){
    Serial.println("Help Needed!");
    //send message via radio
  }
}

void checkDistressFromHub(){

}
