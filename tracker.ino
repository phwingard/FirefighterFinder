#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
//#include <MadgwickAHRS.h>
#include "heartRate.h"          //Heart rate calculating algorithm
#include <SPI.h>
#include <RH_RF69.h>
#include "MyPacketStruct.h"
#include <SensorFusion.h>
SF fusion;
float deltat;

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[50]; //infrared LED sensor data
uint16_t redBuffer[50];  //red LED sensor data
#else
uint32_t irBuffer[50]; //infrared LED sensor data
uint32_t redBuffer[50];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read


/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
//Madgwick filter;
//Function Declaration for EMA Filters
float LowEMA(float Aold, float Anew, float beta);
float HighEMA(float Aold, float Anew, float beta);
float Vupdate(float delta, float a, float vold);
float Pupdate(float delta, float v, float pold);

void displaySensorDetails(void)
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

//Thermistor variables
int ThermistorPin = A0;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//Radio variables
#define RF69_FREQ 915.0
//#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      30  //
  #define RFM69_RST     31  // "A"
  #define LED           40  //dont use yet
//#endif
RH_RF69 rf69(RFM69_CS, RFM69_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission

//custom radio packet
MyRadioPacket message;
int remoteHelp = 0;
#define GATEWAYID   1


//pushbutton variables
int helpButton = 32;
int buttonState = 0;


void setup() {

  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  Wire.begin();
  //help button
  pinMode(helpButton, INPUT);
  
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
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

  while (!Serial) {
    delay(1);
  }
  /* Initialise the sensor */
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
  //Setup filter
  delay(500);

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
  
  //pinMode(LED, OUTPUT);
  Serial.println("Done with init.");
  delay(1000);

}

float axold = 0, ayold = 0, azold = 0;
float axro = 0, ayro = 0, azro = 9.8;
float mxold = 0, myold = 0, mzold = 0;
float gravx = 0, gravy = 0, gravz = 9.8;
float vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
float pxo = 0, pyo = 0, pzo = 0;
float vxo = 0, vyo = 0, vzo = 0;


void loop() {

  buttonPushed();
  showDistress();
  //Thermistor code
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  T = (T * 9.0)/ 5.0 + 32.0; 

  if(T > 70){
    remoteHelp = 1;
    showDistress();
  }
  else{
    remoteHelp = 0;
    showDistress();
  }

  //Serial.print("Temperature: "); 
  //Serial.print(T);
  //Serial.println(" F"); 
  
  bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    /*Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);*/
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 50; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 25; i < 50; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      /*Serial.print(F("red="));
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
      Serial.println(validSPO2, DEC);*/
    }

      /*Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);
      Serial.print(F(", SPO2="));
      Serial.println(spo2, DEC);
      */
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  
  float roll, yaw, pitch;
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  
  
  //Getting Sensor Data
  // Get a new sensor event 
  sensors_event_t event;
  gyro.getEvent(&event);
  gx = event.gyro.x; 
  gy = event.gyro.y;
  gz = event.gyro.z;
  
  sensors_event_t aevent, mevent;
  // Get a new sensor event 
  accelmag.getEvent(&aevent, &mevent);
  ax = aevent.acceleration.x;
  ay = aevent.acceleration.y;
  az = aevent.acceleration.z;
  
  //Removes noise from Accelerometer Data
  ax = LowEMA(axold, ax, 0.3);
  ay = LowEMA(ayold, ay, 0.3);
  az = LowEMA(azold, az, 0.3);
  axold = ax;
  ayold = ay;
  azold = az;
  
  mx = aevent.magnetic.x;
  my = aevent.magnetic.y;
  mz = aevent.magnetic.z;
  
  //Removes noise from Magnetometer Data
  mx = LowEMA(mxold, mx, 0.5);
  my = LowEMA(myold, my, 0.5);
  mz = LowEMA(mzold, mz, 0.5);
  mxold = mx;
  myold = my;
  mzold = mz;
  
  //Madgwick filter
  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick
  float deg2rad = 0.0174533;
  
  roll = fusion.getRoll()*deg2rad;
  pitch = fusion.getPitch()*deg2rad;
  yaw = fusion.getYaw()*deg2rad;
  
  
  //Getting the Rotation Vector
  float r11, r12, r13, r21, r22, r23, r31, r32, r33;
  r11 = cos(pitch)*cos(yaw);
  r12 = cos(pitch)*sin(yaw);
  r13 = -sin(pitch);
  r21 = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
  r22 = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
  r23 = sin(roll)*cos(pitch);
  r31 = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
  r32 = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
  r33 = cos(roll)*cos(pitch);
  
  //Rotating Acceleration by Rotation Vector
  float axr, ayr, azr;
  axr = r11*ax + r12*ay + r13*az;
  ayr = r21*ax + r22*ay + r23*az;
  azr = r31*ax + r32*ay + r33*az;
  
  //Low Pass Filter to remove Rapid 'jitter'
  axr = LowEMA(axro, axr, 0.1);
  ayr = LowEMA(ayro, ayr, 0.1);
  azr = LowEMA(azro, azr, 0.1);
  axro = axr;
  ayro = ayr;
  azro = azr;

  //Gravity Adjustment
  gravx = LowEMA(gravx, axr, 0.02);
  gravy = LowEMA(gravy, ayr, 0.02);
  gravz = LowEMA(gravz, azr, 0.02);

 


  //Position Calculation
  vx = Vupdate(deltat, axr-gravx, vx);
  vy = Vupdate(deltat, ayr-gravy, vy);
  vz = Vupdate(deltat, azr-gravz, vz);
  vxo = LowEMA(vxo, vx, 0.002);
  vyo = LowEMA(vyo, vy, 0.002);
  vzo = LowEMA(vzo, vz, 0.002);

  px = Pupdate(deltat, vx-vxo, px);
  py = Pupdate(deltat, vy-vyo, py);
  pz = Pupdate(deltat, vz-vzo, pz);

/*
  Serial.print("X: "); Serial.print(px, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(py, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(pz, 4); Serial.print("  ");  
  Serial.println("m");*/
  
 /* //Print for Accelerometer
  Serial.print("X: "); Serial.print(axr, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ayr, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(azr, 4); Serial.print("  ");
  Serial.println("m/s^2");*/

 //Print for Magnetometer
 /*
  Serial.print("X: "); Serial.print(mx, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(my, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mz, 4); Serial.print("  ");
  Serial.println("uT");
  
  //Print for Gyroscope
  Serial.print("X: "); Serial.print(gx, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gy, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gz, 4); Serial.print("  ");
  Serial.println("rad/sec");*/
  


  dtostrf(T, 5, 0, message.temp);
  dtostrf(heartRate, 5, 0, message.bpm);
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
  rf69.send((uint8_t *)message.temp, strlen(message.temp));

  rf69.waitPacketSent();

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

void buttonPushed(){
  buttonState = digitalRead(helpButton);
  if(buttonState == HIGH){
    //Serial.println("Help Needed!");
    //send message via radio
  }
}

void showDistress(){
  if(buttonState == HIGH || remoteHelp == 1){
    digitalWrite(LED, HIGH);
  }
  else{
    digitalWrite(LED, LOW);
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

float Vupdate(float delta, float a, float vold){
  float vnew = vold + delta*a;
  return vnew;
}

float Pupdate(float delta, float v, float pold){
  float pnew = pold + delta*v;
  return pnew;
}
