//Copyright 2019 Michael White
//License Notice at Notice.md

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include "heartRate.h"          //Heart rate calculating algorithm
#include <SPI.h>
#include <RH_RF69.h>
#include "MyPacketStruct.h"
#include <SensorFusion.h>
SF fusion;
float deltat;

#include <MatrixMath.h>
#define N (3)
float g = 9.80665;
double aG0 = 0;
double aG1 = 0;
double aG2 = 0;

double Vu = 0;
double Vv = 0;
double Vw = 0;
float c0 = 0;
float c1 = 0;
float c2 = 0;

float acx = 0;
float acy = 0;
float acz = 0;

float acxo = 0;
float acyo = 0;
float aczo = 0;

float alx = 0;
float aly = 0;
float alz = 0;

int vCount = 0;

mtx_type grav[1][N];
mtx_type orientation[N][N];
mtx_type oriInv[N][N];
mtx_type accelG[1][N];

int t = 0;
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
float filteredBPM = 0;
float alpha = 0.3;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
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
int ThermistorPin = A9;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float t1 = 1.009249522e-03, t2 = 2.378405444e-04, t3 = 2.019202697e-07;


#define RF69_FREQ 915.0

  #define RFM69_INT     9  // 
  #define RFM69_CS      10  //
  #define RFM69_RST     8  // "A"

RH_RF69 rf69(RFM69_CS, RFM69_INT);
int16_t packetnum = 0;  // packet counter, we increment per xmission

//custom radio packet
MyRadioPacket message;
int remoteHelp = 0;
#define GATEWAYID   1


//pushbutton variables
int helpButton = 7;
int buttonState = 0;
int LED = 6;

void setup() {

  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  Wire.begin();
  //help button
  pinMode(helpButton, INPUT);  
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  Serial.println("Place your finger on the sensor and wait a few seconds for a reading.");


  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
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
    while (1);
  }
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    //Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.println("Done with init.");
  delay(1000);

}

float axold = 0, ayold = 0, azold = 0;
float axro = 0, ayro = 0, azro = 10.0; //9.8;
float mxold = 0, myold = 0, mzold = 0;
float gravx = 0, gravy = 0, gravz = 10.0;//9.8;
float vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
float pxo = 0, pyo = 0, pzo = 0;
float vxo = 0, vyo = 0, vzo = 0;
float roll, yaw, pitch;
float gx, gy, gz, ax, ay, az, mx, my, mz;

float altYaw = 0;


void loop() {

  buttonPushed();
  showDistress();
  //Thermistor code
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (t1 + t2*logR2 + t3*logR2*logR2*logR2));
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


  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not         
  //If a finger is detected
  if (checkForBeat(irValue) == true)                        //If a heart beat is detected
  {                                         //Deactivate the buzzer to have the effect of a "bip"
    //We sensed a beat!
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

      Serial.print("IRVal: ");
      Serial.print(irValue);
      Serial.print("BPM: ");
      Serial.print(beatsPerMinute);
      Serial.print("   Filtered BPM: ");
      Serial.print(filteredBPM); 
      Serial.print("   Avg BPM: ");
      Serial.println(beatAvg);
    }
  }
 
  //Getting Sensor Data
  /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);
  gx = event.gyro.x;// - 0.03; 
  gy = event.gyro.y;// - 0.02;
  gz = event.gyro.z;// - 0.025;


  sensors_event_t aevent, mevent;
  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
  ax = aevent.acceleration.x - 0.276667;
  ay = aevent.acceleration.y + 0.26 - .16;
  az = aevent.acceleration.z - 0.5;

  //Removes noise from Accelerometer Data
  ax = LowEMA(axold, ax, 0.3);
  ay = LowEMA(ayold, ay, 0.3);
  az = LowEMA(azold, az, 0.3);
  axold = ax;
  ayold = ay;
  azold = az;

  mx = aevent.magnetic.x + 14.05;
  my = aevent.magnetic.y - 110.04;
  mz = aevent.magnetic.z + 131.84;
  
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
  yaw = fusion.getYaw()*deg2rad; //- 0.00061;



 //Getting the Rotation Vector
 float r11, r12, r13, r21, r22, r23, r31, r32, r33;
 r11 = cos(pitch)*cos(yaw);
 //Serial.println(r11);
 r12 = cos(pitch)*sin(yaw);
 r13 = -sin(pitch);
 r21 = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
 r22 = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
 r23 = sin(roll)*cos(pitch);
 r31 = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
 r32 = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
 r33 = cos(roll)*cos(pitch);

 orientation[0][0] = r11;
 orientation[0][1] = r12;
 orientation[0][2] = r13;
 orientation[1][0] = r21;
 orientation[1][1] = r22;
 orientation[1][2] = r23;
 orientation[2][0] = r31;
 orientation[2][1] = r32;
 orientation[2][2] = r33;


 Matrix.Copy((mtx_type*)orientation, N, N, (mtx_type*)oriInv);
 Matrix.Invert((mtx_type*)oriInv, N);


 float det = oriInv[0][0]*(oriInv[1][1]*oriInv[2][2] - oriInv[1][2]*oriInv[2][1]) - 
             oriInv[0][1]*(oriInv[1][0]*oriInv[2][2] - oriInv[1][2]*oriInv[2][0]) +
             oriInv[0][2]*(oriInv[1][0]*oriInv[2][1] - oriInv[1][1]*oriInv[2][0]);

 //Graviy adjustment
 aG0 = oriInv[0][2] / det * g;
 aG1 = oriInv[1][2] / det * g;
 aG2 = oriInv[2][2] / det * g;

 
//linear acceleration
 alx = ax - aG0 - acxo;
 aly = ay - aG1 - acyo;
 alz = az - aG2 - aczo;

 
 Vu = Vupdate(deltat, alx, c0);
 Vv = Vupdate(deltat, aly, c1);
 Vw = Vupdate(deltat, alz, c2);

 c0 = Vu;
 c1 = Vv;
 c2 = Vw;
 //u=x v=y w=z

 
//Velocity Cutoffs
 float abv = Vu*Vu + Vv*Vv + Vw*Vw;

 if ((abv > 12.5*12.5 ) || (abv < 0.1*0.1)){
  Vu = 0;
  Vv = 0;
  Vw = 0;
 }


 float aba = alx*alx + aly*aly + alz*alz;
  
 if ((aba > 12.5*12.5 ) || (aba < 0.1*0.1)){
  Vu = 0;
  Vv = 0;
  Vw = 0;
  vCount++;  
 }
 else{
  vCount = 0;
 }

//centripital
 acx = -gz*Vv + gy*Vw;
 acy = gz*Vu - gx*Vw;
 acz = -gy*Vu + gx*Vv;
 acxo = acx;
 acyo = acy;
 aczo = acz;

/*
 alx = ax - acx;
 aly = ay - acy;
 alz = az - acz;*/
 

 //Rotating Acceleration by Rotation Vector
 float axr, ayr, azr;
 axr = r11*alx + r12*aly + r13*alz;
 ayr = r21*alx + r22*aly + r23*alz;
 azr = r31*alx + r32*aly + r33*alz;


//Low Pass Filter to remove Rapid 'jitter'

 alx = LowEMA(axro, axr, 0.1);
 aly = LowEMA(ayro, ayr, 0.1);
 alz = LowEMA(azro, azr, 0.1);
 axro = alx;
 ayro = aly;
 azro = alz;

//Gravity Adjustment

 gravx = LowEMA(gravx, alx, 0.0002);
 gravy = LowEMA(gravy, aly, 0.0002);
 gravz = LowEMA(gravz, alz, 0.0002);


 //Serial.println(t);
 if(t < 6000){
  vx = 0;
  vy = 0;
  vz = 0;
  vxo = 0;
  vyo = 0;
  vzo = 0; 
  px = 0;
  py = 0;
  pz = 0;
 }


 //Position Calculation
 vx = Vupdate(deltat, alx - gravx, vx);
 vy = Vupdate(deltat, aly - gravy, vy);
 vz = Vupdate(deltat, alz - gravz, vz);
 
 
 vxo = LowEMA(vxo, vx, 0.002);
 vyo = LowEMA(vyo, vy, 0.002);
 vzo = LowEMA(vzo, vz, 0.002);


 px = Pupdate(deltat, vx-vxo, px);
 py = Pupdate(deltat, vy-vyo, py);
 pz = Pupdate(deltat, vz-vzo, pz);
 

  Serial.print("X: "); Serial.print(px, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(py, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(pz, 4); Serial.print("  ");  
  Serial.println("m");
  
  //Print for Accelerometer
  /*
  Serial.print("X: "); Serial.print(axr, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ayr, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(azr, 4); Serial.print("  ");
  Serial.println("m/s^2");*/

 //Print for Magnetometer
  /*Serial.print("X: "); Serial.print(mx, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(my, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mz, 4); Serial.print("  ");
  Serial.println("uT");*/
  
  //Print for Gyroscope
  /*Serial.print("X: "); Serial.print(gx, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gy, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gz, 4); Serial.print("  ");
  Serial.println("rad/sec");*/
  
  


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
