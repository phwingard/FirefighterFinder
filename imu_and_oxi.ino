//Copyright 2019 Michael White
//License Notice at Notice.md

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <MadgwickAHRS.h>
#include "heartRate.h"          //Heart rate calculating algorithm
//#include <SPI.h>
//#include <RH_RF69.h>
//#include "MyPacketStruct.h"
//#include <math.h>
#include <SensorFusion.h>
#include <MatrixMath.h>
#define N (3)
float g = 9.80665;
double aG0 = 0;
double aG1 = 0;
double aG2 = 0;

//#define PI 3.1415926535897932384626433832795

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

SF fusion;
float deltat;

MAX30105 particleSensor;
const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
float filteredBPM = 0;
float alpha = 0.3;


Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Madgwick filter;

float mag_offsets[3]            = { -14.05F, 110.04F, -131.84F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.004, -0.029, 0.072 },
                                    { 0.029, 1.284, -0.033 },
                                    { 0.072, -0.033, 0.782 } }; 

float mag_field_strength        = 25.72F;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  Serial.println("Place your finger on the sensor and wait a few seconds for a reading.");

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

int t = 0;
void loop() {
  t = millis();
  // put your main code here, to run repeatedly:
long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
 //Serial.println(irValue);           
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
      /*
      Serial.print("BPM: ");
      Serial.print(beatsPerMinute);
      Serial.print("   Filtered BPM: ");
      Serial.print(filteredBPM); 
      Serial.print("   Avg BPM: ");
      Serial.println(beatAvg);*/
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
  /*
 Serial.println("Low Pass Filtered (Accel): ");
 Serial.print(ax);  Serial.print(" "); Serial.print(ay); Serial.print(" ");  Serial.println(az); 

 Serial.println("Low Pass Filtered (Magneto): ");
 Serial.print(mx);  Serial.print(" "); Serial.print(my); Serial.print(" ");  Serial.println(mz); 

 Serial.println("Low Pass Filtered (Gyro): ");
 Serial.print(gx);  Serial.print(" "); Serial.print(gy); Serial.print(" ");  Serial.println(gz); 

*/
//Madgwick filter
  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  /*
 Serial.println("Madgwick Filtered: ");
 Serial.print(ax);  Serial.print(" "); Serial.print(ay); Serial.print(" ");  Serial.println(az); 
*/

 Serial.println("Madgwick Filtered (Accel): ");
 Serial.print(ax);  Serial.print(" "); Serial.print(ay); Serial.print(" ");  Serial.println(az); 

 Serial.println("Madgwick Filtered (Magneto): ");
 Serial.print(mx);  Serial.print(" "); Serial.print(my); Serial.print(" ");  Serial.println(mz); 

 Serial.println("Madgwick Filtered (Gyro): ");
 Serial.print(gx);  Serial.print(" "); Serial.print(gy); Serial.print(" ");  Serial.println(gz); 

  
  float deg2rad = 0.0174533;

  roll = fusion.getRoll()*deg2rad;
  pitch = fusion.getPitch()*deg2rad;
  yaw = fusion.getYaw()*deg2rad; //- 0.00061;
  
  Serial.print("Yaw: "); 
  Serial.print(yaw, 4); Serial.print("  ");
  Serial.print("Pitch: "); Serial.print(pitch, 4); Serial.print("  ");
  Serial.print("Roll: "); Serial.print(roll, 4); Serial.print("  ");  
  Serial.println("rads");
/*
  altYaw = /*180 * atan(az / sqrt(ax*ax + az*az)) // float(M_PI)/;
  Serial.print("Alt Yaw:");
  Serial.println(altYaw);*/


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
/*
  grav[0][0] = 0;
  grav[1][0] = 0;
  grav[2][0] = g;*/
 //Matrix.Print((mtx_type*)grav, N, N, "grav");
 //Matrix.Print((mtx_type*)orientation, N, N, "orientation");

 Matrix.Copy((mtx_type*)orientation, N, N, (mtx_type*)oriInv);
 Matrix.Invert((mtx_type*)oriInv, N);
 //Matrix.Print((mtx_type*)orientation, N, N, "inverted");
 //Matrix.Print((mtx_type*)orientation, N, N, "orientation");

 //Matrix.Multiply((mtx_type*)oriInv, (mtx_type*)grav, N, N, 1, (mtx_type*)accelG);
 //Matrix.Print((mtx_type*)accelG, N, N, "accel G");

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


 Serial.println("Gravity Correction Matrix: ");
 Serial.print(aG0);  Serial.print(" "); Serial.print(aG1); Serial.print(" ");  Serial.println(aG2); 


 //Serial.print(aG0);  Serial.print(aG1);  Serial.println(aG2);
 //Serial.println(aG2);
 
 Vu = Vupdate(deltat, alx, c0);
 Vv = Vupdate(deltat, aly, c1);
 Vw = Vupdate(deltat, alz, c2);
 /*c0 = LowEMA(c0, Vu, 0.002);
 c1 = LowEMA(c1, Vv, 0.002);
 c2 = LowEMA(c2, Vw, 0.002);*/
 c0 = Vu;
 c1 = Vv;
 c2 = Vw;
 //Serial.print(Vu);  Serial.print(Vv);  Serial.println(Vw); 
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

  
  Serial.print("Veloctiy Count: ");
  Serial.println(vCount);
/*
 if(alx < 0.001){
  vCount++;
  Serial.print("Veloctiy Count: ");
  Serial.println(vCount);
 }
 else{
  vCount = 0;
 }

 if(aly < 0.001){
  vCount++;
  Serial.print("Veloctiy Count: ");
  Serial.println(vCount);
 }
 else{
  vCount = 0;
 }

  if(alz < 0.001){
  vCount++;
  Serial.print("Veloctiy Count: ");
  Serial.println(vCount);
 }
 else{
  vCount = 0;
 }
 
 if(vCount > 100){
  Vu = 0;
  Vv = 0;
  Vw = 0;
  vCount = 0;
 }*/



 
/*
 if ((vy > 12.5) || (-vy > 12.5) || (vy > 8.3) || (-vy > 8.3)){
  vy = 0;
 }

 if ((vz > 12.5) || (-vz > 12.5) || (vy > 8.3) || (-vy > 8.3)){
  vz = 0;
 }*/




//centripital
 acx = -gz*Vv + gy*Vw;
 acy = gz*Vu - gx*Vw;
 acz = -gy*Vu + gx*Vv;
 acxo = acx;
 acyo = acy;
 aczo = acz;
 
 Serial.println("Linear Velocity: ");
 Serial.print(Vu);  Serial.print(" "); Serial.print(Vv); Serial.print(" ");  Serial.println(Vw); 

 Serial.println("Old Linear Velocity: ");
 Serial.print(c0);  Serial.print(" "); Serial.print(c1); Serial.print(" ");  Serial.println(c2); 

/*
 alx = ax - acx;
 aly = ay - acy;
 alz = az - acz;*/
 //Serial.print(alx);  Serial.print(" "); Serial.print(aly); Serial.print(" ");  Serial.println(alz); 

 /*
 Serial.println("Gravity Corrected: ");
 Serial.print(alx);  Serial.print(" "); Serial.print(aly); Serial.print(" ");  Serial.println(alz); 
*/










 

 //Rotating Acceleration by Rotation Vector
 float axr, ayr, azr;
 axr = r11*alx + r12*aly + r13*alz;
 ayr = r21*alx + r22*aly + r23*alz;
 azr = r31*alx + r32*aly + r33*alz;

/*
 if ((axr > 3.5 ) || (axr < 0.7)){
  axr = 0;
 }

 if ((ayr > 3.5 ) || (ayr < 0.7)){
  ayr = 0;
 }

  if ((azr > 3.5 ) || (azr < 0.7)){
  azr = 0;
 }*/
/*
 Serial.println("Rotated Back to Ref Frame: ");
 Serial.print(axr);  Serial.print(" "); Serial.print(ayr); Serial.print(" ");  Serial.println(azr); 
*/

//Low Pass Filter to remove Rapid 'jitter'

 alx = LowEMA(axro, axr, 0.1);
 aly = LowEMA(ayro, ayr, 0.1);
 alz = LowEMA(azro, azr, 0.1);
 axro = alx;
 ayro = aly;
 azro = alz;

//Gravity Adjustment

 gravx = LowEMA(gravx, alx, 0.05);
 gravy = LowEMA(gravy, aly, 0.05);
 gravz = LowEMA(gravz, alz, 0.05);
/*
 Serial.println("Gravity Vector: ");
 Serial.print(gravx);  Serial.print(" "); Serial.print(gravy); Serial.print(" ");  Serial.println(gravz); 
*/

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
 /*float abv = vx*vx + vy*vy +  vz*vz;
 
 /*
 if ((abv > 12.5 ) || (abv < 0.025)){
  vx = 0;
  vy = 0;
  vz = 0;
 }
/*
 if ((vy > 12.5) || (-vy > 12.5) || (vy > 8.3) || (-vy > 8.3)){
  vy = 0;
 }

 if ((vz > 12.5) || (-vz > 12.5) || (vy > 8.3) || (-vy > 8.3)){
  vz = 0;
 }*/


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
  
Serial.println(" ");
  
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
