#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <MadgwickAHRS.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Madgwick filter;
//Function Declaration for EMA Filters
float LowEMA(float Aold, float Anew, float beta);
float HighEMA(float Aold, float Anew, float beta);

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
  //For Debug only
  Serial.begin(9600);

  /* Wait for the Serial Monitor */
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
  filter.begin(25);

}
// Initialize the EMA filter
float Axnew = 0; float Aynew = 0; float Aznew = 0; //Sets initial acceleration to 0
float Axold = 0; float Ayold = 0; float Azold = 0; //Sets initial acceleration to 0
float Mxnew = 0; float Mynew = 0; float Mznew = 0; //Sets initial Magnetic Reading to 0
float Mxold = 0; float Myold = 0; float Mzold = 0; //Sets initial Magnetic Reading to 0
float Gxnew = 0; float Gynew = 0; float Gznew = 0; //Sets initial Magnetic Reading to 0
float Gxold = 0; float Gyold = 0; float Gzold = 0; //Sets initial Magnetic Reading to 0
float Pnew = 0, Pold = 0, Ynew = 0, Yold = 0, Rnew = 0, Rold = 0;//Sets up filtering for Madgwick Output
float alpha = 0.5; //Sets alpha for EMA filter

void loop() {
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
  
  /*Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
  */

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

  Serial.print("A ");
  Serial.print("X: "); Serial.print(ax, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ay, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(az, 4); Serial.print("  ");
  Serial.println("m/s^2");
}


float LowEMA(float Aold, float Anew, float beta){
  float Afil = beta*Anew + (1-beta)*Aold;
  return Afil;
}

float HighEMA(float Aold, float Anew, float beta){
  float Afil = beta*Anew - (beta*Anew + (1-beta)*Aold);
  return Afil;
}
