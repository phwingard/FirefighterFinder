
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
//#include <MadgwickAHRS.h>
#include <SensorFusion.h>
SF fusion;
float deltat;

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

void setup() {
  // put your setup code here, to run once:
  //For Debug only
  Serial.begin(115200);

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

}
float axold = 0, ayold = 0, azold = 0;
float axro = 0, ayro = 0, azro = 9.8;
float mxold = 0, myold = 0, mzold = 0;
float gravx = 0, gravy = 0, gravz = 9.8;
float vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
float pxo = 0, pyo = 0, pzo = 0;
float vxo = 0, vyo = 0, vzo = 0;

void loop() {
  // put your main code here, to run repeatedly:
  float roll, yaw, pitch;
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  

  //Getting Sensor Data
  /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);
  gx = event.gyro.x; 
  gy = event.gyro.y;
  gz = event.gyro.z;

  sensors_event_t aevent, mevent;
  /* Get a new sensor event */
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
 

  Serial.print("X: "); Serial.print(px, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(py, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(pz, 4); Serial.print("  ");  
  Serial.println("m");
  
 /* //Print for Accelerometer
  Serial.print("X: "); Serial.print(axr, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ayr, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(azr, 4); Serial.print("  ");
  Serial.println("m/s^2");*/

/* //Print for Magnetometer
  Serial.print("X: "); Serial.print(mx, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(my, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mz, 4); Serial.print("  ");
  Serial.println("uT");
  
  //Print for Gyroscope
  Serial.print("X: "); Serial.print(gx, 4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gy, 4); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gz, 4); Serial.print("  ");
  Serial.println("rad/sec");
  */
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
