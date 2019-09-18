#include <SPIFlash.h>

#include <SPI.h>
#include <RFM69.h>



//Globals and imports
/*#include "MAX30105.h"  //dl these and reverse engineer
#include "heartRate.h"*/
//#include <Wire.h>
//#include <SPI.h>

int threshold = 400;  //threshold for alarm, adjust for later

/*MAX30105 particleSensor;  //object for particle sensor
const byte RATE_SIZE = 4; //for averaging rates
byte rates[RATE_SIZE];  //Array of heart rate easurements
byte rateSpot = 0;
long lastBeat = 0;
float BPM;
int avgBeat;*/

#define NETWORKID 0
#define MYNODEID 1
#define TONODEID 2
#define FREQUENCY RF69_915MHZ
#define USEACK false  //may change later
#define radioLED 9
#define radioGND 8
RFM69 radio;


void setup() {
  //Analog Pins
  /*pinMode(A0, INPUT);  //thermistor
  pinMode(A1, INPUT);  //smoke sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST);  //adjust for other micro
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);  //turns LED to red to show it's running
  */Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready"); 
  pinMode(radioLED,OUTPUT);
  digitalWrite(radioLED,LOW);
  pinMode(radioGND,OUTPUT);
  digitalWrite(radioGND,LOW);
  Serial.begin(9600);  //baud rate of 9600 for serial output
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();  
}

void loop() {
  /*
  //Thermistor code
  double temp, tempC, tempF;
  double codeVal; 
  
  for(int i = 0; i < 100; i++){
    codeVal += analogRead(A0);
  }
  codeVal /= 100;
  
  long seriesRes = 10000;  //change to our desired series resistance
  double thermRes = (1023 - codeVal) / (codeVal * seriesRes);
  temp = 1 / (298.15 + (1 / 3455) * log(thermRes / 10000));  //modify 1st and 2nd values for Beta value and Kelvin value
  tempC = temp + 273.15;
  tempF = tempC * 1.8 + 32;

  Serial.print("Temperature: ");
  Serial.println(tempF);  //output temp in F

  //Smoke sensor code
  int smokeVal = analogRead(A1);
  Serial.print("Smoke level: ");
  Serial.println(smokeVal);

  if(smokeVal > threshold){
    //send signal.  probably will just send raw data and set threshold on hub
  }

  //Oximeter code
  long irValue = particleSensor.getIR();

  if(irValue > 7000){  //threshold for finger, may need to adjust for neck
    if(checkForBeat(irValue) == true){
      long delta = millis() - lastBeat;
      lastBeat = millis();

      BPM = 60 / (delta / 1000.0);
      Serial.print("Beats per Minute: ");
      Serial.println(BPM);
      
      if(BPM < 255 && BPM > 20){
        rates[rateSpot++] = (byte)BPM;
        rateSpot %= RATE_SIZE;
        avgBeat = 0;
        for(byte x = 0; x < RATE_SIZE; x++){
          avgBeat += rates[x];
        }
        avgBeat /= RATE_SIZE;
        Serial.print("Average beat: ");
        Serial.println(avgBeat);
      }
      
    }
  Serial.print("Oxygen level: ");
  Serial.println(irValue);    
  }
*/

  //radio code
  static char sendBuffer[62];
  static int sendLength = 0;

  if (Serial.available() > 0){
    char input = Serial.read();

    if(input != '\r'){
      sendBuffer[sendLength] = input;
      sendLength++;
    }

    if((input == '\r') || (input == '\n') || (sendLength == 61)){
      Serial.print("Sending to node: ");
      Serial.print(TONODEID, DEC);
      Serial.print(", message [");
      for(byte i = 0; i < sendLength; i++){
        Serial.print(sendBuffer[i]);
      }
      Serial.print("]");

      if(USEACK){
        if(radio.sendWithRetry(TONODEID, sendBuffer, sendLength)){
          Serial.println("ACK received!");
        }
        else{
          Serial.println("No ACK received.");
        }
      }

      else{
        radio.send(TONODEID, sendBuffer, sendLength);
      }
      sendLength = 0;
      Blink(radioLED,10);
      
    }
    //add receiver stuff later
  }
   

}

void Blink(byte PIN, int DELAY_MS){
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
}
