//Copyright 2019 Michael White
//License Notice at Notice.md

#ifndef MyPacketStruct_h
#define MyPacketStruct_h

struct myRadioPacket{
  char temp[5];
  char bpm[5];
  char spO2[5];
  char posX[5];
  char posY[5];
  char posZ[5];
  char helpButton[5];
  char radioID[5];


};

typedef struct myRadioPacket MyRadioPacket;  //size of 28 bytes

#endif


/*
  long irValue = particleSensor.getIR(); 
  if(irValue > 7000){  
    dtostrf(irValue, 7, 3, message.oxy);         
    if (checkForBeat(irValue) == true){
      long delta = millis() - lastBeat;                   //Measure duration between two beats
      lastBeat = millis();  
      beatsPerMinute = 60 / (delta / 1000.0);           //Calculating the BPM
      filteredBPM = (alpha2 * beatsPerMinute) + ((1 - alpha2) * filteredBPM);
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
        Serial.print(filteredBPM); 
        Serial.print("   Avg BPM: ");
        //Serial.print(beatAvg);
        dtostrf(filteredBPM, 7, 3, message.bpm);
      }
    }
  }
  
    //Oximeter code
  bufferLength = 50; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    //while (particleSensor.available() == false) //do we have new data?
    particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

    //Serial.print(F("red="));
    //Serial.print(redBuffer[i], DEC);
    //Serial.print(F(", ir="));
    //Serial.println(irBuffer[i], DEC);

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
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      //Serial.print(F(", HRvalid="));
      //Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.println(spo2, DEC);

    }

      //Serial.print(F(", SPO2Valid="));
      //Serial.println(validSPO2, DEC);
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
  }
  
  //IMU code
  float roll, pitch, heading;
  unsigned long microsNow;

  //Code from MadgwickAHRS sample
  microsNow = micros();
    /* Get a new sensor event 
  sensors_event_t event;
  gyro.getEvent(&event);
  /*High Pass Filter for Gyro events Excludes Drift
  Gxold = Gxnew; Gyold = Gynew; Gzold = Gznew;
  Gxnew = HighEMA(Gxold, event.gyro.x, 0.9);
  Gynew = HighEMA(Gyold, event.gyro.y, 0.9);
  Gznew = HighEMA(Gzold, event.gyro.z, 0.9);
  
  /* Display the results (speed is measured in rad/s) 
  Serial.print("X: "); Serial.print(Gxnew); Serial.print("  ");
  Serial.print("Y: "); Serial.print(Gynew); Serial.print("  ");
  Serial.print("Z: "); Serial.print(Gznew); Serial.print("  ");
  Serial.println("rad/s ");
  delay(10); 

  //Accel and Magnetometer Sample code
  sensors_event_t aevent, mevent;

  /* Get a new sensor event 
  accelmag.getEvent(&aevent, &mevent);
  /*Low Pass Filter for Accel events
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
  
  /*Low Pass Filter for magnetic events
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
  Serial.println("");
  //Madgwick filter Implementation
  filter.updateIMU(Gxnew, Gynew, Gznew, Axnew, Aynew, Aznew);
  roll = filter.getRollRadians(); pitch = filter.getPitchRadians(); heading = filter.getYawRadians();
  //Filtering Noise off of Madgwick Output
  /*Low Pass Filter for Direction
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
  Serial.println();  
  
  */
