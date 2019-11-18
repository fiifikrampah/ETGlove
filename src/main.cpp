/*
  Library for the MMA8452Q
  By: Jim Lindblom and Andrea DeVore
  SparkFun Electronics

  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14587

  This sketch uses the SparkFun_MMA8452Q library to initialize
  the accelerometer, change the output data date, and stream
  calculated x, y, z, acceleration values from it (in g units).

  Hardware hookup:
  Arduino --------------- MMA8452Q Breakout
    3.3V  ---------------     3.3V
    GND   ---------------     GND
  SDA (A4) --\/330 Ohm\/--    SDA
  SCL (A5) --\/330 Ohm\/--    SCL

  The MMA8452Q is a 3.3V max sensor, so you'll need to do some
  level-shifting between the Arduino and the breakout. Series
  resistors on the SDA and SCL lines should do the trick.

  License: This code is public domain, but if you see me
  (or any other SparkFun employee) at the local, and you've
  found our code helpful, please buy us a round (Beerware
  license).

  Distributed as is; no warrenty given.
*/

#include <Wire.h>                 // Must include Wire library for I2C
#include "SparkFun_MMA8452Q.h"    // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include <arduinoFFT.h>
#include "math.h"
 
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
 
unsigned int sampling_period_us;
unsigned long microseconds;
double vImag[SAMPLES];
double magnitude[SAMPLES];

MMA8452Q accel;                   // create instance of the MMA8452 class
arduinoFFT FFT = arduinoFFT();    // create FFT instance 

void setup() {
  Serial.begin(9600);
  Serial.println("MMA8452Q Change Output Data Rate Code!");
  Wire.begin();

  if (accel.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    while (1);
  }

  /* Default output data rate (ODR) is 800 Hz (fastest)
     Set data rate using ODR_800, ODR_400, ODR_200, 
     ODR_100, ODR_50, ODR_12, ODR_6, ODR_1
     Sets data rate to 800, 400, 200, 100, 50, 12.5, 
     6.25, or 1.56 Hz respectively 
     See data sheet for relationship between voltage
     and ODR (pg. 7)
     https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/MMA8452Q-rev8.1.pdf */
  accel.setDataRate(ODR_100);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
}

void loop() {
  if (accel.available()) {      // Wait for new data from accelerometer


  /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
        double val = (accel.getCalculatedX()*accel.getCalculatedX()) + (accel.getCalculatedY()*accel.getCalculatedY()) + (accel.getCalculatedZ()*accel.getCalculatedZ());
        magnitude[i]= sqrt(val);        
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        }
    }

     /*FFT-x*/
    FFT.Windowing(magnitude, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(magnitude, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(magnitude, vImag, SAMPLES);
    double peak = FFT.MajorPeak(magnitude, SAMPLES, SAMPLING_FREQUENCY);
    
    /*PRINT RESULTS*/
    Serial.println("Nyquist frequency is: ");     //Print out what frequency is the most dominant.
    Serial.println(peak);

  }
}
