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

double vImag2[SAMPLES];
double magnitude2[SAMPLES];
double accel_val = 0;
double emg_val = 0;
double peak = 0;
double peak2 = 0;
double peakAvg = 0;
double peakTemp = 0;
int count = 0;
double peakAvg2 = 0;
double peakTemp2 = 0;
int count2 = 0;
int motor_pin = 5;
int pot_pin = A1; //set potentiometer pin
int output, motor_val; //used with pot control 

MMA8452Q accel;                   // create instance of the MMA8452 class
arduinoFFT FFT = arduinoFFT();    // create FFT instance 

void setup() {
  pinMode(motor_pin, OUTPUT);
  Serial.begin(9600);
  Serial.println("MMA8452Q Change Output Data Rate Code!");
  Wire.begin();

  if (accel.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    while (1);
  }

  accel.setDataRate(ODR_400);
  accel.setScale(SCALE_2G);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
}

double getAccel() {
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
    FFT.Windowing(magnitude, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.Compute(magnitude, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(magnitude, vImag, SAMPLES);
    double peak = FFT.MajorPeak(magnitude, SAMPLES, SAMPLING_FREQUENCY);
    peak = peak - (2*9.81);
    peakTemp += peak; 
    count = count + 1; 
    if (count == 1){
      peakAvg = peakTemp/1;
      count = 0;
      peakTemp = 0;
    }
    return peakAvg;
  }
}



double getEMG() {
    // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(voltage);
  delay(1);

  /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
        magnitude2[i]= voltage;        
        vImag2[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        }
    }

     /*FFT-x*/
    FFT.Windowing(magnitude2, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.Compute(magnitude2, vImag2, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(magnitude2, vImag2, SAMPLES);
    double peak2 = FFT.MajorPeak(magnitude2, SAMPLES, SAMPLING_FREQUENCY);
     /*PRINT RESULTS*/
    // Serial.println("Nyquist frequency of EMG is: ");     //Print out what frequency is the most dominant.
    // Serial.println(peak2);

    peakTemp2 += peak2; 
    count2 = count2 + 1; 
    if (count2 == 1){
      peakAvg2 = peakTemp2/1;
      count2 = 0;
      peakTemp2 = 0;
    }
    return peakAvg2;
}

void motors()
{

    for(int i=0; i<255; i++)
    {
      analogWrite(motor_pin, 255); //spin motors at 150 speed
      if (i == 128) //halfway through the loop, check hz again
        accel_val = getAccel();
        if (accel_val < 13.9 and accel_val > 8.1){
          motors(); //re-call motors()
        }
        else{ //if not within range, slow motors down
          for(int i=255; i>0; i--){
            analogWrite(motor_pin, i);
            delay(2);
        }
      }

  }
}

//main 
void loop(){
  accel_val = getAccel();
  Serial.println("The calculated frequency is: ");     //Print out what avg frequency is the most dominant.
  Serial.println(accel_val);
  // delay(5);
  if (accel_val < 13.9 and accel_val > 8.1){
    Serial.println("There is ET vibration: ");
    Serial.println("Suppression has begun");
    motors(); 
  }
    // if the value is within ET range, spin the motors
  // Control from pot (incomplete logic): 
  //Reading from potentiometer
  // output = analogRead(pot_pin);
  // motor_val = map(output, 0, 1023, 0, 255); // Map the potentiometer value from 0 to 255
  // analogWrite(motor_pin, motor_val); //write pot value to motor
  // delay(1);
  // emg_val = getEMG();
 
}