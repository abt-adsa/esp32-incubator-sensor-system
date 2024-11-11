#include <Arduino.h>

#define MIC_PIN 14

const int BIAS = 1920;
int analog;
int maxVal;
const int LOOP_TIME = 500;        // Reduced sampling period for faster updates
const int SPL_REF = 94;
const float PEAK_REF = 0.00631;
const float AVG_REF = 0.00631;
const float ampGain = 125.0;

int n;
unsigned long sum;
unsigned long readStartTime;

float average;
float dBSPLPeak;
float dBSPLAvg;

void setup() {   
  Serial.begin(9600);           // Higher baud rate for faster serial output
}
 
void loop() {
  maxVal = 0;
  sum = 0;
  n = 0;

  readStartTime = millis();

  // Sampling for the defined LOOP_TIME (in ms)
  while (millis() - readStartTime < LOOP_TIME) {
    analog = abs(analogRead(MIC_PIN) - BIAS);
    if (analog > maxVal)
      maxVal = analog;

    sum += analog;
    n++;
  }

  // Calculate average amplitude over the sampling period
  average = (float)sum / n;

  // Calculate SPL in dB for peak and average
  dBSPLPeak = SPL_REF + 20 * log10((float)maxVal / (PEAK_REF * ampGain)) - 80;
  dBSPLAvg = SPL_REF + 20 * log10((float)average / (AVG_REF * ampGain));

  // Output results to Serial Monitor
  Serial.print(" maxVal = ");
  Serial.print(maxVal);
  Serial.print("    ");
  Serial.print(" average = ");
  Serial.print(average);
  Serial.print("    ");
  Serial.print(dBSPLPeak, 1);
  Serial.print(" dB SPL Peak");
  Serial.print("    ");
  Serial.print(dBSPLAvg, 1);
  Serial.println(" dB SPL average");

  // No delay here to allow continuous measurement
}
