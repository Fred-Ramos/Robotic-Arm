#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include <VL53L0X.h>

//DISTANCE SENSOR VARIABLES
VL53L0X tof;
float measurement, distance, prev_distance;

unsigned long interval_tof;
unsigned long currentMicros_tof, previousMicros_tof;

//DISTANCE SENSOR FUNCTIONS
//TOF SENSOR SETUP
void setup_VL53L0X(){
  interval_tof = 60 * 1000;

  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  tof.setMeasurementTimingBudget(40000);
  // Start distance measurement
  tof.startContinuous();     
}

void tof_measure(){
  currentMicros_tof = micros();
  if (currentMicros_tof - previousMicros_tof >= interval_tof){
    previousMicros_tof = currentMicros_tof;

    measurement = tof.readRangeContinuousMillimeters(); 
    if (measurement) {
      prev_distance = distance;
      distance = measurement * 1e-1; //pass to centimeters
      measurement = 0;
    }
    Serial.println(" Dist: " + String(distance, 3));
  }
}


#endif