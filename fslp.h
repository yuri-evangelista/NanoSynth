#ifndef FSLP_H_
#define FSLP_H_

#include <Arduino.h>

/***********************************************************
                            FSLP
***********************************************************/
// Logic for Interlink Electronics FSLP (Force Sensing Linear Potentiometer)
// Requires specific wiring of Sense Line (SL) and Drive Lines (D1, D2).

inline int fslpGetPosition(byte fslpSenseLine, byte fslpDriveLine1, byte fslpDriveLine2, byte fslpBotR0)
{
  // Step 1 - Clear charge
  pinMode(fslpSenseLine, OUTPUT);  digitalWrite(fslpSenseLine, LOW);
  pinMode(fslpDriveLine1, OUTPUT); digitalWrite(fslpDriveLine1, LOW);
  pinMode(fslpDriveLine2, OUTPUT); digitalWrite(fslpDriveLine2, LOW);
  pinMode(fslpBotR0, OUTPUT);      digitalWrite(fslpBotR0, LOW);
  
  // Step 2 - Set up drive line voltages
  digitalWrite(fslpDriveLine1, HIGH);
  pinMode(fslpBotR0, INPUT);
  pinMode(fslpSenseLine, INPUT);

  // Step 3 - Wait for stabilization
  delayMicroseconds(10); 

  // Step 4 - Measurement
  return analogRead(fslpSenseLine);
}

inline int fslpGetPressure(byte fslpSenseLine, byte fslpDriveLine1, byte fslpDriveLine2, byte fslpBotR0)
{
  // Step 1 - Set up drive line voltages
  pinMode(fslpDriveLine1, OUTPUT); digitalWrite(fslpDriveLine1, HIGH);
  pinMode(fslpBotR0, OUTPUT);      digitalWrite(fslpBotR0, LOW);
  pinMode(fslpSenseLine, INPUT);
  pinMode(fslpDriveLine2, INPUT);

  // Step 2 - Stabilization (optional delay, usually handled by ADC speed)

  // Step 3 - Take measurements
  int v1 = analogRead(fslpDriveLine2);
  int v2 = analogRead(fslpSenseLine);
 
  // Step 4 - Calculate pressure (See FSLP Integration Guide)
  if (v1 == v2) {
    return 32 * 1023; // Max reading/error
  }
  return 16 * v2 / (v1 - v2);
}

/**
 * Main FSLP Reader
 * Returns: true if pressure > threshold (note active), false otherwise.
 * Updates: pressure, position, grade, gain via reference
 */
inline bool getFSLP(byte pinSense, byte pinD1, byte pinD2, byte pinR0, 
                    int &pressure, int &position, int &grade, int &gain) {
                      
  pressure = fslpGetPressure(pinSense, pinD1, pinD2, pinR0);
  position = fslpGetPosition(pinSense, pinD1, pinD2, pinR0);

  // Map raw position (15-1023) to 12 semitones
  grade = map(position, 15, 1023, 0, 12);
  
  // Map pressure logarithmically to gain (volume)
  // 16 levels of volume
  gain = map((int)log(pressure + 1 - 10) / log(1024) * 1024, 1, 2000, 1, 255); 

  // Threshold for "Note On"
  return (pressure > 10);
}

#endif /* FSLP_H_ */