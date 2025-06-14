#ifndef TRI_LINEAR_HALL_H
#define TRI_LINEAR_HALL_H

#include <Arduino.h>
#include <math.h>
#include "../common/base_classes/Sensor.h"

class TriLinearHall: public Sensor{
public:
  // --- Configuration ---
  static const uint8_t  PINS       = 3;   // channels A0, A1, A2

  // --- Constructor ---
  TriLinearHall(uint8_t _pp, uint16_t window, float _tau);

  // --- Initialization & state update ---
  void     init();
  uint16_t rollAverage(uint8_t ch);
  uint16_t minMax(uint8_t ch);
  float    normalise(uint8_t ch);

  // --- Angle & velocity getters ---
  float    getSensorAngle();
  float    getVelocity();

private:
  uint16_t WINDOW;  // samples per rolling average
  uint8_t  POLE_PAIRS;   // 8 magnets → 4 electrical pole-pairs
  float TAU; // velocity smoothing interval in seconds
  float lastElec;
  float elecUnwrapped;
  
  // rolling average state
  uint16_t *buf  [PINS];
  uint32_t sum   [PINS];
  uint16_t idx   [PINS];
  uint16_t spl   [PINS];
  uint16_t avg   [PINS];

  // min/max tracking
  uint16_t minRd [PINS];
  uint16_t maxRd [PINS];
  uint16_t center[PINS];

  // velocity tracking
  float     MAX_VEL_RAD_S;
  float     lastVelAngle;
  float     lastValidVel;
  unsigned long lastVelMicros;
  
  // rolling‐average state for velocity
  float    velSum   = 0;
  uint16_t velIdx   = 0;
  uint16_t velCount = 0;
};

#endif // TRI_LINEAR_HALL_H
