#ifndef LINEAR_HALL_H
#define LINEAR_HALL_H

#include <Arduino.h>
#include <math.h>
#include "../common/base_classes/Sensor.h"

class LinearHall: public Sensor{
public:
  // --- Configuration (you can tweak these before including) ---
  static const uint8_t  PINS       = 3;   // channels A0, A1, A2

  // --- Constructor ---
  // _pp and window are optional overrides of the above
  LinearHall(uint8_t _pp, uint16_t window, uint16_t vel_window);

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
  uint16_t VEL_WINDOW;
  float lastElec;
  float elecUnwrapped;
  float   velAccum;
  uint8_t velCount;
  
  // rolling average state
  uint16_t *buf   [PINS];
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
};

#endif // LINEAR_HALL_H
