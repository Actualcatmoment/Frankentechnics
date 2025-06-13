#ifndef LINEAR_HALL_H
#define LINEAR_HALL_H

#include <Arduino.h>
#include <math.h>
#include "../common/base_classes/Sensor.h"

class LinearHall : public Sensor{
public:
  /**
   * @param polePairs   Number of electrical pole-pairs (e.g. 4 for 8 magnets)
   * @param windowSize  Rolling-average window length (e.g. 20)
   */
  LinearHall(uint8_t _pp, uint16_t window);
  
  void setMaxVelocity(float vmax_rad_s) { MAX_VEL_RAD_S = vmax_rad_s; } //configuration of maximum velocity 

  /** Seed buffers and compute per-channel centers */
  void init();

  /** Call as often as you like; returns smoothed & unwrapped shaft angle [0…2π) or shaft velocity in rads/s */
  float getSensorAngle();
  float getVelocity();
  
private:
  // core helpers
  uint16_t  rollAverage(uint8_t ch);
  uint16_t  minMax     (uint8_t ch);
  float     normalise  (uint8_t ch);

  // parameters
  uint8_t  POLE_PAIRS;
  uint16_t WINDOW;

  // fixed: three analog channels A0…A2
  static constexpr uint8_t PINS = 3;

  // rolling-average state
  uint16_t  *buf;      // flatten [PINS][WINDOW] into 1D
  uint32_t  *sum;      // [PINS]
  uint16_t  *idx;      // [PINS]
  uint16_t  *spl;      // [PINS]
  uint16_t  *avg;      // [PINS]

  // min/max & center
  uint16_t  *minRd;    // [PINS]
  uint16_t  *maxRd;    // [PINS]
  uint16_t  *center;   // [PINS]
  
  // unwrap tracking
  float lastElec;
  float elecUnwrapped;
  
  // for velocity calculation
  float        lastVelAngle = 0.0f;      // last mechanical unwrapped angle
  unsigned long lastVelTime  = 0;       // micros() at last call
  float        lastValidVel   = 0.0f;     // last “good” velocity
  float  MAX_VEL_RAD_S  = 10.0f;   // maximum tolerated velocity [rad/s]
  
};

#endif // LINEAR_HALL_H
