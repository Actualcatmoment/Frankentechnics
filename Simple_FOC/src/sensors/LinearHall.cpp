// Shaft angle calculation with 3-phase Hall sensors and rolling average smoothing

#include "LinearHall.h"

LinearHall::LinearHall(uint8_t _pp, uint16_t _win) : Sensor()
  , WINDOW(_win)      // samples per rolling average
  , POLE_PAIRS(_pp)       // 8 magnets → 4 electrical pole-pairs
{
// --- Configuration ---
constexpr uint8_t PINS= 3;   // channels A0, A1, A2
uint32_t sum[PINS]    = {0}; // running sum per channel
uint16_t idx[PINS]    = {0}; // rolling index per channel
uint16_t spl[PINS]    = {0}; // sample count per channel
uint16_t avg[PINS]    = {0};
uint16_t center[PINS], minRd[PINS], maxRd[PINS];
}

// Initialize buffers and sums
void LinearHall::init() {
  memset(buf, 0, sizeof(buf));
  memset(spl, 0, sizeof(spl));
  // sums and idx[] are zeroed on declaration
  for (int i = 0; i < PINS; ++i) {
    minRd[i] = UINT16_MAX;
    maxRd[i] = 0;
    center[i] = 0;
  }
  for(int i = 0; i < WINDOW; ++i) {
    for (int i = 0; i < PINS; ++i){
      rollAverage(i);
      minMax(i);
    }
  }
}

// Update rolling average for channel ch (0 → A0, 1 → A1, 2 → A2)
uint16_t LinearHall::rollAverage(uint8_t ch){ //find rolling average
  uint16_t x = analogRead(A0 + ch); //read values
  uint32_t offset = ch * WINDOW + idx[ch];
  sum[ch]   -= buf[offset];
  buf[offset] = x;
  sum[ch]   += x;
  idx[ch] = (idx[ch] + 1) % WINDOW; //increment index
  if(spl[ch] < WINDOW){spl[ch]++;} //increment real sample count
  avg[ch] = sum[ch] / spl[ch]; //average value
  return avg[ch];
}

//find min/max values
uint16_t LinearHall::minMax(uint8_t ch){
  uint8_t limit = spl[ch]; //until full, scan only valid entries.
  for (int i = 0; i < limit; ++i) {
    uint16_t v = buf[ch * WINDOW + i];
    if (v < minRd[ch]) minRd[ch] = v; //min read
    if (v > maxRd[ch]) maxRd[ch] = v; //max read
  }
  center[ch] = (minRd[ch] + maxRd[ch]) / 2; //midpoint
  return center[ch];
}

float LinearHall::normalise(uint8_t ch) {
  float pos = maxRd[ch] - minMax(ch);
  float neg = minMax(ch) - minRd[ch];
  float amp = max(pos, neg);
  
  return (rollAverage(ch) - minMax(ch)) / amp;
}

// Compute shaft mechanical angle [rad]
float LinearHall::getSensorAngle() {
  // Read and smooth each channel
  float VA = normalise(0);
  float VB = normalise(1);
  float VC = normalise(2);

  // Clarke transform (αβ-plane)
  float alpha = VA - 0.5f * (VB + VC);
  float beta  = (VB - VC) * 0.86602540378f;  // sqrt(3)/2

  // Electrical angle [0..2π)
  float elec = atan2(beta, alpha);
  if (elec < 0) elec += 2*PI;

  // keep these static so they persist across calls:
  static float lastElec      = 0;
  static float elecUnwrapped = 0;

  // 3) detect and correct wrap-around at ±π
  float delta = elec - lastElec;
  if (delta >  M_PI) delta -= 2 * M_PI;
  if (delta < -M_PI) delta += 2 * M_PI;

  elecUnwrapped += delta;
  lastElec        = elec;

  // 4) convert to mechanical angle (unwrapped)
  float mechUnwrapped = elecUnwrapped / float(POLE_PAIRS);

  // 5) fold it into [0 … 2π)
  float mechWrapped = fmod(mechUnwrapped, 2 * M_PI);
  if (mechWrapped < 0) mechWrapped += 2 * M_PI;

  return mechWrapped;
}

float LinearHall::getVelocity() {
  float angle = getSensorAngle();        // returns wrapped [0…2π)
  float now   = micros() * 1e-6f;        // seconds
  
  float delta = angle - lastVelAngle; // calc delta(Angle)

  float dt = now - (lastVelTime * 1e-6f); //calculate delta(Time) this call vs previous
  if (dt <= 0) dt = 1e-6f;   // guard against zero
  float vel = delta / dt;        // calc raw rad/s

  if(fabs(vel) > MAX_VEL_RAD_S) {return lastValidVel;} //Sanity check for greater than 10rads/s (or user defined value)
	
  // store for next call
  lastValidVel = vel;
  lastVelAngle = angle;
  lastVelTime  = micros();

  return vel;
}
