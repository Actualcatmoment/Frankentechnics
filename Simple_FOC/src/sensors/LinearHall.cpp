// Shaft angle calculation with 3-phase Hall sensors and rolling average smoothing

#include "LinearHall.h"

LinearHall::LinearHall(uint8_t _pp, uint16_t window, float _tau): 
    WINDOW(window),      // samples per rolling average
    POLE_PAIRS(_pp),       // 8 magnets → 4 electrical pole-pairs
	TAU(_tau),
    MAX_VEL_RAD_S(20.0f),
	lastVelAngle(0.0f),
	lastValidVel(0.0f),
	lastVelMicros(0),
	lastElec(0.0f),
	elecUnwrapped(0.0f)
  {
	for (int i = 0; i < PINS; ++i) buf[i] = new uint16_t[WINDOW];
  }

// Update rolling average for channel ch (0 → A0, 1 → A1, 2 → A2)
uint16_t LinearHall::rollAverage(uint8_t ch){ //find rolling average
  uint16_t x = analogRead(A0 + ch); //read values
  sum[ch] -= buf[ch][idx[ch]]; //sliding window sum
  buf[ch][idx[ch]] = x;
  sum[ch] += x;

  idx[ch] = (idx[ch] + 1) % WINDOW; //increment index
  if(spl[ch] < WINDOW){spl[ch]++;} //increment real sample count
  avg[ch] = sum[ch] / spl[ch]; //average value
  return avg[ch];
}

//find min/max values
uint16_t LinearHall::minMax(uint8_t ch){
  uint8_t limit = spl[ch]; //until full, scan only valid entries.
  for (int i = 0; i < limit; ++i) {
    uint16_t v = buf[ch][i];
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

  // 3) detect and correct wrap-around at ±π
  float delta = elec - lastElec;
  if (delta >  M_PI) delta -= 2 * M_PI;
  if (delta < -M_PI) delta += 2 * M_PI;

  if (fabs(delta) < 0.01f) delta = 0.0f; //reject tiny deltas
  elecUnwrapped += delta;   // now grows continuously
  lastElec        = elec;

  // 4) convert to mechanical angle (unwrapped)
  float mechUnwrapped = elecUnwrapped / float(POLE_PAIRS);

  // 5) fold it into [0 … 2π)
  float mechWrapped = fmod(mechUnwrapped, 2 * M_PI);
  if (mechWrapped < 0) mechWrapped += 2 * M_PI;

  return mechWrapped;
}

float LinearHall::getVelocity() {
  // 1) get the latest mechanical angle
  float mech = getSensorAngle();

  // 2) compute dt correctly
  unsigned long now = micros();
  float dt = (now - lastVelMicros) * 1e-6f;
  if (dt <= 0) dt = 1e-6f;
  lastVelMicros = now;

  // 3) delta‐angle with wrap correction
  float delta = mech - lastVelAngle;
  if (delta >  M_PI) delta -= 2*M_PI;
  if (delta < -M_PI) delta += 2*M_PI;
  lastVelAngle = mech;

  // 4) instantaneous velocity
  float rawVel = delta / dt;

  // 5) optional dead‐band & sanity clamp
  if (fabs(rawVel) < 0.01f) rawVel = 0.0f; 
  if (fabs(rawVel) > MAX_VEL_RAD_S) rawVel = lastValidVel;

  // 6) single‐pole IIR low-pass filter (time-constant τ)
  //    lastValidVel holds the filtered output from the prior call.
  float alpha = dt / (TAU + dt);          // dt/(τ + dt)
  lastValidVel = alpha * rawVel
               + (1.0f - alpha) * lastValidVel;
			   
  return lastValidVel;
}

// Initialize buffers and sums
void LinearHall::init() {
  for(int i = 0; i < PINS; ++i) {
  memset(buf[i],  0, WINDOW * sizeof(uint16_t));
  }
  velSum = 0; velIdx = velCount = 0;
  memset(sum,  0, sizeof(sum));
  memset(idx,  0, sizeof(idx));
  memset(spl,  0, sizeof(spl));
  memset(avg,  0, sizeof(avg));

  //reset min/max/center
  for (int i = 0; i < PINS; ++i) {
    minRd[i] = UINT16_MAX;
    maxRd[i] = 0;
    center[i] = 0;
  }
  
  //warm up for the rolling window
  for(int i = 0; i < WINDOW; ++i) {
    for (uint8_t i = 0; i < PINS; ++i){
      rollAverage(i);
      minMax(i);
    }
  }
  
  //seed unwrap state
  float VA = normalise(0),
		VB = normalise(1),
		VC = normalise(2);
  float alpha = VA - 0.5f * (VB + VC);
  float beta  = (VB - VC) * 0.86602540378f;  // sqrt(3)/2
  float elec0 = atan2(beta, alpha);
  if (elec0 < 0) elec0 += 2*PI;
  lastElec      = elec0;
  elecUnwrapped = elec0;
  
  //seed velocity state
  float mech0 = elec0 / float(POLE_PAIRS);
  lastVelMicros = micros();
  lastValidVel = 0.0f;
  lastVelAngle = mech0;
}