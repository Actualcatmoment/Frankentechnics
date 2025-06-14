#ifndef LINEAR_HALL_SENSOR_LIB_H
#define LINEAR_HALL_SENSOR_LIB_H

#include <SimpleFOC.h>

// This function can be overridden with custom ADC code on platforms with poor analogRead performance.
void ReadLinearHalls(int hallA, int hallB, int hallC, int *a, int *b, int *c);

/*
This sensor class is for three linear hall effect sensors such as 49E, which are
positioned 120 electrical degrees apart.
It can also be used for a single magnet mounted to the motor shaft (set pp to 1).

For more information, see this forum thread and PDF
https://community.simplefoc.com/t/40-cent-magnetic-angle-sensing-technique/1959
https://gist.github.com/nanoparticle/00030ea27c59649edbed84f0a957ebe1

For this 3phase version specifically see: https://community.simplefoc.com/t/technics-sl-d2-direct-drive-turntable-restoration-ft-linear-hall-sensors/7313/3

*/

class LinearHall120: public Sensor{
  public:
    LinearHall120(int hallA, int hallB, int hallC, int pp);

	// Initialize without moving motor
    void init(int centerA, int centerB, int centerC); 
	
	// Move motor to find center values
    void init(class FOCMotor *motor); 

    // Get current shaft angle from the sensor hardware, and
    // return it as a float in radians, in the range 0 to 2PI.
    //  - This method is pure virtual and must be implemented in subclasses.
    //    Calling this method directly does not update the base-class internal fields.
    //    Use update() when calling from outside code.
    float getSensorAngle() override;

    int centerA;
    int centerB;
	int centerC;
    int lastA, lastB, lastC;

  private:
    int pinA;
    int pinB;
	int pinC;
    int pp;
    int electrical_rev;
    float prev_reading;
};

#endif
