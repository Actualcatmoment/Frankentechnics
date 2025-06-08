 #include <SimpleFOC.h>

// BLDC motor instance BLDCMotor(polepairs, (R), (KV), (L)) 8, 42, nan, 66
BLDCMotor motor = BLDCMotor(8, 42);

// BLDC driver instance BLDCDriver3PWM(phA, phB, phC, (en))
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 6, 11, 8); 

// commander instance
Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

//sensor instance
HallSensor sensor = HallSensor(A0, A1, A2, 8);

void setup() {
  analogReadResolution(12);
  analogWriteResolution(10);
  analogWrite(D10, 125);

  // start serial
  Serial.begin(38400);
  while(!Serial){};

  //Sensor
  sensor.init();

  //Driver config and motor link.
  driver.voltage_power_supply = 30;
  driver.voltage_limit = 12;
  driver.pwm_frequency = 50000;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  // set motion control type to velocity openloop, and torque control to voltage (default).
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.phase_resistance = 42;
  motor.current_limit = 0.7;
  motor.init();

  //Enable commander
  commander.add('M',onMotor,"my motor");

  motor.initFOC();
}

void loop() {
  //Motor control & serial commands for simpleFOC
  motor.loopFOC();
  motor.move();
  commander.run();
} 
