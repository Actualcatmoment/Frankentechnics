 #include <SimpleFOC.h>

// BLDC motor instance BLDCMotor(polepairs, (R), (KV), (L))
BLDCMotor motor = BLDCMotor(8, 42);

// BLDC driver instance BLDCDriver3PWM(phA, phB, phC, (en))
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8); 

// commander instance
Commander command = Commander(Serial);
void doMotor(char* cmd){ command.motor(&motor,cmd); }

//sensor instance
LinearHall120 sensor = LinearHall120(A0, A1, A2, 4);

void setup() {
  pinMode(6, OUTPUT);
  //Start serial, Wait for stable.
  Serial.begin(115200);
  while(!Serial){};
  SimpleFOCDebug::enable();

  //Driver config and motor link.
  driver.enable_active_high = false;
  driver.pwm_frequency = 32000;
  driver.voltage_power_supply = 30;
  driver.voltage_limit = 10;
  driver.init();
  motor.linkDriver(&driver);

  // set motion control type to velocity openloop, and torque control to voltage (default).
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.modulation_centered = 1.0;
  motor.motion_downsample = 0.0;
  motor.phase_resistance = 42.0;
  motor.velocity_limit = 20;
  motor.voltage_limit = 10;
  motor.current_limit = 1;
  motor.voltage_sensor_align = 7;
  motor.sensor_direction=Direction::CW;
  motor.zero_electric_angle=5.3848;
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 0; // disable monitor at first - optional
  motor.init();

  //Sensor
  sensor.init(542, 489, 520);
  motor.linkSensor(&sensor);
  
  motor.initFOC();

  //Enable commander
  command.add('M',doMotor,"motor");

}

void loop() {
  //Motor control & serial commands for simpleFOC
  motor.loopFOC();
  motor.move();
  command.run();
  motor.monitor();
} 
