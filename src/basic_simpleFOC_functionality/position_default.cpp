// #include <SimpleFOC.h>

// BLDCMotor motor = BLDCMotor(14);
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);
// // channel A and B callbacks
// // void doA(){encoder.handleA();}
// // void doB(){encoder.handleB();}

// void setup() {
  
//   // initialize encoder sensor hardware
//   encoder.init();
// //   encoder.enableInterrupts(doA, doB); 
//   // link the motor to the sensor
//   motor.linkSensor(&encoder);

//   // driver config
//   driver.init();

//   driver.voltage_limit = 10;

//   motor.linkDriver(&driver);

//   // set motion control loop to be used
//   motor.controller = MotionControlType::angle;

//   // controller configuration 
//   // default parameters in defaults.h

//   // controller configuration based on the control type 
//   // velocity PID controller parameters
//   // default P=0.5 I = 10 D =0
//   motor.PID_velocity.P = 0.2;
//   motor.PID_velocity.I = 20;
//   motor.PID_velocity.D = 0.0;
//   // jerk control using voltage voltage ramp
//   // default value is 300 volts per sec  ~ 0.3V per millisecond
//   motor.PID_velocity.output_ramp = 1000;

//   // velocity low pass filtering
//   // default 5ms - try different values to see what is the best. 
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.01;

//   // angle P controller -  default P=20
//   motor.P_angle.P = 20;

//   //  maximal velocity of the position control
//   // default 20
//   motor.velocity_limit = 4;
//   // default voltage_power_supply
//   motor.voltage_limit = 3;

//   // use monitoring with serial 
//   Serial.begin(115200);
//   // comment out if not needed
// //   motor.useMonitoring(Serial);
  
//   // initialize motor
//   motor.init();
//   // align encoder and start FOC
//   motor.initFOC();


//   Serial.println("Motor ready.");
//   _delay(1000);
// }

// // angle set point variable
// float target_angle = 1;
// // timestamp for changing direction
// long timestamp_us = _micros();

// void loop() {

//   // each one second
//   if(_micros() - timestamp_us > 1e6) {
//       timestamp_us = _micros();
//       // inverse angle
//       target_angle = -target_angle;   
//   }

//   // main FOC algorithm function
//   motor.loopFOC();

//   // Motion control function
//   motor.move(target_angle);
// }