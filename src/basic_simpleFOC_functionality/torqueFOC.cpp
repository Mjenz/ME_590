// #include <SimpleFOC.h>

// // BLDCMotor(pole pair number, phase resistance (optional) );
// // BLDCMotor motor = BLDCMotor(14,2.5);
// BLDCMotor motor = BLDCMotor(7,1.8);
// // BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// // MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// MagneticSensorSPI sensor = MagneticSensorSPI(17, 14);

// Commander command = Commander(Serial);
// void doMotor(char* cmd) { command.motor(&motor, cmd); }

// void setup() { 
  
//   // use monitoring with serial 
//   Serial.begin(115200);
//   // enable more verbose output for debugging
//   // comment out if not needed
//   SimpleFOCDebug::enable(&Serial);

//   // TODO: driver config
//   // init the sensor
//   sensor.init();
//   // link the motor to the sensor
//   motor.linkSensor(&sensor);

//   // TODO: driver config
//   // init the driver
//   if(!driver.init()){
//     Serial.println("Driver init failed!");
//     return;
//   }
//   // link driver
//   motor.linkDriver(&driver);

//   // TODO: motor conf

//   // set motion control loop to be used
//   motor.torque_controller = TorqueControlType::voltage;
//   motor.controller = MotionControlType::torque;

//   // use monitoring with serial
//   // comment out if not needed
//   motor.useMonitoring(Serial);

//   motor.voltage_sensor_align = 5;

//   // initialize motor
//   if(!motor.init()){
//     Serial.println("Motor init failed!");
//     return;
//   }
//   // align sensor and start FOC
//   if(!motor.initFOC()){
//     Serial.println("FOC init failed!");
//     return;
//   }

//   // set the initial motor target
//   motor.target = 2; // Volts 

//   // add target command M
//   command.add('M', doMotor, "Motor");

//   Serial.println(F("Motor ready."));
//   Serial.println(F("Set the target using serial terminal and command M:"));
//   _delay(1000);
// }

// void loop() {
//   // main FOC algorithm function
//   motor.loopFOC();

//   // Motion control function
//   motor.move();

//   // user communication
//   command.run();
// }