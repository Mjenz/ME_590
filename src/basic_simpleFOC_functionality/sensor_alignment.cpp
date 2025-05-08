// #include <SimpleFOC.h>

// BLDCMotor motor = BLDCMotor(7);  // Example: 7 pole pairs
// BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11);
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);  // CS on pin 10

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   // Initialize sensor and motor
//   sensor.init();
//   motor.linkSensor(&sensor);

//   driver.voltage_power_supply = 12; // set the power supply voltage
//   driver.init();
//   motor.linkDriver(&driver);

//   motor.voltage_sensor_align = 3;  // Aligning voltage (sensor alignment)
//   motor.init();                    // Initialize motor parameters
//   motor.initFOC();                 // Initialize FOC (Field-Oriented Control), alignment will happen here

//   // After alignment, the zero_electric_angle and sensor direction will be set correctly
//   Serial.println("Alignment complete!");
//   Serial.print("Offset: ");
//   Serial.println(motor.zero_electric_angle);  // This should be the correct alignment angle
//   Serial.print("Direction: ");
//   Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
// }

// void loop() {
//   // The motor is aligned and ready for movement
//   // You can now add your motor control logic here
// }
