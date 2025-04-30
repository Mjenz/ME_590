// #include <SimpleFOC.h>

// // #include "hardware/pwm.h"
// // #include "hardware/irq.h"
// // void sync_pwm_slices(int pinA, int pinB, int pinC) {
// //     // Get default PWM config
// //     pwm_config cfg = pwm_get_default_config();

// //     // Aim for ~25 kHz frequency with full resolution (wrap = 255)
// //     pwm_config_set_clkdiv(&cfg, 1.0f); // No division
// //     uint16_t top = 5000;  // Adjust this value depending on system clock
// //     pwm_config_set_wrap(&cfg, top);   // Lower wrap = higher frequency

// //     for (int pin : {pinA, pinB, pinC}) {
// //         gpio_set_function(pin, GPIO_FUNC_PWM);
// //         uint slice = pwm_gpio_to_slice_num(pin);
// //         pwm_init(slice, &cfg, false); // Don't start yet
// //     }

// //     // Enable all slices at the same time
// //     for (int pin : {pinA, pinB, pinC}) {
// //         uint slice = pwm_gpio_to_slice_num(pin);
// //         pwm_set_enabled(slice, true);
// //     }
// // }
// // BLDCMotor(pole pair number, phase resistance (optional) );
// BLDCMotor motor = BLDCMotor(14,3);
// // BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);

// // instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
// void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

// void setup() {

//   // use monitoring with serial 
//   Serial.begin(115200);
//   // enable more verbose output for debugging
//   // comment out if not needed
//   SimpleFOCDebug::enable(&Serial);

//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 12;
//   // limit the maximal dc voltage the driver can set
//   // as a protection measure for the low-resistance motors
//   // this value is fixed on startup
//   driver.voltage_limit = 10;
// //   sync_pwm_slices(0,1,2);
//   if(!driver.init()){
//     Serial.println("Driver init failed!");
//     return;
//   }
//   // link the motor and the driver
//   motor.linkDriver(&driver);

//   // limiting motor movements
//   // limit the voltage to be set to the motor
//   // start very low for high resistance motors
//   // current = voltage / resistance, so try to be well under 1Amp
//   motor.voltage_limit = 10;   // [V]
 
//   // open loop control config
//   motor.controller = MotionControlType::velocity_openloop;

//   // init motor hardware
//   if(!motor.init()){
//     Serial.println("Motor init failed!");
//     return;
//   }

//   // set the target velocity [rad/s]
//   motor.target = 6.28; // one rotation per second

//   // add target command T
//   command.add('T', doTarget, "target velocity");
//   command.add('L', doLimit, "voltage limit");

//   Serial.println("Motor ready!");
//   Serial.println("Set target velocity [rad/s]");
//   _delay(1000);
// }

// void loop() {
//   // open loop velocity movement
//   motor.move();

//   // user communication
//   command.run();
// }