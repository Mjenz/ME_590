// #include <SimpleFOC.h>
// #include <math.h>
// #include <string>
// #include <iostream>

// #include "hardware/pwm.h"
// #include "hardware/irq.h"


// #define SIN_LENGTH 50
// // motor instance
// BLDCMotor motor = BLDCMotor(14,3);
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);

// // one possible combination of current sensing pins for SimpleFOCShield v2
// // shunt - 10milliOhm
// // gain  - 50 V/V 
// InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50.0, 26,27, _NC);


// // Sine array setup
// float sin_array[SIN_LENGTH];
// void generate_sin(float * array, int length) {
//   for (int i = 0; i < length; i++) {
//     float x = (2 * PI * i) / length;
//     array[i] = 2.0f * sin(x); // Sine wave, amplitude 2 radians
//   }
// }

// void sync_pwm_slices(uint pinA, uint pinB, uint pinC) {
//     uint sliceA = pwm_gpio_to_slice_num(pinA);
//     uint sliceB = pwm_gpio_to_slice_num(pinB);
//     uint sliceC = pwm_gpio_to_slice_num(pinC);

//     // Disable all slices before configuration
//     pwm_set_enabled(sliceA, false);
//     pwm_set_enabled(sliceB, false);
//     pwm_set_enabled(sliceC, false);

//     // Set counters to 0
//     pwm_set_counter(sliceA, 0);
//     pwm_set_counter(sliceB, 0);
//     pwm_set_counter(sliceC, 0);

//     // Sync enable with interrupts disabled for tight timing
//     // uint32_t irq_state = disabl();
//     pwm_set_enabled(sliceA, true);
//     pwm_set_enabled(sliceB, true);
//     pwm_set_enabled(sliceC, true);
//     // restore_interrupts(irq_state);
// }


// void loop_sinusoid() {
//     int c = 0;
//     float target_torque = 0;

//     while(1){
//         motor.loopFOC();
//         // read currents
//         float DC_current = current_sense.getDCCurrent(motor.electrical_angle);
//         float current_mag = current_sense.getDCCurrent();
//         // PhaseCurrent_s phase_current = current_sense.getPhaseCurrents();
//         // DQCurrent_s current = current_sense.getFOCCurrents(motor.electrical_angle);
//         // Set target angle from sine array
//         target_torque = sin_array[c];
//         char ms[50];
//         sprintf(ms,">target_angle:%f\n>DC currentmag:%f\n>DCcurrent:%f\n", target_torque,current_mag,DC_current);//phase_current.a,phase_current.b);
//         Serial.println(ms);
//         motor.move(target_torque);  // Tell motor to move to that angle

//         c++;
//         if (c >= SIN_LENGTH) c = 0;

//         if(Serial.available()){
//             String input = Serial.readStringUntil('\r');
//             if (input == "X") {
//                 break;
//             }
//         }

//         _delay(100); // Delay to slow down sine wave playback
//     }
// }


// void setup(){
//     // initialize encoder sensor hardware
//     encoder.init();

//     // link the motor to the sensor
//     motor.linkSensor(&encoder);

//     // driver config
//     sync_pwm_slices(0,1,2);
//     driver.init();
//     motor.linkDriver(&driver);

//     // link the driver to the current sense
//     current_sense.linkDriver(&driver);
//     // set motion control loop to be used
//     // motor.controller = MotionControlType::torque;
//     motor.torque_controller = TorqueControlType::foc_current;
//     // Set the phase pattern
//     motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

//     // controller configuration based on the control type 
//     // velocity PID controller parameters
//     // default P=0.5 I = 10 D =0
//     // motor.PID_velocity.P = 0.1;
//     // motor.PID_velocity.I = 20;
//     // motor.PID_velocity.D = 0.0;    
    
//     motor.PID_current_d.P = 0.1;
//     motor.PID_current_d.I = 20;
//     motor.PID_current_d.D = 0.0;
//     // jerk control using voltage voltage ramp
//     // default value is 300 volts per sec  ~ 0.3V per millisecond
//     motor.PID_velocity.output_ramp = 1000;

//     // velocity low pass filtering
//     // default 5ms - try different values to see what is the best. 
//     // the lower the less filtered
//     motor.LPF_velocity.Tf = 0.01;

//     // angle P controller -  default P=20
//     motor.P_angle.P = 20;

//     //  maximal velocity of the position control
//     // default 20
//     motor.velocity_limit = 4;
//     // default voltage_power_supply
//     motor.voltage_limit = 10;

//     // use monitoring with serial 
//     Serial.begin(115200);
//     // comment out if not needed
//     motor.useMonitoring(Serial);

//     // initialize motor
//     motor.init();

//     // init current sense
//     current_sense.init();

//     // link the current sense to the motor
//     motor.linkCurrentSense(&current_sense);

//     // // invert phase b gain
//     // current_sense.gain_b *=-1;
    
//     // alignment
//     motor.voltage_sensor_align=10;
//     current_sense.driverAlign(motor.voltage_sensor_align);

//     // start the FOC
//     motor.initFOC();
// }

// // angle set point variable
// float target_torque = .5;
// // timestamp for changing direction
// long timestamp_us = _micros();

// void loop() {
//     // generate the waveform
//     generate_sin(sin_array, SIN_LENGTH);

//     Serial.println("SimpleFOC testbed:");
//     Serial.println("For sin loop type 'L'");
//     while(!Serial.available()){}
//     if (Serial.available()) {
//         String input = Serial.readStringUntil('\r');
//         if (input == "L") {
//             loop_sinusoid();  // This blocks forever — optional improvement below
//         }
//     }

// }

// // void loop() {

// //     // each one second
// //     if(_micros() - timestamp_us > 1e6) {
// //         timestamp_us = _micros();
// //         // inverse angle
// //         target_torque = -target_torque;   
// //     }

// //     // main FOC algorithm function
// //     motor.loopFOC();

// //     // Motion control function
// //     motor.move(target_torque);
// // }