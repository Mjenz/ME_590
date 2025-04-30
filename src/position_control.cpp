// #include <SimpleFOC.h>
// #include <math.h>
// #include <string>
// #include <iostream>

// #include "hardware/pwm.h"
// #include "hardware/irq.h"

// #define SIN_LENGTH 50
// // Motor and driver setup
// BLDCMotor motor = BLDCMotor(14,3);
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);

// // Commander command = Commanader(Serial);
// // void doMotor(char* cmd) { command.motor(&motor, cmd); }

// // Sine array setup
// float sin_array[SIN_LENGTH];
// void generate_sin(float * array, int length) {
//   for (int i = 0; i < length; i++) {
//     float x = (2 * PI * i) / length;
//     array[i] = 0.2f * sin(x); // Sine wave, amplitude 2 radians
//   }
// }


// void setup() {
//   encoder.init();
//   motor.linkSensor(&encoder);

//   driver.init();
//   motor.linkDriver(&driver);

//   motor.controller = MotionControlType::angle;

//   motor.PID_velocity.P = 0.025; // was .5
//   motor.PID_velocity.I = 1; // was 20
//   motor.PID_velocity.D = 0.0001; // was 0.001
//   motor.PID_velocity.output_ramp = 1000;
//   motor.LPF_velocity.Tf = 0.001;// was 0.01
//   motor.P_angle.P = 20;
//   motor.velocity_limit = 4;
//   motor.voltage_limit = 8;

//   Serial.begin(115200);
//   motor.useMonitoring(Serial);

//   motor.init();
//   motor.initFOC();

//   Serial.println("Motor ready.");
//   generate_sin(sin_array, SIN_LENGTH);
//   _delay(1000);
// }

// void loop_sinusoid() {
//     int c = 0;
//     float target_angle = 0;

//     while(1){
//         motor.loopFOC();

//         // Set target angle from sine array
//         target_angle = sin_array[c];
//         char ms[50];
//         sprintf(ms,">target_angle:%f\n>actual:%f\n", target_angle, encoder.getAngle());
//         Serial.println(ms);
//         motor.move(target_angle);  // Tell motor to move to that angle

//         c++;
//         if (c >= SIN_LENGTH) c = 0;

//         if(Serial.available()){
//             String input = Serial.readStringUntil('\r');
//             if (input == "X") {
//                 break;
//             }
//         }

//         _delay(20); // Delay to slow down sine wave playback
//     }
// }


// void loop() {
//     loop_sinusoid();
//     // Serial.println("SimpleFOC testbed:");
//     // Serial.println("For sin loop type 'L'");
//     // while(!Serial.available()){}
//     // if (Serial.available()) {
//     //     String input = Serial.readStringUntil('\r');
//     //     if (input == "L") {
//     //         loop_sinusoid();  // This blocks forever — optional improvement below
//     //     }
//     // }

// }