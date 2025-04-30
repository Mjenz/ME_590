// #include <SimpleFOC.h>

// // BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(14,3);
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);

// // current sensor
// InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50.0, 26,27, _NC);

// #define SIN_LENGTH 50

// // Sine array setup
// float sin_array[SIN_LENGTH];
// void generate_sin(float * array, int length) {
//   for (int i = 0; i < length; i++) {
//     float x = (2 * PI * i) / length;
//     array[i] = 2 * sin(x); // Sine wave, amplitude 2 radians
//   }
// }


// void loop_sinusoid() {
//     int c = 0;
//     float target_torque = 0;

//     while(1){
//         motor.loopFOC();
//         // read currents
//         float DC_current = current_sense.getDCCurrent(motor.electrical_angle);
//         float current_mag = current_sense.getDCCurrent();

//         float electricalangle = motor.electrical_angle;

//         DQCurrent_s foc_current = current_sense.getFOCCurrents(electricalangle);
//         float d_foc_current = foc_current.d;
//         float q_foc_current = foc_current.q;

//         // PhaseCurrent_s phase_current = current_sense.getPhaseCurrents();
//         // DQCurrent_s current = current_sense.getFOCCurrents(motor.electrical_angle);
//         // Set target angle from sine array
//         target_torque = sin_array[c];

//         char ms[75];
//         sprintf(ms,">target_torque:%f\n>DCcurrentmag:%f\n>DCcurrent:%f\n>D_FOC_Current:%f\n>Q_FOC_Current:%f\n",target_torque,current_mag,DC_current,d_foc_current,q_foc_current);//phase_current.a,phase_current.b);
//         Serial.println(ms);
//         motor.move(target_torque);  // Tell motor to move to that angle

//         c++;
//         if (c >= SIN_LENGTH) c = 0;

//         _delay(20); // Delay to slow down sine wave playback
//     }
// }

// void setup() { 
  
//     // initialize encoder sensor hardware
//     encoder.init();
//     // link the motor to the sensor
//     motor.linkSensor(&encoder);
  
//     // driver config
//     // power supply voltage [V]
//     driver.voltage_power_supply = 12;
//     driver.init();
//     // link driver
//     motor.linkDriver(&driver);
//     // link the driver to the current sense
//     current_sense.linkDriver(&driver);

//     // current sense init hardware
//     current_sense.init();
//     // link the current sense to the motor
//     motor.linkCurrentSense(&current_sense);
  
//     // set torque mode:
//     motor.torque_controller = TorqueControlType::foc_current; 
//     // set motion control loop to be used
//     motor.controller = MotionControlType::torque;
  
//     // foc current control parameters (Arduino UNO/Mega)
//     motor.PID_current_q.P = 5;
//     motor.PID_current_q.I= 300;
//     motor.PID_current_d.P= 5;
//     motor.PID_current_d.I = 300;
//     motor.LPF_current_q.Tf = 0.01; 
//     motor.LPF_current_d.Tf = 0.01; 

//     // use monitoring with serial 
//     Serial.begin(115200);
//     // comment out if not needed
//     motor.useMonitoring(Serial);
  
//     // initialize motor
//     motor.init();
//     // align sensor and start FOC
//     if(!motor.initFOC()){
//         //while(1){}
//     }
//     _delay(1000);
//   }
  
//   void loop() {
//     generate_sin(sin_array, SIN_LENGTH);
//     loop_sinusoid();
//   }