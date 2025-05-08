// #include <SimpleFOC.h>

// #include <Wire.h>

// #include <math.h>
// #include <string>
// #include <iostream>

// #include "hardware/timer.h"
// #include "pico/critical_section.h"

// #include <FreeRTOS.h>
// #include <task.h>
// #include <semphr.h>
// #include <queue.h>
// // These things
// #define I2C_SDA 21
// #define I2C_SCL 22
// #define MPU6050 0x68
// #define IMU_Q_LEN 20
// #define DELAY 25

// #define MAX_SIN_LENGTH 150

// // GLOBAL VARIABLES]

// // Motor and driver setup
// BLDCMotor motor = BLDCMotor(14,3);
// BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);
// InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50.0, 26,27, _NC);

// // Sin array 
// float sin_array[MAX_SIN_LENGTH];
// int actual_waveform_length;

// // IMU globals
// const int MPU = 0x68; // MPU6050 I2C address
// float AccX, AccY, AccZ;
// int dropped_values = 0; // to track number of values not added to queue
// float time_between_steps;
// float user_cadence;

// // Critical section
// critical_section_t serial_lock;

// // RTOS type handles
// SemaphoreHandle_t printMutex;
// SemaphoreHandle_t userSpeedMutex;
// SemaphoreHandle_t waveformMutex;
// QueueHandle_t imuData;

// // FUNCTION DECLARATIONS

// // Sine array setup, called when change in cadence
// void generate_sin(float cadence) {
//   // Determine length of waveform, inverseley proportional to cadence
//   static float avg_walking_cadence = 1.66666;
//   static int avg_sin_len = 80;

//   if (cadence != 0){


//     // arbitrary equation i made up
//     uint16_t new_sin_len = avg_sin_len + (int)(20.0*(avg_walking_cadence - cadence));

//     // Generate new waveform
//     if (xSemaphoreTake(waveformMutex, 0)) {
//       for (int i = 0; i < new_sin_len; i++) {
//         float x = (2 * PI * i) / new_sin_len;
//         sin_array[i] = .2f * sin(x); // Sine wave, amplitude 2 radians
//       }
//       actual_waveform_length = new_sin_len;
//       xSemaphoreGive(waveformMutex);
//     }
//   } else {
//     if (xSemaphoreTake(waveformMutex, 0)) {
//       for (int i = 0; i < avg_sin_len; i++) {
//         float x = (2 * PI * i) / avg_sin_len;
//         sin_array[i] = .2f * sin(x); // Sine wave, amplitude 2 radians
//       }
//     actual_waveform_length = avg_sin_len;
//     xSemaphoreGive(waveformMutex);
//     }
//   }
// }


// // TASK DECLARATIONS
// void simpleFOC(void * params) {

//   // Counter and target angle variables init
//   int c = 0;
//   float target_torque = 0;
//   int wave_len = 80;

//   // initialize encoder sensor hardware
//   encoder.init();
//   // link the motor to the sensor
//   motor.linkSensor(&encoder);

//   // driver config
//   // power supply voltage [V]
//   driver.voltage_power_supply = 12;
//   driver.init();
//   // link driver
//   motor.linkDriver(&driver);
//   // link the driver to the current sense
//   current_sense.linkDriver(&driver);

//   // current sense init hardware
//   current_sense.init();
//   // link the current sense to the motor
//   motor.linkCurrentSense(&current_sense);

//   // set torque mode:
//   motor.torque_controller = TorqueControlType::foc_current; 
//   // set motion control loop to be used
//   motor.controller = MotionControlType::torque;

//   // foc current control parameters (Arduino UNO/Mega)
//   motor.PID_current_q.P = 5;
//   motor.PID_current_q.I= 300;
//   motor.PID_current_d.P= 5;
//   motor.PID_current_d.I = 300;
//   motor.LPF_current_q.Tf = 0.01; 
//   motor.LPF_current_d.Tf = 0.01; 

//   // use monitoring with serial 
//   Serial.begin(115200);
//   // comment out if not needed
//   motor.useMonitoring(Serial);

//   // initialize motor
//   motor.init();
//   // align sensor and start FOC
//   if(!motor.initFOC()){
//       //while(1){}
//   }
//   delay(1000);

//   // Create the initial sin array
//   generate_sin(0);
  
//   // Task while loop
//   while(1){

//     motor.loopFOC();
//     // read currents
//     float DC_current = current_sense.getDCCurrent(motor.electrical_angle);
//     float current_mag = current_sense.getDCCurrent();

//     float electricalangle = motor.electrical_angle;

//     DQCurrent_s foc_current = current_sense.getFOCCurrents(electricalangle);
//     float d_foc_current = foc_current.d;
//     float q_foc_current = foc_current.q;

//     // PhaseCurrent_s phase_current = current_sense.getPhaseCurrents();
//     // DQCurrent_s current = current_sense.getFOCCurrents(motor.electrical_angle);
//     // Set target angle from sine array

//     char ms[75];
//     sprintf(ms,">target_torque:%f\n>DCcurrentmag:%f\n>DCcurrent:%f\n>D_FOC_Current:%f\n>Q_FOC_Current:%f\n",target_torque,current_mag,DC_current,d_foc_current,q_foc_current);//phase_current.a,phase_current.b);
//     // Print string using mutex
//     if (xSemaphoreTake(printMutex, 0)) {
//       Serial.print(ms);
//       xSemaphoreGive(printMutex);
//     }

//     // Set target angle from sine array
//     if (xSemaphoreTake(waveformMutex, 0)) {
//       target_torque = sin_array[c];
//       wave_len = actual_waveform_length;
//       xSemaphoreGive(waveformMutex);
//     }

//     motor.move(target_torque);  // Tell motor to move to that angle

//     // Increment counter
//     c++;

//     // Check to loop back counter
//     if (c >= wave_len) c = 0;

//     // Delay to keep at a known frequency
//     vTaskDelay(20 / portTICK_PERIOD_MS);
//   }
// }

// void readIMU(void * params) {
//   // I2C init
//   Wire.begin();                      // Initialize comunication
//   Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
//   Wire.write(0x6B);                  // Talk to the register 6B
//   Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
//   Wire.endTransmission(true);        //end the transmission

//   // filtered value init
//   float smooth_data = .5;

//   while(1){
//     // === Read acceleromter data === //
//     Wire.beginTransmission(MPU);
//     Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
//     //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
//     AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
//     AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
//     AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

//     // Filter relevant value
//     // smooth_data = smooth_data - (.7 * (smooth_data - (AccX)));

//     // Save relevant value to imuData queue
//     if (xQueueSend(imuData, (void*)& AccX, 5) != pdTRUE) {
//       dropped_values++;
//       if (xSemaphoreTake(printMutex, 0)) {
//         Serial.printf(">droppedVals:%d\n",dropped_values);
//         xSemaphoreGive(printMutex);
//       }
//     }
    
//     // Create string to print the values on the serial monitor
//     char ms[50];
//     sprintf(ms,">ax:%f\n>ay:%f\n>az:%f\n", AccX, AccY, AccZ);
  
//     // Print string using mutex
//     if (xSemaphoreTake(printMutex, 0)) {
//       Serial.print(ms);
//       xSemaphoreGive(printMutex);
//     }
  
//     // Delay to keep loop running at known frequency
//     vTaskDelay(20 / portTICK_PERIOD_MS);
//   }
// }

// void cadenceDetection(void * params) {

// 	// step detection variables init
//   static uint8_t count = 0; 		// counter variable
// 	static float avg;				      // average of IMU values, to calculate peaks
// 	static bool ready = false;		// flag to indicate ready for next step
//   static float imu_val;          // imu value read in
//   static float imu_arr[DELAY];// array used to filter IMU data during step detection

//   // threasholding vals init
//   static float lower_threshold = 1;
//   static float upper_threshold = 2;
//   static int min_points_between_steps = 120; // likely number of IMU points between real steps
//   static int readings_per_second = 610;
  
//   while(1) {

//     // Get data from imuData queue
//     if (xQueueReceive(imuData, (void*)&imu_val,0) == pdTRUE) {
//       // Increment count
//       count++;
      
//       // save data
//       for (int ii = 0; ii < DELAY - 1; ii++) {
//         imu_arr[ii] = imu_arr[ii + 1]; // shift
//       }

//       // lower threasholding
//       imu_arr[DELAY - 1] = imu_val;
//       if (imu_arr[2] < lower_threshold) {
//         ready = true; // now it is ready for next step
//       }

//       // Step detection
//       if (imu_arr[1] > imu_arr[0] && imu_arr[2] < imu_arr[1] && ready == true) {
//         // Filter out bad steps (1/5 of a second apart, mistakes not running
//         if (count > min_points_between_steps) {
//           // if imu magnitude is big enough, it is a step
//           if (imu_arr[1] > upper_threshold) {

//             if (xSemaphoreTake(userSpeedMutex, 5)) {
//               time_between_steps = (float)(count) / readings_per_second;
//               user_cadence = 1 / time_between_steps;
//               count = 0;
//               ready = false; // reset trigger bool
//               xSemaphoreGive(userSpeedMutex);

//               // Generate new waveform
//               generate_sin(user_cadence);

//               // Print
//               if (xSemaphoreTake(printMutex, 0)) {
//                 Serial.printf(">user_cadence:%f\n",user_cadence);
//                 xSemaphoreGive(printMutex);
//               }
//             }            
//           }
//         }
//       }
//     }
//     vTaskDelay(20 / portTICK_PERIOD_MS);

//   }
// }


// // Initializes core 0
// void setup() {

//   // Initialize serial
//   Serial.begin(921600);

//   // Initialize mutexes
//   printMutex = xSemaphoreCreateMutex();   // controlls access to Serial
//   userSpeedMutex = xSemaphoreCreateMutex();   // controlls access to user speed information passed between cores

//   // Create the simpleFOC task - this is the only task running on this core0
//   xTaskCreate(
//     simpleFOC, 
//     "simpleFOC",
//     2048,
//     NULL,
//     7,
//     NULL
//   );

//   // Delete the loop task
//   vTaskDelete(NULL);
  
//   // Delay 1 second
//   delay(1000);
// }

// // Initializes core 1
// void setup1() {

//   // Initialize queues
//   imuData = xQueueCreate(IMU_Q_LEN, sizeof(float));
//   waveformMutex = xSemaphoreCreateMutex();   // controlls access to the trajectory

//   // Create the simpleFOC task - this is the only task running on this core0
//   xTaskCreate(
//     readIMU, 
//     "readIMU",
//     2048,
//     NULL,
//     1,
//     NULL
//   );
//   xTaskCreate(
//     cadenceDetection, 
//     "Cadence detection",
//     2048,
//     NULL,
//     1,
//     NULL
//   );

//   // Delete the loop task
//   vTaskDelete(NULL);
  
//   // Delay 1 second
//   delay(1000);
// }

// void loop(){}
// // void loop1(){}
