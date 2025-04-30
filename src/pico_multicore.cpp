#include <SimpleFOC.h>

#include <Wire.h>

#include <math.h>
#include <string>
#include <iostream>

#include "hardware/timer.h"
#include "pico/critical_section.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
// These things
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6050 0x68
#define IMU_Q_LEN 20
#define DELAY 25

#define MAX_SIN_LENGTH 300

// GLOBAL VARIABLES]

// Motor and driver setup
BLDCMotor motor = BLDCMotor(14,3);
BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);

// Sin array 
float sin_array[MAX_SIN_LENGTH];
uint8_t actual_waveform_length;

// IMU globals
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
uint8_t dropped_values = 0; // to track number of values not added to queue
float time_between_steps;
float user_cadence;

// Critical section
critical_section_t serial_lock;

// RTOS type handles
SemaphoreHandle_t printMutex;
SemaphoreHandle_t userSpeedMutex;
SemaphoreHandle_t waveformMutex;
QueueHandle_t imuData;

// FUNCTION DECLARATIONS

// Sine array setup, called when change in cadence
void generate_sin(float cadence) {
  // Determine length of waveform, inverseley proportional to cadence
  static float avg_walking_cadence = 1.66666;
  static uint16_t avg_sin_len = 80;

  // arbitrary equation i made up
  uint16_t new_sin_len = avg_sin_len + (int)(20.0*(avg_walking_cadence - cadence));

  for (int i = 0; i < length; i++) {
    float x = (2 * PI * i) / length;
    array[i] = .2f * sin(x); // Sine wave, amplitude 2 radians
  }
}


// TASK DECLARATIONS
void simpleFOC(void * params) {

  // Counter and target angle variables init
  int c = 0;
  float target_angle = 0;

  // Critical Section init
  critical_section_init(&serial_lock);

  // SimpleFOC init
  encoder.init();
  motor.linkSensor(&encoder);

  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.025; // was .5
  motor.PID_velocity.I = 1; // was 20
  motor.PID_velocity.D = 0.0001; // was 0.001
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.001;// was 0.01
  motor.P_angle.P = 20;
  motor.velocity_limit = 4;
  motor.voltage_limit = 8;

  // motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  Serial.println("Motor ready.");

  // Create the initial sin array
  generate_sin(sin_array, 50);
  
  // Task while loop
  while(1){
      motor.loopFOC();

      // Create string to print target and actual angle
      char ms[50];
      sprintf(ms,">target_angle:%f\n>actual:%f\n", target_angle, encoder.getAngle());
      
      // Print string using mutex
      if (xSemaphoreTake(printMutex, 0)) {
        // Serial.print(ms);
        xSemaphoreGive(printMutex);
      }

      // Set target angle from sine array
      target_angle = sin_array[c];

      // Tell motor to move to target angle
      motor.move(target_angle);  

      // Increment counter
      c++;

      // Check to loop back counter
      if (c >= actual_waveform_length) c = 0;

      // Delay to keep at a known frequency
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void readIMU(void * params) {
  // I2C init
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // filtered value init
  float smooth_data = .5;

  while(1){
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

    // Filter relevant value
    // smooth_data = smooth_data - (.7 * (smooth_data - (AccX)));

    // Save relevant value to imuData queue
    if (xQueueSend(imuData, (void*)& AccX, 5) != pdTRUE) {
      dropped_values++;
      if (xSemaphoreTake(printMutex, 0)) {
        Serial.printf(">droppedVals:%d\n",dropped_values);
        xSemaphoreGive(printMutex);
      }
    }
    
    // Create string to print the values on the serial monitor
    char ms[50];
    sprintf(ms,">ax:%f\n>ay:%f\n>az:%f\n", AccX, AccY, AccZ);
  
    // Print string using mutex
    if (xSemaphoreTake(printMutex, 0)) {
      // Serial.print(ms);
      xSemaphoreGive(printMutex);
    }
  
    // Delay to keep loop running at known frequency
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void cadenceDetection(void * params) {

	// step detection variables init
  static uint8_t count = 0; 		// counter variable
	static float avg;				      // average of IMU values, to calculate peaks
	static bool ready = false;		// flag to indicate ready for next step
  static float imu_val;          // imu value read in
  static float imu_arr[DELAY];// array used to filter IMU data during step detection

  // threasholding vals init
  static float lower_threshold = 1;
  static float upper_threshold = 2;
  static int min_points_between_steps = 120; // likely number of IMU points between real steps
  static int readings_per_second = 610;
  
  while(1) {

    // Get data from imuData queue
    if (xQueueReceive(imuData, (void*)&imu_val,0) == pdTRUE) {
      // Increment count
      count++;
      Serial.printf(">x:%f\n",imu_val);
      // save data
      for (int ii = 0; ii < DELAY - 1; ii++) {
        imu_arr[ii] = imu_arr[ii + 1]; // shift
      }

      // lower threasholding
      imu_arr[DELAY - 1] = imu_val;
      if (imu_arr[2] < lower_threshold) {
        ready = true; // now it is ready for next step
      }

      // Step detection
      if (imu_arr[1] > imu_arr[0] && imu_arr[2] < imu_arr[1] && ready == true) {
        // Filter out bad steps (1/5 of a second apart, mistakes not running
        if (count > min_points_between_steps) {
          // if imu magnitude is big enough, it is a step
          if (imu_arr[1] > upper_threshold) {

            if (xSemaphoreTake(userSpeedMutex, 5)) {
              time_between_steps = (float)(count) / readings_per_second;
              user_cadence = 1 / time_between_steps;
              count = 0;
              ready = false; // reset trigger bool
              xSemaphoreGive(userSpeedMutex);

              // Generate new waveform
              generate_sin(user_cadence);

              // Print
              if (xSemaphoreTake(printMutex, 0)) {
                Serial.printf(">user_cadence:%f\n",user_cadence);
                xSemaphoreGive(printMutex);
              }
            }            
          }
        }
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);

  }
}


// Initializes core 0
void setup() {

  // Initialize serial
  Serial.begin(921600);

  // Initialize mutexes
  printMutex = xSemaphoreCreateMutex();   // controlls access to Serial
  userSpeedMutex = xSemaphoreCreateMutex();   // controlls access to user speed information passed between cores

  // Create the simpleFOC task - this is the only task running on this core0
  xTaskCreate(
    simpleFOC, 
    "simpleFOC",
    2048,
    NULL,
    7,
    NULL
  );

  // Delete the loop task
  vTaskDelete(NULL);
  
  // Delay 1 second
  delay(1000);
}

// Initializes core 1
void setup1() {

  // Initialize queues
  imuData = xQueueCreate(IMU_Q_LEN, sizeof(float));
  waveformMutex = xSemaphoreCreateMutex();   // controlls access to the trajectory

  // Create the simpleFOC task - this is the only task running on this core0
  xTaskCreate(
    readIMU, 
    "readIMU",
    2048,
    NULL,
    1,
    NULL
  );
  xTaskCreate(
    cadenceDetection, 
    "Cadence detection",
    2048,
    NULL,
    1,
    NULL
  );

  // Delete the loop task
  vTaskDelete(NULL);
  
  // Delay 1 second
  delay(1000);
}

void loop(){}
// void loop1(){}
