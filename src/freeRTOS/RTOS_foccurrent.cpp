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

#define MAX_SIN_LENGTH 1000

// GLOBAL VARIABLES]

// Motor and driver setup
BLDCMotor motor = BLDCMotor(14,3);
BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);
InlineCurrentSense current_sense  = InlineCurrentSense(0.01, 50.0, 26,27, _NC);

// Sin array 
float sin_array[MAX_SIN_LENGTH];
int actual_waveform_length;

// IMU globals
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
int dropped_values = 0; // to track number of values not added to queue
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
  static int avg_sin_len = MAX_SIN_LENGTH;

  if (xSemaphoreTake(waveformMutex, 0)) {
      for (int i = 0; i < int(avg_sin_len/2); i++) {
        sin_array[i] = -0.5;
      }
      for (int i = int(avg_sin_len/2); i < avg_sin_len; i++) {
        sin_array[i] = 0.5;

      }
    actual_waveform_length = MAX_SIN_LENGTH;
    xSemaphoreGive(waveformMutex);
  }
  
}


// TASK DECLARATIONS
void simpleFOC(void * params) {


  // SimpleFOC init

  // Counter and target angle variables init
  int c = 0;
  float target_torque = 0;
  int wave_len = MAX_SIN_LENGTH;

  encoder.init();
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = 22;
  driver.init();
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;   
  motor.PID_current_q.P = 1;
  motor.PID_current_q.I= 2;
  motor.PID_current_d.P= 1;
  motor.PID_current_d.I = 2;
  motor.LPF_current_q.Tf = 0.01; 
  motor.LPF_current_d.Tf = 0.01; 
  motor.useMonitoring(Serial);
  motor.init();
  if(!motor.initFOC()){
      //while(1){}
  }

  // Timer init
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1);
  xLastWakeTime = xTaskGetTickCount();
  TickType_t start;
  TickType_t end;

  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // Create the initial sin array
  generate_sin(0);

  // Task while loop
  while(1){
    // start = xTaskGetTickCount();

    motor.loopFOC();
    // read currents
  

    float electricalangle = motor.electrical_angle;

    float DC_current = current_sense.getDCCurrent(electricalangle);
    float current_mag = current_sense.getDCCurrent();

    DQCurrent_s foc_current = current_sense.getFOCCurrents(electricalangle);
    float d_foc_current = foc_current.d;
    float q_foc_current = foc_current.q;

    if (xSemaphoreTake(waveformMutex, 0)) {
      target_torque = sin_array[c];
      wave_len = MAX_SIN_LENGTH;
      xSemaphoreGive(waveformMutex);
    }
    motor.move(target_torque);

    char ms[100];

    sprintf(ms,">target_torque:%f\n>DCcurrentmag:%f\n>DCcurrent:%f\n>D_FOC_Current:%f\n>Q_FOC_Current:%f\n",target_torque,current_mag,DC_current,d_foc_current,q_foc_current);//phase_current.a,phase_current.b);
    // Print string using mutex
    if (xSemaphoreTake(printMutex, 0)) {
      Serial.print(ms);
      xSemaphoreGive(printMutex);
    }


    c++;
    if (c >= wave_len) c = 0;

    // end = xTaskGetTickCount();
    // Serial.println(end-start);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Initializes core 0
void setup() {

  // Initialize serial
  Serial.begin(115200);

  // Initialize mutexes
  printMutex = xSemaphoreCreateMutex();   // controlls access to Serial
  userSpeedMutex = xSemaphoreCreateMutex();   // controlls access to user speed information passed between cores
  waveformMutex = xSemaphoreCreateMutex();   // controlls access to the trajectory

  // Create the simpleFOC task - this is the only task running on this core0
  xTaskCreate(
    simpleFOC, 
    "simpleFOC",
    2048,
    NULL,
    2,
    NULL
  );

  // Delete the loop task
  vTaskDelete(NULL);
  
  // Delay 1 second
  delay(1000);
}

void loop() {}
