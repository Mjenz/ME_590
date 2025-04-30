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

// These things
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6050 0x68

#define SIN_LENGTH 50

// GLOBAL VARIABLES]

// Motor and driver setup
BLDCMotor motor = BLDCMotor(14,3);
BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);

// Sin array 
float sin_array[SIN_LENGTH];

// IMU globals
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;

// Critical section
critical_section_t serial_lock;

// Mutex handles
SemaphoreHandle_t printMutex;


// FUNCTION DECLARATIONS

// Sine array setup
void generate_sin(float * array, int length) {
  for (int i = 0; i < length; i++) {
    float x = (2 * PI * i) / length;
    array[i] = 0.2f * sin(x); // Sine wave, amplitude 2 radians
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

  // Create the sin array
  generate_sin(sin_array, SIN_LENGTH);
  
  // Task while loop
  while(1){
      motor.loopFOC();

      // Create string to print target and actual angle
      char ms[50];
      sprintf(ms,">target_angle:%f\n>actual:%f\n", target_angle, encoder.getAngle());
      
      // Print string using mutex
      if (xSemaphoreTake(printMutex, 0)) {
        Serial.print(ms);
        xSemaphoreGive(printMutex);
      }

      // Set target angle from sine array
      target_angle = sin_array[c];

      // Tell motor to move to target angle
      motor.move(target_angle);  

      // Increment counter
      c++;

      // Check to loop back counter
      if (c >= SIN_LENGTH) c = 0;

      // Delay to keep at a known frequency
      delay(20); 
  }
}

void readIMU(void * params) {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
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
    
    // Create string to print the values on the serial monitor
    char ms[50];
    sprintf(ms,">ax:%f\n>ay:%f\n>az:%f\n", AccX, AccY, AccZ);
  
    // Print string using mutex
    if (xSemaphoreTake(printMutex, 0)) {
      Serial.print(ms);
      xSemaphoreGive(printMutex);
    }
  
    // Delay to keep loop running at known frequency
    delay(20);
    }
}

// Initializes core 0
void setup() {

  // Initialize serial
  Serial.begin(921600);

  // Initialize mutexes
  printMutex = xSemaphoreCreateMutex();   // controlls access to Serial

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

  // Create the simpleFOC task - this is the only task running on this core0
  xTaskCreate(
    readIMU, 
    "readIMU",
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
