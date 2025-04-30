// #include <SimpleFOC.h>
// #include "hardware/timer.h"
// #include <Wire.h>



// #define I2C_SDA 21
// #define I2C_SCL 22
// #define MPU6050 0x68

// const int MPU = 0x68; // MPU6050 I2C address
// float AccX, AccY, AccZ;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();                      // Initialize comunication
//   Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
//   Wire.write(0x6B);                  // Talk to the register 6B
//   Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
//   Wire.endTransmission(true);        //end the transmission
//   delay(20);
// }
// void loop() {
//   // === Read acceleromter data === //
//   Wire.beginTransmission(MPU);
//   Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
//   //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
//   AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
//   AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
//   AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
//   // Print the values on the serial monitor
//   char ms[50];
//   sprintf(ms,">ax:%f\n>ay:%f\n>az:%f\n", AccX, AccY, AccZ);
//   Serial.print(ms);


//   _delay(10);
// }
