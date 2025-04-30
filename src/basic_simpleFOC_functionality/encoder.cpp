// #include <SimpleFOC.h>

// //VERIFIED ON ESP32

// // MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// // cs              - SPI chip select pin 
// // bit_resolution  - sensor resolution
// // angle_register  - (optional) angle read register - default 0x3FFF
// MagneticSensorSPI as5047u = MagneticSensorSPI(17, 14);

// void setup() {
//   // monitoring port
//   Serial.begin(115200);
//   pinMode(LED_BUILTIN, OUTPUT);

//   // initialise magnetic sensor hardware
//   as5047u.init();

//   Serial.println("as5047u ready");
//   _delay(1000);
// }

// void loop() {
//   // IMPORTANT - call as frequently as possible
//   // update the sensor values 
//   as5047u.update();
  
// //   digitalWrite(LED_BUILTIN, HIGH);
// //   delay(10);
// //   digitalWrite(LED_BUILTIN, LOW);
// //   delay(10);

//   // display the angle and the angular velocity to the terminal
//   Serial.print(as5047u.getAngle());
//   Serial.print("\t");
//   Serial.println(as5047u.getVelocity());
// }

