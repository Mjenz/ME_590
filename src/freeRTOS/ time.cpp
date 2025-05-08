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


// void Task1code(void *pvParameters) {
//     TickType_t xLastWakeTime;
//     TickType_t prev = 0;
//     TickType_t now;

//     const TickType_t xFrequency = pdMS_TO_TICKS(1); // 10 ms = 100 Hz

//     // Initialize the xLastWakeTime variable with the current time.
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;) {
//         // Do something
//         now = xTaskGetTickCount();
//         Serial.println(now-prev);
//         prev = now;

//         // to break things
//         // int c = 0;
//         // while(c <1000000) c++;

//         // Wait for the next cycle.
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }
  


// // Initializes core 0
// void setup() {

//     // Initialize serial
//     Serial.begin(921600);

//     // Create the simpleFOC task - this is the only task running on this core0
//     xTaskCreate(
//     Task1code, 
//       "Task1code",
//       2048,
//       NULL,
//       7,
//       NULL
//     );
  
//     // Delete the loop task
//     vTaskDelete(NULL);
    
//     // Delay 1 second
//     delay(1000);
//   }

//   void loop() {}