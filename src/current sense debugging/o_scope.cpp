#include <SimpleFOC.h>

#define PIN 14

// BLDCMotor(pole pair number, phase resistance (optional) );
// BLDCMotor motor = BLDCMotor(14,2.5);
BLDCMotor motor = BLDCMotor(14);
// BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(0,1,2,3);
// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
MagneticSensorSPI encoder = MagneticSensorSPI(17, 14);
// InlineCurrentSense current_sense  = InlineCurrentSense(1.0f, 50.0f,26,27,_NC);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() { 
  

 Serial.begin(115200);

 SimpleFOCDebug::enable(&Serial);
 driver.voltage_power_supply = 12;
 driver.voltage_limit = 10;
 if(!driver.init()){
   Serial.println("Driver init failed!");
   return;
 }

 motor.linkDriver(&driver);
 motor.voltage_limit = 10;   // [V]
 motor.controller = MotionControlType::velocity_openloop;
 if(!motor.init()){
   Serial.println("Motor init failed!");
   return;
 }
 motor.target = 1.57;

delay(1000);
}



void loop() {

    while(1){
        motor.loopFOC();
        motor.move();
    }

}

void setup1() {
    analogReadResolution(12);
}

void loop1() {
    char ms[100];
    int adc1 = 26;
    int adc2 = 27;
    int adc3 = 28;
    bool val = true;
    
    while(1){
        // gpio_put(PIN, true);
        // _delay(100);
        // gpio_put(PIN, false);
        // _delay(100);

        int analogValue1 = analogRead(adc1);
        int analogValue2 = analogRead(adc2);
        int analogValue3 = analogRead(adc3);
        float voltage1 = analogValue1 * 3.3 / 4096.0;  // Assuming 12-bit ADC, adjust if needed
        float voltage2 = analogValue2 * 3.3 / 4096.0;  // Assuming 12-bit ADC, adjust if needed
        float voltage3 = analogValue3 * 3.3 / 4096.0;  // Assuming 12-bit ADC, adjust if needed
        sprintf(ms,">adc1:%f\n>adc2:%f\n>adc3:%f\n",voltage1,voltage2,voltage3);
        Serial.print(ms);
        
    }
}