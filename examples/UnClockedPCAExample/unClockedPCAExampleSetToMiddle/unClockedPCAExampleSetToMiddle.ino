#include <Arduino.h>
#include <PCA_Driver.h>

//#include "Constants.h"

const uint8_t i2cAddress = 0x40;
const uint8_t PWM_CHANNELS = 16;

// These periods in seconds are based of the DIY MORE Extended Range Servo
const float MIN_PERIOD = 0.0005;
const float MID_PERIOD = 0.0015;
const float MAX_PERIOD = 0.0025;

// const float EXTERNAL_CLOCK = 50000000.0; //50MHz
const float PWM_FREQ = 329.9198; // Frequency Calculated from Prescaler math.

PCA9685* pcaController;

uint16_t pwm_min[PWM_CHANNELS];
uint16_t pwm_mid[PWM_CHANNELS];
uint16_t pwm_max[PWM_CHANNELS];
uint16_t pwm_12BitRange[PWM_CHANNELS];

uint16_t calculate12BitTicks(float inPeriod, PCA9685* pcaCont);

void setup()
{
    Serial.begin(115200);
    Wire.begin();
      delay(2000);

    Serial.println("Scanning...");

    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);
        }
    }

    Wire.end(); // closing wire device after verifying address.
    delay(100);

    pinMode(SDA, INPUT_PULLUP);  // Enable internal pull-up on SDA
    pinMode(SCL, INPUT_PULLUP);  // Enable internal pull-up on SCL
    
    pcaController = new PCA9685(i2cAddress); // Default 25MHz internal Clock
    pcaController->setPWMFrequency(PWM_FREQ); 

    delay(100);

    // Verify the chip is actually awake
    uint8_t mode = pcaController->readByte(0x00); // MODE1
    if (mode & 0x10) {
        Serial.println("Warning: PCA9685 is still in SLEEP mode!");
    } else {
        Serial.println("PCA9685 is awake and running.");
    }

    for (int i = 0; i < PWM_CHANNELS; ++i){
        pwm_min[i] = calculate12BitTicks(MIN_PERIOD, pcaController);
        pwm_mid[i] = calculate12BitTicks(MID_PERIOD, pcaController);
        pwm_max[i] = calculate12BitTicks(MAX_PERIOD, pcaController);
        pwm_12BitRange[i] = pwm_max[i] - pwm_min[i]; 
    }
    Serial.print("min: "); Serial.print(pwm_min[0]);
    Serial.print(", mid: "); Serial.print(pwm_mid[0]);
    Serial.print(", max: "); Serial.print(pwm_max[0]);
    Serial.print(", 12BitRange: "); Serial.print(pwm_12BitRange[0]);
    Serial.print(", Degree Step Size: "); Serial.println(180.0/((float)pwm_max[0]));
    pcaController->setAllPWM( 0, pwm_mid[0]);
    delay(1000);
}

void loop()
{
    // // Serial.println("Setting min");
    // pcaController->setPWM(0, 0, pwm_min[0]);
    // delay(1000);
    // // Serial.println("Setting mid");
    // pcaController->setPWM(0, 0, pwm_mid[0]);
    // delay(1000);
    // // Serial.println("Setting max");
    // pcaController->setPWM(0, 0, pwm_max[0]);
    // delay(1000);

    // pcaController->setPWM(0, 0, pwm_mid[0]);
    // delay(1000);
}

uint16_t calculate12BitTicks(float inPeriod, PCA9685* pcaCont)
{
    uint16_t pwm_ticks = ceil(inPeriod*pcaCont->getClock()/(pcaCont->getPrescale() + 1));
    if(pwm_ticks < 4096 - pcaCont->getPWMBias()){
        pwm_ticks += pcaCont->getPWMBias();
    } else {
        pwm_ticks = 4095;
    }
    //Serial.print("Ticks: "); Serial.println(pwm_ticks);
    return pwm_ticks;
}
