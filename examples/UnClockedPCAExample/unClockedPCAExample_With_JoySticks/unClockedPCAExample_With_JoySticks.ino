#include <Arduino.h>
#include <PCA_Driver.h>

// Oldschool C style Macro Defines
#define X1_PIN A0
#define Y1_PIN A1
#define X2_PIN A2
#define Y2_PIN A3
#define SWITCH1_PIN D4
#define SWITCH2_PIN D2


// New School C++ style global constants
const uint8_t i2cAddress = 0x40;
const uint8_t PWM_CHANNELS = 6;

// These periods in seconds are based of the DIY MORE Extended Range Servo
const float MIN_PERIOD = 0.0005;
const float MID_PERIOD = 0.0015;
const float MAX_PERIOD = 0.0025;

const float EXTERNAL_CLOCK = 25000000.0; //25MHz
const float PWM_FREQ = 329.9198; // Frequency Calculated from Prescaler math.



// Global Variables
PCA9685* pcaController; // PWM Controller pointer


// These arrays are the min, mid and max ticks for the Servos
// min represents 0 degrees, mid represents 90 degrees max represents 180 degrees.
// These values depend on chosen PWM Frequency
// The one servo which needs it's min and max values tweaked is the claw servo
// as it's range is mechanically constrained.

uint16_t pwm_min[PWM_CHANNELS];
uint16_t pwm_mid[PWM_CHANNELS];
uint16_t pwm_max[PWM_CHANNELS];

uint16_t pwm_12BitRange[PWM_CHANNELS]; // this Array is for range in 12bit ticks
uint16_t pwm_pos[PWM_CHANNELS]; // This Array is for current servo position

// Function Prototypes
uint16_t calculate12BitTicks(float inPeriod, PCA9685* pcaCont);
void set_pos(uint16_t axis_value, uint8_t channel);

void setup()
{
    Serial.begin(115200);
    Wire.begin(); // Need For I2C
    delay(2000);
    digitalWrite(SDA, 1); // Enable Arduiono 50kohm pullup resistor on SDA
    digitalWrite(SCL, 1); // Enable Arduiono 50kohm pullup resistor on SCL

    Serial.println("Scanning...");


    // This For Loop just reports i2c devices found
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);
        }
    }

    Wire.end(); // closing wire device after verifying address.
    delay(100);


    analogReadResolution(10);

    pinMode(SWITCH1_PIN, INPUT);
    pinMode(SWITCH2_PIN, INPUT);
    digitalWrite(SWITCH1_PIN, HIGH);
    digitalWrite(SWITCH2_PIN, HIGH);

    pcaController = new PCA9685(i2cAddress); // set to 25MHz Internal Clock
    pcaController->setPWMFrequency(PWM_FREQ);  // Frequency Calculated from Prescaler math.

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
      pwm_pos[i] = calculate12BitTicks(MID_PERIOD, pcaController);
      pwm_12BitRange[i] = pwm_max[i] - pwm_min[i]; 
  }

  Serial.print("min: "); Serial.print(pwm_min[0]);
  Serial.print(", mid: "); Serial.print(pwm_mid[0]);
  Serial.print(", max: "); Serial.print(pwm_max[0]);
  Serial.print(", 12BitRange: "); Serial.print(pwm_12BitRange[0]);
  Serial.print(", Degree Step Size: "); Serial.println(180.0/((float)pwm_max[0]));

  pcaController->setAllPWM(0, pwm_mid[0]);
  delay(1000);
}

void loop()
{
  // Read Analogue Pins
  uint16_t x1 = analogRead(X1_PIN);
  uint16_t y1 = analogRead(Y1_PIN);
  uint16_t x2 = analogRead(X2_PIN);
  uint16_t y2 = analogRead(Y2_PIN);

  set_pos(x1, 0);
  set_pos(y1, 1);
  set_pos(x2, 2);
  set_pos(y2, 3);
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

void set_pos(uint16_t axis_value, uint8_t channel){
  if(pwm_pos[channel] <= pwm_max[channel]){
    if(axis_value > 900){
      pwm_pos[channel] += 16;
    } else if( axis_value > 800){
      pwm_pos[channel] += 8;
    } else if(axis_value > 700){
      pwm_pos[channel] += 4;
    } else if(axis_value > 600){
      pwm_pos[channel]++;
    }
    if(pwm_pos[channel] > pwm_max[channel]) pwm_pos[channel] = pwm_max[channel];
  }

  if(pwm_pos[channel] >= pwm_min[channel]){
    if(axis_value < 100){
      pwm_pos[channel] -= 16;
    } else if( axis_value < 200){
      pwm_pos[channel] -= 8;
    } else if(axis_value < 300){
      pwm_pos[channel] -= 4;
    } else if(axis_value < 400){
      pwm_pos[channel]--;
    }
    if(pwm_pos[channel] < pwm_min[channel]) pwm_pos[channel] = pwm_min[channel];
  }
  pcaController->setPWM(channel, 0, pwm_pos[channel]);
}
