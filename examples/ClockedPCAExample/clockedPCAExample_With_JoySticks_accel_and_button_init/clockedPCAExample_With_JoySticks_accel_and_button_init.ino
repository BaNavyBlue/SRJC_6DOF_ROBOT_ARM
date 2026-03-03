#include <Arduino.h>
#include <PCA_Driver.h>
#include <Mx2125.h>
#include <FspTimer.h>
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include "TextAnimation.h"

// Oldschool C style Macro Defines
#define X1_PIN A0
#define Y1_PIN A1
#define X2_PIN A2
#define Y2_PIN A3
#define SWITCH1_PIN D4
#define SWITCH2_PIN D2
#define ACC_XAXIS 5
#define ACC_YAXIS 6

#define ROTATE_BASE 0
#define ARM_SEG1 1
#define ARM_SEG2 2
#define ARM_SEG3 3
#define WRIST 4
#define CLAW 5

#define LARGE_STEP 8
#define MED_STEP 4
#define SMALL_STEP 2

// New School C++ style global constants
const uint8_t i2cAddress = 0x40;
const uint8_t PWM_CHANNELS = 6;

const int PWM_BIAS = 45;

// These periods in seconds are based of the DIY MORE Extended Range Servo
const float MIN_PERIOD = 0.0005;
const float MID_PERIOD = 0.0015;
const float MAX_PERIOD = 0.00258;
const float ANGLE_RANGE = 188.0;

const uint16_t CLAW_MIN_TICKS = 1740;
const uint16_t CLAW_MAX_TICKS = 2842;

const float EXTERNAL_CLOCK = 50000000.0; //50MHz
const float PWM_FREQ = 329.9198; // Frequency Calculated from Prescaler math.
// const float PWM_FREQ = 50.0;



// Global Variables
Mx2125  Accelerometer(ACC_XAXIS, ACC_YAXIS, A5);
PCA9685* pcaController; // PWM Controller pointer
ArduinoLEDMatrix matrix;
TEXT_ANIMATION_DEFINE(anim, 100)

char serialMsg[256];
char matMsg[32];
// Stuff for Interupt timer
FspTimer Timer;
volatile uint8_t button1 = 1;
volatile uint8_t button2 = 1;
volatile int button1_counts = 0; // Shared variable
volatile int button2_counts = 0;
volatile uint8_t b1_mode = 0;
volatile uint8_t b2_mode = 0;
volatile bool requestNext = false;

// setting these wrong so the mode led's will be set upon first main loop
uint8_t b1_prev = 1;
uint8_t b2_prev = 1;

void IRQ_HIT(timer_callback_args_t *p_args) {
    if(!button1){
      if(button1_counts >= 2){
        b1_mode = !b1_mode;
        button1_counts = -20;
      } else {
        button1_counts++;
      }
    } else {
      button1_counts = 0;
    }

    if(!button2){
      if(button2_counts >= 2){
        b2_mode = !b2_mode;
        button2_counts = -20;
      } else {
        button2_counts++;
      }
    } else {
      button2_counts = 0;
    }
}

// This is only really for scrolling text that doesn't slow down the main loop.
void matrixCallback() {
  // callback is executed in IRQ and should run as fast as possible
  requestNext = true;
}


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
void find_starting_angle_and_home(void);
void set_mode_leds(void);
float calculated_angle(uint8_t channel);

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // Need For I2C
  delay(2000);

  uint8_t timerType = GPT_TIMER;
  int8_t channel = FspTimer::get_available_timer(timerType);
  
  if (channel < 0) {
    // Handle error if no timer is available
    Serial.println("Timer Channel Error!");
    while (1);
  }

  bool ok = Timer.begin(TIMER_MODE_PERIODIC, timerType, channel, 100.0, 50.0, &IRQ_HIT); // 100 Hz, 50% duty cycle (though duty cycle is less relevant for simple periodic interrupt)
  if (!ok) {
    // Handle error if timer init failed
    Serial.println("Your timer is fail!");
    while (1);
  }


  Timer.setup_overflow_irq(); // Connects the timer event to the CPU ISR
  Timer.open(); // Required to make it work
  Timer.start();

  digitalWrite(SDA, 1); // Enable Arduiono 50kohm pullup resistor on SDA
  digitalWrite(SCL, 1); // Enable Arduiono 50kohm pullup resistor on SCL

  matrix.begin();

  matrix.beginDraw();

  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(100);
  matrix.setCallback(matrixCallback);

  sprintf(matMsg, "Scanning...");
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(matMsg);
  matrix.endTextAnimation(SCROLL_LEFT, anim);
  
  matrix.loadTextAnimationSequence(anim);
  matrix.play();
  // Serial.println("Scanning...");
  sprintf(matMsg, "Found device at ");

  // This For Loop just reports i2c devices found
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      //matrix.endDraw();
      char found[16];
      sprintf(found, " 0x%x", addr);
      strcat(matMsg, found);
      // Serial.print("Found device at 0x");
      // Serial.println(addr, HEX);

    }
  }
  // matrix.textFont(Font_4x6);
  matrix.endDraw();
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(matMsg);
  matrix.endTextAnimation(SCROLL_LEFT, anim);

  matrix.loadTextAnimationSequence(anim);
  matrix.play();

  delay(10000); // Just here to see "Found Message"
  matrix.endDraw();
  analogReadResolution(10);

  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

  pcaController = new PCA9685(i2cAddress, EXTERNAL_CLOCK); // set to 50MHz External Clock
  pcaController->sleepPCA();
  pcaController->setPWMFrequency(PWM_FREQ);  // Frequency Calculated from Prescaler math.
  pcaController->setPWMBias(PWM_BIAS);
  //pcaController->setAllPWM(0, 0);
  for (int i = 0; i < PWM_CHANNELS; ++i){
      pwm_min[i] = calculate12BitTicks(MIN_PERIOD, pcaController);
      pwm_mid[i] = calculate12BitTicks(MID_PERIOD, pcaController);
      pwm_max[i] = calculate12BitTicks(MAX_PERIOD, pcaController);
      pwm_pos[i] = calculate12BitTicks(MID_PERIOD, pcaController);
      pwm_12BitRange[i] = pwm_max[i] - pwm_min[i]; 
  }
  pwm_min[5] = CLAW_MIN_TICKS;
  pwm_max[5] = CLAW_MAX_TICKS;

  Serial.print("min: "); Serial.print(pwm_min[0]);
  Serial.print(", mid: "); Serial.print(pwm_mid[0]);
  Serial.print(", max: "); Serial.print(pwm_max[0]);
  Serial.print(", 12BitRange: "); Serial.print(pwm_12BitRange[0]);
  Serial.print(", Degree Step Size: "); Serial.println(ANGLE_RANGE/((float)pwm_12BitRange[0]));


  delay(1000);
  find_starting_angle_and_home();
  delay(1000);
  
}

uint16_t prev_pos = 0;

void loop()
{
  // Read Analogue Pins
  uint16_t x1 = analogRead(X1_PIN);
  uint16_t y1 = analogRead(Y1_PIN);
  uint16_t x2 = analogRead(X2_PIN);
  uint16_t y2 = analogRead(Y2_PIN);
  button1 = digitalRead(SWITCH1_PIN);
  button2 = digitalRead(SWITCH2_PIN);

  set_pos(x1, ROTATE_BASE);
  switch(b1_mode){
    case 0:
      set_pos(y1, ARM_SEG1);
      break;
    case 1:
      set_pos(y1, ARM_SEG3);
      break;
  }
  set_pos(x2, WRIST);

  switch(b2_mode){
    case 0:
      set_pos(y2, ARM_SEG2);
      break;
    case 1:
      set_pos(y2, CLAW);
      break;
  }


  

  /* Uncomment to Demonstrate how much Serial Print commands slow down controls */ 
  // prev_pos = pwm_pos[1];
  // if(prev_pos != pwm_pos[1]){
  //   uint16_t angle = Accelerometer.mx_rotation();
  //   Serial.print("Angle: "); Serial.print(angle);
  //   Serial.print( ", current ticks: "); Serial.print(pwm_pos[1]);
  //   Serial.print(", Calculated Angle: "); Serial.println((ANGLE_RANGE/pwm_12BitRange[1])*(pwm_pos[1] - pwm_min[1]));
  // }


  /* This code is much faster than doing multiple Serial.print commands */
  // sprintf(serialMsg, "pos[0]: %u, pos[1]: %u, pos[2]: %u, pos[3]: %u, pos[4]: %u, pos[5]: %u\n", pwm_pos[0], pwm_pos[1], pwm_pos[2], pwm_pos[3], pwm_pos[4], pwm_pos[5]);
  // Serial.print(serialMsg);
  if(b1_prev != b1_mode || b2_prev != b2_mode){
    set_mode_leds();
    b1_prev = b1_mode;
    b2_prev = b2_mode;
  }

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
      pwm_pos[channel] += LARGE_STEP;
    } else if( axis_value > 800){
      pwm_pos[channel] += MED_STEP;
    } else if(axis_value > 700){
      pwm_pos[channel] += SMALL_STEP;
    } else if(axis_value > 600){
      pwm_pos[channel]++;
    }
    if(pwm_pos[channel] > pwm_max[channel]) pwm_pos[channel] = pwm_max[channel];
  }

  if(pwm_pos[channel] >= pwm_min[channel]){
    if(axis_value < 100){
      pwm_pos[channel] -= LARGE_STEP;
    } else if( axis_value < 200){
      pwm_pos[channel] -= MED_STEP;
    } else if(axis_value < 300){
      pwm_pos[channel] -= SMALL_STEP;
    } else if(axis_value < 400){
      pwm_pos[channel]--;
    }
    if(pwm_pos[channel] < pwm_min[channel]) pwm_pos[channel] = pwm_min[channel];
  }
  pcaController->setPWM(channel, 0, pwm_pos[channel]);
}


void find_starting_angle_and_home(void){
    uint16_t angle = Accelerometer.mx_rotation();
    if( angle > 270){
      angle = 0;
    } else if ( angle > 180){
      angle = 180;
    }
    pwm_pos[1] = pwm_min[1] + lround(angle/(ANGLE_RANGE/pwm_12BitRange[1]));
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" what are ticks: "); Serial.println(pwm_pos[1]);
    pcaController->setPWM(1, 0, pwm_pos[1]);
    pcaController->wakePCA();

    sprintf(matMsg, "PWR & B1");

    do{
      if(requestNext){
        requestNext = false;
        matrix.beginText(0, 1, 0xFFFFFF);
        matrix.println(matMsg);
        // matrix.endText(SCROLL_LEFT);
        matrix.endTextAnimation(SCROLL_LEFT, anim);
    
        matrix.loadTextAnimationSequence(anim);
        matrix.play();
      }
    }while(digitalRead(SWITCH1_PIN));

    matrix.endDraw();
    int calc_angle = lround(calculated_angle(1));
    while(calc_angle != 135){
      if(calc_angle < 135){
        pwm_pos[1] += 1;
      } else {
        pwm_pos[1] -= 1;
      }
      pcaController->setPWM(1, 0, pwm_pos[1]);
      calc_angle = lround(calculated_angle(1));

    }
}

float calculated_angle(uint8_t channel){
  return (ANGLE_RANGE/pwm_12BitRange[channel])*(pwm_pos[channel] - pwm_min[channel]);
}

void set_mode_leds(void){
  sprintf(matMsg, "%u|%u", b1_mode, b2_mode);
        matrix.beginText(0, 1, 0xFFFFFF);
        matrix.println(matMsg);
        // matrix.endText(SCROLL_LEFT);
        matrix.endText();
        matrix.endDraw();
}
