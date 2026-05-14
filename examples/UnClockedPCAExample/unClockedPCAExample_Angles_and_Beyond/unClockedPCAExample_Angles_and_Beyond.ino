/*
    Author: Antone Bajor With the Exceptions of Garret Hardisty's Chat GPT Dance Code as marked.
    Location: Santa Rosa Jr. College Mechatronics.
*/

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

#define PICKUP_0 112.578
#define PICKUP_1 59.608170
#define PICKUP_2 145.57
#define PICKUP_3 188
#define PICKUP_4 10.281758
#define PICKUP_5 74.705437

#define DROP_0 105.9411
#define DROP_1 141.536865
#define DROP_2 40.28107
#define DROP_3 16.593977
#define DROP_4 10.281758
#define DROP_5 150.386993

#define PRONE_0 105.811012
#define PRONE_1 180.841812
#define PRONE_2 6.247145
#define PRONE_3 188.0
#define PRONE_4 16.919350

#define MODES 4

// New School C++ style global constants
const uint8_t i2cAddress = 0x40;
const uint8_t PWM_CHANNELS = 6;

const int PWM_BIAS = 45;

// These periods in seconds are based of the DIY MORE Extended Range Servo
// Many other servos range is 1ms - 2ms with 1.5ms as the center.
// Note the range is in seconds.
const float MIN_PERIOD = 0.0005;
const float MID_PERIOD = 0.0015;
const float MAX_PERIOD = 0.00258;  // I added the extra .08ms you may want to remove.
const float ANGLE_RANGE = 188.0;

const float CLAW_MIN_DEGREES = 71.0; //1740;
const float CLAW_MAX_DEGREES = 144.0; //2842;

const float EXTERNAL_CLOCK = 25000000.0; //25MHz
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


/* Start CHAT GPT BLOCK*/
// DANCE MODE
// Hold BOTH joystick buttons for about 0.8 sec to start/stop the dance routine.
bool autoRepeatMode = false;  // true = dance mode is running
bool bothButtonsWereHeld = false;
unsigned long bothButtonsStartMs = 0;
const unsigned long BOTH_BUTTON_HOLD_MS = 800;

uint8_t autoStep = 0;
bool autoStepStarted = false;
unsigned long autoStepStartMs = 0;

const uint8_t AUTO_SPEED = 9;       // smaller = smoother/slower, larger = faster
const unsigned long AUTO_PAUSE_MS = 120;

// Claw auto positions use the actual calibrated claw PWM range, not generic arm degrees.
// If the claw opens/closes backwards, swap these two values.
const uint8_t CLAW_AUTO_OPEN_PERCENT = 100;
const uint8_t CLAW_AUTO_CLOSED_PERCENT = 15;
/* End CHAT GPT BLOCK */

/* This is an interrupt timer for handling button push events*/
void IRQ_HIT(timer_callback_args_t *p_args) {

    /* Start CHAT GPT BLOCK */
    // If both buttons are pressed, let the main loop handle auto mode.
    // This prevents the normal button mode toggles from fighting the auto toggle.
    if(!button1 && !button2){
      button1_counts = 0;
      button2_counts = 0;
      return;
    }

    /* End CHAT GPT BLOCK */

    if(!button1){
      if(button1_counts >= 2){
        b1_mode = (b1_mode + 1)%MODES;
        button1_counts = -20;
      } else {
        button1_counts++;
      }
    } else {
      button1_counts = 0;
    }

    if(!button2){
      if(button2_counts >= 2){
        b2_mode = (b2_mode + 1) % (MODES - 1);
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

// Function Prototypes. All of these functions are defined bellow main loop
uint16_t calculate12BitTicks(float inPeriod, PCA9685* pcaCont);
void set_pos(uint16_t axis_value, uint8_t channel);
void find_starting_angle_and_home(void);
void set_mode_leds(void);
float calculated_angle(uint8_t channel);
uint16_t angleToTicks(uint8_t channel, float degrees);
bool moveChannelToward(uint8_t channel, uint16_t targetTicks, uint8_t stepSize);
uint16_t clawPercentToTicks(uint8_t percentOpen);
bool moveArmTo(float baseDeg, float seg1Deg, float seg2Deg, float seg3Deg, float wristDeg, uint16_t clawTicks);
void checkAutoRepeatToggle(void);
void runAutoRepeatMotion(void);

void setup()
{
    Serial.begin(115200);
    Wire.begin(); // Need For I2C
    delay(2000);


    // These variables are for the interupt timer.
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

    // Section for the UNO R4 LED Matrix
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
            Serial.print("Found device at 0x");
            Serial.println(addr, HEX);

        }
    }

    Wire.end(); // closing wire device after verifying address.
    delay(100);


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

    // I decided to do a dynamic memory initialization of the PCA Controller.  No real reason other than C++
    pcaController = new PCA9685(i2cAddress); // set to 25MHz Internal Clock Default
    pcaController->sleepPCA();
    pcaController->setPWMFrequency(PWM_FREQ);  // Frequency Calculated from Prescaler math.
    pcaController->setPWMBias(PWM_BIAS);
    //pcaController->setAllPWM(0, 0);

    delay(100);

    // Verify the chip is actually awake
    uint8_t mode = pcaController->readByte(0x00); // MODE1
    if (mode & 0x10) {
        Serial.println("Warning: PCA9685 is still in SLEEP mode!");
    } else {
        Serial.println("PCA9685 is awake and running.");
    }


    // Set Min Mid and Max values for Servos. All the same by default except the Claw
    for (int i = 0; i < PWM_CHANNELS; ++i){
        pwm_min[i] = calculate12BitTicks(MIN_PERIOD, pcaController);
        pwm_mid[i] = calculate12BitTicks(MID_PERIOD, pcaController);
        pwm_max[i] = calculate12BitTicks(MAX_PERIOD, pcaController);
        pwm_pos[i] = calculate12BitTicks(MID_PERIOD, pcaController);
        pwm_12BitRange[i] = pwm_max[i] - pwm_min[i]; 
    }
    // Go back and set the claw values.
    //
    // This gives the min to max pwm range in amount of ticks per Degree
    float ticks_per_deg = (pwm_max[0] - pwm_min[0])/180.0;

    pwm_min[5] = pwm_min[0] + lround(CLAW_MIN_DEGREES*ticks_per_deg);
    pwm_max[5] = pwm_min[0] + lround(CLAW_MAX_DEGREES*ticks_per_deg);
    pwm_mid[5] = pwm_min[5] + ((pwm_max[5] - pwm_min[5]) / 2);
    pwm_12BitRange[5] = pwm_max[5] - pwm_min[5];
    pwm_pos[5] = pwm_mid[5];

    Serial.print("min: "); Serial.print(pwm_min[0]);
    Serial.print(", mid: "); Serial.print(pwm_mid[0]);
    Serial.print(", max: "); Serial.print(pwm_max[0]);
    Serial.print(", 12BitRange: "); Serial.print(pwm_12BitRange[0]);
    Serial.print(", Degree Step Size: "); Serial.println(ANGLE_RANGE/((float)pwm_12BitRange[0]));


    delay(1000);
    // This function trys to set the main arm to 135degrees currently bassed off starting accelerometer position need to update for full home. 
    find_starting_angle_and_home();
    delay(1000);
  
}

uint16_t prev_pos = 0;


/*THIS IS THE MAIN LOOP WHERE MOST SERIOUS BUSINESS HAPPENS*/
void loop()
{
  // Read Analog Pins
  uint16_t x1 = analogRead(X1_PIN);
  uint16_t y1 = analogRead(Y1_PIN);
  uint16_t x2 = analogRead(X2_PIN);
  uint16_t y2 = analogRead(Y2_PIN);

  // Read Digital Pins
  button1 = digitalRead(SWITCH1_PIN);
  button2 = digitalRead(SWITCH2_PIN);

  /* Start CHAT GPT BLOCK */
  checkAutoRepeatToggle();

  // In dance mode, the arm runs a repeating dance loop.
  // Manual joystick control is paused until both buttons are held again.
  if(autoRepeatMode){
    runAutoRepeatMotion();
    return;
  }
  /* End CHAT GPT BLOCK */

  
  // Check for current mode of opperation for servo access.

  switch(b1_mode){
    case 0:
      set_pos(x1, ROTATE_BASE);
      set_pos(y1, ARM_SEG1);
      break;
    case 1:
      set_pos(x1, ROTATE_BASE);
      set_pos(y1, ARM_SEG3);
      break;
    case 2:
        break;
  }


  switch(b2_mode){
    case 0:
      set_pos(x2, WRIST);
      set_pos(y2, ARM_SEG2);
      break;
    case 1:
      set_pos(x2, WRIST);
      set_pos(y2, CLAW);
      break;
    case 2:
      if(b1_mode == 2){
        while(!moveArmTo(PICKUP_0, PICKUP_1,PICKUP_2,PICKUP_3,PICKUP_4,pwm_min[5])){}
        while(!moveArmTo(PICKUP_0, PICKUP_1,PICKUP_2,PICKUP_3,PICKUP_4,pwm_max[5])){}
        while(!moveArmTo(DROP_0, DROP_1,DROP_2,DROP_3,DROP_4,pwm_max[5])){}
        while(!moveArmTo(DROP_0, DROP_1,DROP_2,DROP_3,DROP_4,pwm_min[5])){}
        b1_mode = 0;
        b2_mode = 0;
      } else if(b1_mode == 3){
        while(!moveArmTo(90.0, 90.0,90.0,90.0,90.0,pwm_min[5])){}
        while(!moveArmTo(PRONE_0, PRONE_1,PRONE_2,PRONE_3,PRONE_4,pwm_min[5])){}
        b1_mode = 0;
        b2_mode = 0;
      }
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
  // sprintf(serialMsg, "ang[0]: %f, ang[1]: %f, ang[2]: %f, ang[3]: %f, ang[4]: %f, ang[5]: %f\n", calculated_angle(0), calculated_angle(1), calculated_angle(2), calculated_angle(3), calculated_angle(4), calculated_angle(5));
  // Serial.print(serialMsg);

  // This code updates  led matrix display to show mode from left and right button press.
  if(b1_prev != b1_mode || b2_prev != b2_mode){
    set_mode_leds();
    b1_prev = b1_mode;
    b2_prev = b2_mode;
  }

}

/* This Function is for calculating the total amount of 12BIT PWM "Ticks" are needed for desired high period based on the PCA Clock and the prescaler
  for the chosen PWM frequency */
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

/* This Code is for changing the arm position using the manual thumb sticks
  Note Step Sizes can be changed at top of code changing PWM to lower frequency
  will make the arm move faster if you don't change these step sizes */
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

/* This Code will try to determine the angle of the lower arm segment and change it's angle to a starting angle
   Relative to starting position It will not start moving until the PWR|B1 message scrolls and you click B1 */
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
    
    // This Do While loop sits and writes "PWR & B1" until the left controll button is pressed.
    // Operator should Connect the power to the servo rail here then press the button.
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

    // gets the calculated angle of arm with accelerometer in degrees for first homing step
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


/* This function tries to calculate and return current active servo angle based on the PWM setting */
float calculated_angle(uint8_t channel){
  return (ANGLE_RANGE/pwm_12BitRange[0])*(pwm_pos[channel] - pwm_min[0]);
}


/* This function changes what the LED matrix displays when the thumbsticks are pressed */
void set_mode_leds(void){
  sprintf(matMsg, "%u|%u", b1_mode, b2_mode);
        matrix.beginText(0, 1, 0xFFFFFF);
        matrix.println(matMsg);
        // matrix.endText(SCROLL_LEFT);
        matrix.endText();
        matrix.endDraw();
}

/* Start CHAT GPT BLOCK */

/* CHAT GPT Wrote this function to convert desired angle to PWM ticks */
uint16_t angleToTicks(uint8_t channel, float degrees){
  if(degrees < 0) degrees = 0;
  if(degrees > 188) degrees = 188;

  uint16_t ticks = pwm_min[channel] + lround(degrees / (ANGLE_RANGE / pwm_12BitRange[channel]));
  if(ticks < pwm_min[channel]) ticks = pwm_min[channel];
  if(ticks > pwm_max[channel]) ticks = pwm_max[channel];
  return ticks;
}


/* CHAT GPT Wrote this function to move servo to desired position based on a step size if it over steps 
   it will set to target position */
bool moveChannelToward(uint8_t channel, uint16_t targetTicks, uint8_t stepSize){
  if(pwm_pos[channel] < targetTicks){
    uint16_t nextPos = pwm_pos[channel] + stepSize;
    pwm_pos[channel] = (nextPos > targetTicks) ? targetTicks : nextPos;
  } else if(pwm_pos[channel] > targetTicks){
    int nextPos = pwm_pos[channel] - stepSize;
    pwm_pos[channel] = (nextPos < targetTicks) ? targetTicks : nextPos;
  }

  pcaController->setPWM(channel, 0, pwm_pos[channel]);
  return pwm_pos[channel] == targetTicks;
}


/* CHAT GPT wrote this funtion to open or close the claw to a percentage of it's useful range. */
uint16_t clawPercentToTicks(uint8_t percentOpen){
  if(percentOpen > 100) percentOpen = 100;
  uint16_t ticks = pwm_min[CLAW] + lround((pwm_max[CLAW] - pwm_min[CLAW]) * (percentOpen / 100.0));
  if(ticks < pwm_min[CLAW]) ticks = pwm_min[CLAW];
  if(ticks > pwm_max[CLAW]) ticks = pwm_max[CLAW];
  return ticks;
}


/* CHAT GPT wrote this clever logical and statement to make sure it only completes a routine if all of the joints have reached
   their desired coordinate */
bool moveArmTo(float baseDeg, float seg1Deg, float seg2Deg, float seg3Deg, float wristDeg, uint16_t clawTicks){
  bool done = true;
  done &= moveChannelToward(ROTATE_BASE, angleToTicks(ROTATE_BASE, baseDeg), AUTO_SPEED);
  done &= moveChannelToward(ARM_SEG1, angleToTicks(ARM_SEG1, seg1Deg), AUTO_SPEED);
  done &= moveChannelToward(ARM_SEG2, angleToTicks(ARM_SEG2, seg2Deg), AUTO_SPEED);
  done &= moveChannelToward(ARM_SEG3, angleToTicks(ARM_SEG3, seg3Deg), AUTO_SPEED);
  done &= moveChannelToward(WRIST, angleToTicks(WRIST, wristDeg), AUTO_SPEED);
  done &= moveChannelToward(CLAW, clawTicks, AUTO_SPEED);
  return done;
}


/* CHAT GPT wrote this function to check and see if user is simultaneously holding both thumb stick buttons to enter auto mode */
void checkAutoRepeatToggle(void){
  bool bothPressed = (!button1 && !button2);

  if(bothPressed && !bothButtonsWereHeld){
    bothButtonsStartMs = millis();
    bothButtonsWereHeld = true;
  }

  if(bothPressed && bothButtonsWereHeld && (millis() - bothButtonsStartMs > BOTH_BUTTON_HOLD_MS)){
    autoRepeatMode = !autoRepeatMode;
    autoStep = 0;
    autoStepStarted = false;
    bothButtonsWereHeld = false;   // prevents rapid re-toggle while held

    matrix.beginText(0, 1, 0xFFFFFF);
    matrix.println(autoRepeatMode ? "DANCE" : "MAN");
    matrix.endText();
    matrix.endDraw();
  }

  if(!bothPressed){
    bothButtonsWereHeld = false;
  }
}


/* CHAT GPT created this function to cary out a "Dance" routine if both thumb stick buttons are held down. */
void runAutoRepeatMotion(void){
  bool stepDone = false;

  // Dance routine target poses: base, shoulder, elbow, upper arm, wrist, claw.
  // Hold BOTH joystick buttons again to stop and return to manual mode.
  // If any joint moves too far for your exact arm, adjust the angle numbers below.
  switch(autoStep){
    case 0: // Center stance, claw open
      stepDone = moveArmTo(90, 135, 90, 90, 90, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 1: // Lean left
      stepDone = moveArmTo(55, 128, 82, 102, 65, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 2: // Claw clap closed
      stepDone = moveArmTo(55, 128, 82, 102, 65, clawPercentToTicks(CLAW_AUTO_CLOSED_PERCENT));
      break;
    case 3: // Lean right, claw closed
      stepDone = moveArmTo(125, 128, 82, 102, 115, clawPercentToTicks(CLAW_AUTO_CLOSED_PERCENT));
      break;
    case 4: // Claw clap open
      stepDone = moveArmTo(125, 128, 82, 102, 115, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 5: // Raise up like a wave
      stepDone = moveArmTo(90, 155, 112, 74, 135, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 6: // Wrist flick left
      stepDone = moveArmTo(80, 148, 105, 82, 50, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 7: // Wrist flick right
      stepDone = moveArmTo(100, 148, 105, 82, 140, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 8: // Little dip
      stepDone = moveArmTo(90, 112, 70, 120, 90, clawPercentToTicks(CLAW_AUTO_CLOSED_PERCENT));
      break;
    case 9: // Pop back up and open claw
      stepDone = moveArmTo(90, 150, 105, 80, 90, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 10: // Spin/sway left
      stepDone = moveArmTo(40, 138, 92, 90, 75, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 11: // Spin/sway right
      stepDone = moveArmTo(140, 138, 92, 90, 105, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
    case 12: // Final clap closed in center
      stepDone = moveArmTo(90, 135, 90, 90, 90, clawPercentToTicks(CLAW_AUTO_CLOSED_PERCENT));
      break;
    case 13: // Open and repeat
      stepDone = moveArmTo(90, 135, 90, 90, 90, clawPercentToTicks(CLAW_AUTO_OPEN_PERCENT));
      break;
  }

  if(stepDone){
    if(!autoStepStarted){
      autoStepStarted = true;
      autoStepStartMs = millis();
    }

    if(millis() - autoStepStartMs > AUTO_PAUSE_MS){
      autoStep = (autoStep + 1) % 14;
      autoStepStarted = false;
    }
  } else {
    autoStepStarted = false;
  }
}
/* End CHAT GPT BLOCK */