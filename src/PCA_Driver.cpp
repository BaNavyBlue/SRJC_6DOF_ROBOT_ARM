#include "PCA_Driver.h"
#include <math.h>

#include "PCA_Driver.h"
#include <math.h>

#ifdef PCA9685_LINUX

PCA9685::PCA9685(uint8_t bus, uint8_t address) {
  kI2CBus = bus;
  kI2CAddress = address;
  useExternClk_ = false;
  error = openPCA9685();
  if (error == false) {
    printf("Houston We Have a problem: %d\n", errno);
  }
  setPeriodSlice();
}

PCA9685::PCA9685(uint8_t bus, uint8_t address, double externalClock) {
  kI2CBus = bus;
  kI2CAddress = address;
  cpuClock_ = externalClock;
  useExternClk_ = true;
  error = openPCA9685();
  if (error == false) {
    printf("Houston We Have a problem: %d\n", errno);
  }
  setExtClk();
  setPeriodSlice();
}

#endif

#ifdef PCA9685_ARDUINO

PCA9685::PCA9685(uint8_t address) {
  kI2CAddress = address;
  Wire.begin();
  setPeriodSlice();
}

PCA9685::PCA9685(uint8_t address, double externalClock) {
  kI2CAddress = address;
  cpuClock_ = externalClock;
  useExternClk_ = true;
  Wire.begin();
  setExtClk();
  setPeriodSlice();
}

#endif

PCA9685::~PCA9685() {
#ifdef PCA9685_LINUX
  closePCA9685();
#endif
}

#ifdef PCA9685_LINUX
int PCA9685::get_kI2Address() { return kI2CAddress; }

uint8_t PCA9685::get_kI2CBus() { return kI2CBus; }

void PCA9685::set_kI2CBus(uint8_t bus) { kI2CBus = bus; }
#endif

float PCA9685::getFrequency() { return frequency_; }

int PCA9685::getError() { return error; }

uint8_t PCA9685::getPrescale() { return prescale_; }

void PCA9685::setPeriodSlice() {
  period_slice_ = 1 / (frequency_ * TWO_POW_TWELVE);
}

double PCA9685::getPeriodSlice() { return period_slice_; }

double PCA9685::getClock() { return cpuClock_; }

#ifdef PCA9685_LINUX

bool PCA9685::openPCA9685() {
  char fileNameBuffer[32];
  sprintf(fileNameBuffer, "/dev/i2c-%d", kI2CBus);
  kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
  if (kI2CFileDescriptor < 0) {
    error = errno;
    return false;
  }
  if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {
    error = errno;
    return false;
  }
  return true;
}

void PCA9685::closePCA9685() {
  if (kI2CFileDescriptor > 0) {
    close(kI2CFileDescriptor);
    kI2CFileDescriptor = -1;
  }
}

#endif

void PCA9685::reset() {
  writeByte(PCA9685_MODE1, PCA9685_ALLCALL);
  writeByte(PCA9685_MODE2, PCA9685_OUTDRV);
  // Wait for oscillator to stabilize
#ifdef PCA9685_LINUX
  usleep(5000);
#else
  delay(5);
#endif
}

// Sets the frequency of the PWM signal
// Frequency is ranged between 40 and 1000 Hertz
void PCA9685::setPWMFrequency(float frequency) {
  float rangedFrequency = fmin(fmax(frequency, 23), 1018);
  prescale_ = lround(cpuClock_ / (4096.0 * rangedFrequency) - 1);
  // int prescale = (int)(27500000.0f / (4096 * rangedFrequency) - 0.5f) ; //
  // 10% higher clock
  //  For debugging
  //  printf("PCA9685 Prescale: 0x%02X\n",prescale) ;
  int oldMode = readByte(PCA9685_MODE1);
  int sleepMode = (oldMode & ~PCA9685_RESTART) | PCA9685_SLEEP;
  writeByte(PCA9685_MODE1, sleepMode);
  writeByte(PCA9685_PRE_SCALE, prescale_);
  writeByte(PCA9685_MODE1, oldMode & ~PCA9685_SLEEP);
  // Wait for oscillator to stabilize
#ifdef PCA9685_LINUX
  usleep(5000);
#else
  delay(5);
#endif
  writeByte(PCA9685_MODE1,
            (oldMode & ~PCA9685_SLEEP) | PCA9685_RESTART | MODE1_AI);
  frequency_ = rangedFrequency;
  setPeriodSlice();
  // printf("Setting PCA9685 PWM frequency to %f Hz\n",frequency_) ;
}

// Channels 0-15
// Channels are in sets of 4 bytes
void PCA9685::setPWM(uint8_t channel, uint16_t onValue, uint16_t offValue) {
  writeByte(PCA9685_LED0_ON_L + 4 * channel, onValue & 0xFF);
  writeByte(PCA9685_LED0_ON_H + 4 * channel, onValue >> 8);
  writeByte(PCA9685_LED0_OFF_L + 4 * channel, offValue & 0xFF);
  writeByte(PCA9685_LED0_OFF_H + 4 * channel, offValue >> 8);
}

void PCA9685::setAllPWM(uint16_t onValue, uint16_t offValue) {
  writeByte(PCA9685_ALL_LED_ON_L, onValue & 0xFF);
  writeByte(PCA9685_ALL_LED_ON_H, onValue >> 8);
  writeByte(PCA9685_ALL_LED_OFF_L, offValue & 0xFF);
  writeByte(PCA9685_ALL_LED_OFF_H, offValue >> 8);
}

// Read the given register
uint8_t PCA9685::readByte(uint8_t readRegister) {
#ifdef PCA9685_LINUX

  int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
  if (toReturn < 0) {
    printf("PCA9685 Read Byte error: %d\n", errno);
    error = errno;
    toReturn = -1;
  }
  return toReturn;

#else // ARDUINO

  Wire.beginTransmission(kI2CAddress);
  Wire.write(readRegister);
  Wire.endTransmission(false); // repeated start
  Wire.requestFrom(kI2CAddress, (uint8_t)1);
  return Wire.read();

#endif
}

// Write the the given value to the given register
void PCA9685::writeByte(uint8_t writeRegister, uint8_t writeValue) {
#ifdef PCA9685_LINUX

  int toReturn =
      i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
  if (toReturn < 0) {
    printf("PCA9685 Write Byte error: %d\n", errno);
    error = errno;
  }

#else // ARDUINO

  Wire.beginTransmission(kI2CAddress);
  Wire.write(writeRegister);
  Wire.write(writeValue);
  Wire.endTransmission();

#endif
}

void PCA9685::setExtClk() {
  uint8_t oldmode = readByte(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  writeByte(PCA9685_MODE1,
            newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  writeByte(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

  writeByte(PCA9685_PRE_SCALE, prescale_); // set the prescaler

#ifdef PCA9685_LINUX
  sleep(5);
#else
  delay(5000);
#endif
  // clear the SLEEP bit to start
  writeByte(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
  // Some Arduino nonsense
  // #ifdef ENABLE_DEBUG_OUTPUT
  //   Serial.print("Mode now 0x");
  //   Serial.println(read8(PCA9685_MODE1), HEX);
  // #endif
}

void PCA9685::setPWMBias(int bias) { pwmOnBias_ = bias; }

int PCA9685::getPWMBias() { return pwmOnBias_; }

int PCA9685::stopPCA() {
  printf("Setting all to Zero and Reset\n");
  setAllPWM(0, 0);
  reset();
}

int PCA9685::sleepPCA() {
  uint8_t awake = readByte(PCA9685_MODE1);
  uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
  writeByte(PCA9685_MODE1, sleep);
#ifdef PCA9685_LINUX
  usleep(5000);
#else
  delay(5);
#endif
  return 0;
}

int PCA9685::wakePCA() {
  uint8_t sleep = readByte(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
  writeByte(PCA9685_MODE1, wakeup);
  return 0;
}
