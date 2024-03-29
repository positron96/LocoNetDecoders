#include "PCA9685Driver.h"
#include <Wire.h>

//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
PCA9685Driver::PCA9685Driver(const uint8_t addr)
        : PCA9685Driver(addr, Wire) {}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 *  @param  i2c  A reference to a 'TwoWire' object that we'll use to communicate
 *  with
 */
PCA9685Driver::PCA9685Driver(const uint8_t addr, TwoWire &i2c)
        : _i2caddr(addr), _i2c(&i2c) 
{
    
}

void PCA9685Driver::setAddr(const uint8_t addr) {
    _i2caddr = addr;
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 */
void PCA9685Driver::begin() {
    _i2c->begin();
    
    // This sets the MODE1 register to turn on auto increment.
    write8(PCA9685_MODE1, read8(PCA9685_MODE1) | MODE1_AI);
}

void PCA9685Driver::softResetAll() {
    _i2c->beginTransmission(0);
    _i2c->write(0x06);
    _i2c->endTransmission();
}

/*!
 *  @brief  Sets a restart flag in the PCA9685 chip
 */
void PCA9685Driver::restart() {
    write8(PCA9685_MODE1, read8(PCA9685_MODE1) | MODE1_RESTART);
    delay(1);
}

/*!
 *  @brief  Puts board into sleep mode
 */
void PCA9685Driver::sleep() {
    // set sleep bit high
    write8(PCA9685_MODE1, read8(PCA9685_MODE1) | MODE1_SLEEP);
    delay(1); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void PCA9685Driver::wakeup() {
    uint8_t sleep = read8(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
    write8(PCA9685_MODE1, wakeup);
    delay(1); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void PCA9685Driver::setExtClk(uint8_t prescale) {
    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

    // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
    // use the external clock.
    write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

    write8(PCA9685_PRESCALE, prescale); // set the prescaler

    delay(5);
    // clear the SLEEP bit to start
    write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Mode now 0x");
    Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1600 Hz (on internal 25 MHz crystal)
 *  @param  freq  point frequency that we will attempt to match, in Hz
 */
void PCA9685Driver::setPWMFreq(uint16_t freq) {
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Attempting to set freq ");
    Serial.println(freq);
#endif
    // Range output modulation frequency is dependant on oscillator
    if (freq < 24)
        freq = 24; // 24 Hz if lowest for maximum 0xFF prescaler and 25 MHz
    if (freq > 1526)
        freq = 1526; // With minimal prescaler 0x3 and 25 MHz

    uint32_t prescale = (FREQUENCY_OSCILLATOR_FREQ / freq / 4096) - 1;
    if (prescale < PCA9685_PRESCALE_MIN)
        prescale = PCA9685_PRESCALE_MIN;
    if (prescale > PCA9685_PRESCALE_MAX)
        prescale = PCA9685_PRESCALE_MAX;
    

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Final pre-scale: ");
    Serial.println(prescale);
#endif

    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    write8(PCA9685_MODE1, newmode);                             // go to sleep
    write8(PCA9685_PRESCALE, (uint8_t)prescale); // set the prescaler
    write8(PCA9685_MODE1, oldmode);
    delay(5);

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Mode now 0x");
    Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void PCA9685Driver::setOutputMode(bool totempole) {
    uint8_t oldmode = read8(PCA9685_MODE2);
    uint8_t newmode;
    if (totempole) {
        newmode = oldmode | MODE2_OUTDRV;
    } else {
        newmode = oldmode & ~MODE2_OUTDRV;
    }
    write8(PCA9685_MODE2, newmode);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Setting output mode: ");
    Serial.print(totempole ? "totempole" : "open drain");
    Serial.print(" by setting MODE2 to ");
    Serial.println(newmode);
#endif
}

void PCA9685Driver::setOeMode(const OeMode m) {
    uint8_t oldMode = read8(PCA9685_MODE2);
    uint8_t newMode;
    if(m==OeMode::LOWVAL) newMode = 0;
    if(m==OeMode::HIGHZ)  newMode = MODE2_OUTNE_1;
    if(m==OeMode::OUTDRV) newMode = MODE2_OUTNE_0;
    newMode = (oldMode & ~MODE2_OUTNE_MASK) | newMode;
    write8(PCA9685_MODE2, newMode);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Setting OE mode: ");
    Serial.print(m==OeMode::LOWVAL ? "0" : m==OeMode::HIGHZ? "HighZ" : "OUTDRV-specified");
    Serial.print(" by setting MODE2 to ");
    Serial.println(newmode);
#endif
}

void PCA9685Driver::setInvertMode(const bool invert) {
    uint8_t mode = read8(PCA9685_MODE2);
    if (invert) {
        mode |= MODE2_INVRT;
    } else {
        mode &= ~MODE2_INVRT;
    }
    write8(PCA9685_MODE2, mode);
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t PCA9685Driver::readPrescale(void) {
    return read8(PCA9685_PRESCALE);
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15. 255 for ALL LEDs
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
void PCA9685Driver::setRaw(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Setting PWM ");
    Serial.print(num);
    Serial.print(": ");
    Serial.print(on);
    Serial.print("->");
    Serial.println(off);
#endif

    _i2c->beginTransmission(_i2caddr);
    _i2c->write(num!=255 ? PCA9685_LED0_ON_L + 4*num : PCA9685_ALLLED_ON_L );
    _i2c->write(on);
    _i2c->write(on >> 8);
    _i2c->write(off);
    _i2c->write(off >> 8);
    _i2c->endTransmission();
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void PCA9685Driver::setPWM(uint8_t num, uint16_t val) {
    // Clamp value between 0 and 4095 inclusive.
    val = min(val, (uint16_t)PCA9685_MAX_PWM);
    
    if (val == PCA9685_MAX_PWM) {
        // Special value for signal fully on.
        setRaw(num, PCA9685_FULL_PWM, 0);
    } else if (val == 0) {
        // Special value for signal fully off.
        setRaw(num, 0, PCA9685_FULL_PWM);
    } else {
        setRaw(num, 0, val);
    }
    
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
// void PCA9685Driver::writeMicroseconds(uint8_t num,
//                                                                                                 uint16_t Microseconds) {
// #ifdef ENABLE_DEBUG_OUTPUT
//     Serial.print("Setting PWM Via Microseconds on output");
//     Serial.print(num);
//     Serial.print(": ");
//     Serial.print(Microseconds);
//     Serial.println("->");
// #endif

//     double pulse = Microseconds;
//     double pulselength;
//     pulselength = 1000000; // 1,000,000 us per second

//     // Read prescale
//     uint16_t prescale = readPrescale();

// #ifdef ENABLE_DEBUG_OUTPUT
//     Serial.print(prescale);
//     Serial.println(" PCA9685 chip prescale");
// #endif

//     // Calculate the pulse for PWM based on Equation 1 from the datasheet section
//     // 7.3.5
//     prescale += 1;
//     pulselength *= prescale;
//     pulselength /= _oscillator_freq;

// #ifdef ENABLE_DEBUG_OUTPUT
//     Serial.print(pulselength);
//     Serial.println(" us per bit");
// #endif

//     pulse /= pulselength;

// #ifdef ENABLE_DEBUG_OUTPUT
//     Serial.print(pulse);
//     Serial.println(" pulse for PWM");
// #endif

//     setRaw(num, 0, pulse);
// }


/******************* Low level I2C interface */
uint8_t PCA9685Driver::read8(uint8_t addr) {
    _i2c->beginTransmission(_i2caddr);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
    return _i2c->read();
}

void PCA9685Driver::write8(uint8_t addr, uint8_t d) {
    _i2c->beginTransmission(_i2caddr);
    _i2c->write(addr);
    _i2c->write(d);
    _i2c->endTransmission();
}
