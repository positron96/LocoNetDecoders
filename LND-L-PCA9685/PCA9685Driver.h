#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#undef min
#undef max
#include <etl/bitset.h>

template<uint8_t PIN_OE>
class PCA9685Driver {
public:
    constexpr static int CH_OUT_COUNT = 16;
    static Adafruit_PWMServoDriver pwm;
    static uint8_t maxOutputVals[CH_OUT_COUNT]; ///< divided by 16 (actual max is 12bits)    

    static void init() {
        pwm = Adafruit_PWMServoDriver(0x40);

        pinMode(PIN_OE, OUTPUT);
        digitalWrite(PIN_OE, HIGH); // disable LED driver   
        pwm.begin();
        pwm.setPWMFreq(1600);
        pwm.setOutputMode(false); // open drain
        for(int ch=0; ch<CH_OUT_COUNT; ch++) {
            pwm.setPin(ch, 0, true);
            curVal[ch] = false;
            maxOutputVals[ch] = 32;
            fade[ch] = true;
        }
        maxOutputVals[0] = 8;
        digitalWrite(PIN_OE, LOW); // enable LED driver
    }

    static void set(uint16_t ch, uint8_t val) {
        if(get(ch)==val) return;

        Serial<<F("Setting channel ")<<ch<<F(" to ")<<=val;

        int16_t dst = val!=0 ? (maxOutputVals[ch]<<4) : 0;
        if(fade[ch]) {
            int16_t src = (val==0)?(maxOutputVals[ch]<<4) : 0;
            for(uint8_t i=0; i<RES; i++) {
                uint16_t t = src + (dst-src)*i/RES;
                pwm.setPin(ch, t, true);
                delay(TRANS_TIME/RES);
            }
        }
        pwm.setPin(ch, dst, true);
        curVal[ch] = val!=0;
    }

    static void set2(uint16_t ch0, uint8_t val, uint8_t ofs1, uint8_t ofs2) {
        if(get(ch0+ofs1)==val && get(ch0+ofs2)==val) return;
        pwm.setPin(ch0+ofs1, val!=0 ? maxOutputVals[ch0+ofs1]<<4 : 0, true);
        pwm.setPin(ch0+ofs2, val!=0 ? maxOutputVals[ch0+ofs2]<<4 : 0, true);
        curVal[ch0+ofs1] = val!=0;
        curVal[ch0+ofs2] = val!=0;
    }

    static uint8_t get(uint16_t pin) {
        return curVal[pin] ? 1 : 0;
    }

    static void toggle(uint16_t pin) {
        set(pin, get(pin)==0 ? 1 : 0); 
    }

private:
    static etl::bitset<CH_OUT_COUNT> fade;
    static etl::bitset<CH_OUT_COUNT> curVal;
    
    static constexpr uint32_t TRANS_TIME = 100; // ms
    static constexpr uint8_t RES = 8;

};

template<uint8_t OE>
Adafruit_PWMServoDriver PCA9685Driver<OE>::pwm;

template<uint8_t OE>
uint8_t PCA9685Driver<OE>::maxOutputVals[PCA9685Driver<OE>::CH_OUT_COUNT];

template<uint8_t OE>
etl::bitset<PCA9685Driver<OE>::CH_OUT_COUNT> PCA9685Driver<OE>::fade;

template<uint8_t OE>
etl::bitset<PCA9685Driver<OE>::CH_OUT_COUNT> PCA9685Driver<OE>::curVal;
