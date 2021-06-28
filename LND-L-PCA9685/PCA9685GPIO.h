#pragma once

#include <Arduino.h>
#include "PCA9685Driver.h"
#undef min
#undef max
#include <etl/bitset.h>
#include "SerialUtils.h"
#include <EEPROM.h>

template<uint8_t PIN_OEn>
class PCA9685GPIO {
public:
    using channel_t = uint8_t;
    constexpr static channel_t CH_OUT_COUNT = 16;

    static void initHw() {
        pwm = PCA9685Driver(0x40);
        
        pinMode(PIN_OEn, OUTPUT);
        digitalWrite(PIN_OEn, HIGH); // disable pca
        
        Wire.begin();
        pwm.softResetAll();
        pwm.begin();

        pwm.setPWM(PCA9685_ALL_LEDS, 0);
        pwm.setOpenDrainOutput();
        pwm.setInvertMode(true);
        pwm.setOeMode(PCA9685Driver::OeMode::HIGHZ);
        pwm.setPWMFreq(1600);

        pwm.wakeup();    
        
        digitalWrite(PIN_OEn, LOW); // enable pca
    
    }

    static constexpr uint8_t EEPROM_VER = 1+CH_OUT_COUNT;
    static constexpr int EEPROM_REQUIRED = CH_OUT_COUNT*2;
    static bool load(int eepromAddr) {
        for(channel_t i=0; i<CH_OUT_COUNT; i++) {
            EEPROM.get<uint8_t>(eepromAddr, maxPWM[i]);
            uint8_t f;
            EEPROM.get<uint8_t>(eepromAddr+1, f);
            fade[i] = f!=0;
            eepromAddr+=2;
        }
        return true;
    }
    static void reset() {
        for(channel_t ch=0; ch<CH_OUT_COUNT; ch++) {
            maxPWM[ch] = 32;
            fade[ch] = true;
        }
    }
    static bool save(int eepromAddr) {
        for(channel_t i=0; i<CH_OUT_COUNT; i++) {
            EEPROM.put<uint8_t>(eepromAddr, maxPWM[i]);
            EEPROM.put<uint8_t>(eepromAddr+1, fade[i] ? 255 : 0);
            eepromAddr+=2;
        }
        return true;
    }

    static void set(channel_t ch, bool val) {
        if(get(ch)==val) return;

        //Serial<<F("Setting channel ")<<ch<<F(" to ")<<=val;

        int16_t dst = val ? maxPWM12(ch) : 0;
        if(fade[ch]) {
            int16_t src = (!val)?maxPWM12(ch) : 0;
            for(uint8_t i=0; i<RES; i++) {
                uint16_t t = src + (dst-src)*i/RES;
                pwm.setPWM(ch, t);
                delay(TRANS_TIME/RES);
            }
        }
        pwm.setPWM(ch, dst);
        curVal[ch] = val;
    }

    static void set2(channel_t ch0, bool val, uint8_t ofs1, uint8_t ofs2) {
        channel_t ch1=ch0+ofs1, ch2=ch0+ofs2;
        if(get(ch1)==val && get(ch2)==val) return;

        int16_t dst1 = val ? maxPWM12(ch1) : 0;
        int16_t dst2 = val ? maxPWM12(ch2) : 0;
        if(fade[ch0+ofs1]) {
            int16_t src1 = get(ch1) ? maxPWM12(ch1) : 0;
            int16_t src2 = get(ch2) ? maxPWM12(ch2) : 0;
            for(uint8_t i=0; i<RES; i++) {
                uint16_t t1 = src1 + (dst1-src1)*i/RES;
                uint16_t t2 = src2 + (dst2-src2)*i/RES;
                pwm.setPWM(ch1, t1);
                pwm.setPWM(ch2, t2);
                delay(TRANS_TIME/RES);
            }
        }
        pwm.setPWM(ch1, dst1);
        pwm.setPWM(ch2, dst2);
        curVal[ch1] = val;
        curVal[ch2] = val;
    }

    static bool get(channel_t pin) {
        return curVal[pin];
    }

    static void toggle(channel_t pin) {
        set(pin, !get(pin)); 
    }

    static void setMaxPWM(channel_t ch, uint8_t max) {
        maxPWM[ch] = max >> (8-PWM_BITS);
    }

private:

    static PCA9685Driver pwm;
    static uint8_t maxPWM[CH_OUT_COUNT]; ///< divided by 16 (actual max is 12bits) 

    static etl::bitset<CH_OUT_COUNT> fade;
    static etl::bitset<CH_OUT_COUNT> curVal;
    
    static constexpr uint32_t TRANS_TIME = 100; // ms
    static constexpr uint8_t RES = 8;

    static constexpr uint8_t PWM_BITS = 8;
    static constexpr uint8_t PWM_BITSHIFT = 12-PWM_BITS;

    static uint16_t maxPWM12(channel_t ch) {
        return maxPWM[ch] << PWM_BITSHIFT;
    }

};

template<uint8_t OE>
PCA9685Driver PCA9685GPIO<OE>::pwm;

template<uint8_t OE>
uint8_t PCA9685GPIO<OE>::maxPWM[PCA9685GPIO<OE>::CH_OUT_COUNT];

template<uint8_t OE>
etl::bitset<PCA9685GPIO<OE>::CH_OUT_COUNT> PCA9685GPIO<OE>::fade;

template<uint8_t OE>
etl::bitset<PCA9685GPIO<OE>::CH_OUT_COUNT> PCA9685GPIO<OE>::curVal;
