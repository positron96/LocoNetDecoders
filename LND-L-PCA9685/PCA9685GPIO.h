#pragma once

#include <Arduino.h>
#include "PCA9685Driver.h"
#include "SerialUtils.h"
#include <EEPROM.h>
#include <etl/type_traits.h>

template<uint8_t PIN_OEn>
class PCA9685GPIO {
public:
    using channel_t = uint8_t;
    constexpr static channel_t CH_OUT_COUNT = 16;

    using effect_t = uint8_t;
    static constexpr effect_t EFFECT_FADE = 1;
    static constexpr effect_t EFFECT_FLUORESCENT_LAMP = 2; 

    static void initHw() {
        //pwm = PCA9685Driver(0x40);
        
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

    static constexpr uint8_t EEPROM_VER = 1^CH_OUT_COUNT;
    static constexpr int EEPROM_REQUIRED = CH_OUT_COUNT*2;
    static bool load(int eepromAddr) {
        for(channel_t i=0; i<CH_OUT_COUNT; i++) {
            EEPROM.get<uint8_t>(eepromAddr, states[i].maxPWM);
            eepromAddr+=1;
        }
        return true;
    }
    static void reset() {
        for(channel_t ch=0; ch<CH_OUT_COUNT; ch++) {
            states[ch].maxPWM = 32;
            states[ch].curVal = false;
        }
    }
    static bool save(int eepromAddr) {
        for(channel_t i=0; i<CH_OUT_COUNT; i++) {
            EEPROM.put<uint8_t>(eepromAddr, states[i].maxPWM);
            eepromAddr+=1;
        }
        return true;
    }

    static void set(channel_t ch, bool val, effect_t eff=0) {
        setn<1>(ch, val, eff, 0);
    }


    static constexpr uint8_t MAX_CH = 3;

    template<uint8_t N, typename... Offsets, 
        etl::enable_if_t<sizeof...(Offsets) == N, bool> = true,
        etl::enable_if_t<etl::are_all_same<int, Offsets...>::value, bool> = true,
        etl::enable_if_t<N<=MAX_CH, bool> = true >
    static void setn(const channel_t ch0, const bool val, const effect_t eff, const Offsets... ofs_) {
        const channel_t ch[] = { channel_t(ch0+ofs_)... };
        set_arr(ch, N, val);
    }

    static void set_arr(const channel_t* ch, const uint8_t N, const bool val, const effect_t eff=0) {
        bool allgood = true;
        for(uint8_t i=0; i<N; i++) if(get(ch[i])!=val) allgood = false;
        if(allgood) return;

        int16_t dst[MAX_CH];
        for(uint8_t i=0; i<N; i++) dst[i] = val ? maxPWM12(ch[i]) : 0;

        if(eff==EFFECT_FADE) {
            int16_t src[MAX_CH];

            for(uint8_t i=0; i<N; i++) src[i] = get(ch[i]) ? maxPWM12(ch[i]) : 0;
            for(uint8_t step=0; step<RES; step++) {
                for(uint8_t i=0; i<N; i++) {
                    uint16_t t = src[i] + (dst[i]-src[i])*step/RES;
                    pwm.setPWM(ch[i], t);
                }
                delay(TRANS_TIME/RES);
            }
        } else
        if(eff==EFFECT_FLUORESCENT_LAMP) {
            for(uint8_t step=1; step<=4; step++) {
                int t = 5 + (rand() % 10);
                delay(step*t);
                pwm.setPWM(ch[0], dst[0]);
                t = rand() % 100;
                delay(step*t);)
            }
        }
        for(uint8_t i=0; i<N; i++) {
            pwm.setPWM(ch[i], dst[i]);
            states[ch[i]].curVal = val;
        }
    }

    static bool get(channel_t ch) {
        return states[ch].curVal;
    }

    static void toggle(channel_t pin, effect_t eff=0) {
        set(pin, !get(pin), eff); 
    }

    static void setMaxPWM(channel_t ch, uint8_t mx) {
        states[ch].maxPWM = mx >> (8-PWM_BITS);
    }

private:

    struct state_t {
        uint8_t maxPWM;///< divided by 16 (actual max is 12bits) 
        bool curVal:1;
    } __attribute__((packed));

    static PCA9685Driver pwm;

    static state_t states[CH_OUT_COUNT];
    
    static constexpr uint32_t TRANS_TIME = 100; // ms
    static constexpr uint8_t RES = 8;

    static constexpr uint8_t PWM_BITS = 8;
    static constexpr uint8_t PWM_BITSHIFT = 12-PWM_BITS;

    static uint16_t maxPWM12(channel_t ch) {
        return states[ch].maxPWM << PWM_BITSHIFT;
    }

};

template<uint8_t OE>
PCA9685Driver PCA9685GPIO<OE>::pwm;

template<uint8_t OE>
typename PCA9685GPIO<OE>::state_t PCA9685GPIO<OE>::states[PCA9685GPIO<OE>::CH_OUT_COUNT];


