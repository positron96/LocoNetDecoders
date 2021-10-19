#pragma once

#include <Arduino.h>
#include "SerialUtils.h"
#include <EEPROM.h>
#include <etl/type_traits.h>

template<uint8_t PIN_CLK, uint8_t PIN_DAT, uint8_t PIN_LATCH>
class TLC5947GPIO {
public:
    using channel_t = uint8_t;
    constexpr static channel_t CH_OUT_COUNT = 24;

    static void initHw() {
        
        //pinMode(PIN_OEn, OUTPUT);
        //digitalWrite(PIN_OEn, HIGH); 


        pinMode(PIN_CLK, OUTPUT);
        pinMode(PIN_DAT, OUTPUT);
        pinMode(PIN_LATCH, OUTPUT);
        digitalWrite(PIN_LATCH, LOW);

        write();

        //digitalWrite(PIN_OEn, LOW); 
    
    }

    static constexpr uint8_t EEPROM_VER = 1^CH_OUT_COUNT;
    static constexpr int EEPROM_REQUIRED = CH_OUT_COUNT*2;
    static bool load(int eepromAddr) {
        for(channel_t i=0; i<CH_OUT_COUNT; i++) {
            EEPROM.get<uint8_t>(eepromAddr, states[i].maxPWM);
            uint8_t f;
            EEPROM.get<uint8_t>(eepromAddr+1, f);
            states[i].fade = f!=0;
            eepromAddr+=2;
        }
        return true;
    }
    static void reset() {
        for(channel_t ch=0; ch<CH_OUT_COUNT; ch++) {
            states[ch].maxPWM = 32;
            states[ch].fade = true;
            states[ch].curVal = false;
        }
    }
    static bool save(int eepromAddr) {
        for(channel_t i=0; i<CH_OUT_COUNT; i++) {
            EEPROM.put<uint8_t>(eepromAddr, states[i].maxPWM);
            EEPROM.put<uint8_t>(eepromAddr+1, states[i].fade ? 255 : 0);
            eepromAddr+=2;
        }
        return true;
    }

    static void set(channel_t ch, bool val) {
        setn<1>(ch, val, 0);
    }


    static constexpr uint8_t MAX_CH = 3;

    template<uint8_t N, typename... Offsets, 
        etl::enable_if_t<sizeof...(Offsets) == N, bool> = true,
        etl::enable_if_t<etl::are_all_same<int, Offsets...>::value, bool> = true,
        etl::enable_if_t<N<=MAX_CH, bool> = true >
    static void setn(const channel_t ch0, const bool val, const Offsets... ofs_) {
        const channel_t ch[] = { channel_t(ch0+ofs_)... };
        set_arr(ch, N, val);
    }

    static void set_arr(const channel_t* ch, const uint8_t N, const bool val) {
        bool allgood = true;
        for(uint8_t i=0; i<N; i++) if(get(ch[i])!=val) allgood = false;
        if(allgood) return;

        int16_t dst[MAX_CH];
        for(uint8_t i=0; i<N; i++) dst[i] = val ? maxPWM12(ch[i]) : 0;

        if(states[ch[0]].fade) {
            int16_t src[MAX_CH];
            int16_t t[MAX_CH];

            for(uint8_t i=0; i<N; i++) src[i] = get(ch[i]) ? maxPWM12(ch[i]) : 0;
            for(uint8_t step=0; step<RES; step++) {
                for(uint8_t i=0; i<N; i++) {
                    t[i] = src[i] + (dst[i]-src[i])*step/RES;
                }
                write(ch, t, N);
                delay(TRANS_TIME/RES);
            }
        }
        for(uint8_t i=0; i<N; i++) {
            states[ch[i]].curVal = val;
        }
        write();
    }

    static bool get(channel_t ch) {
        return states[ch].curVal;
    }

    static void toggle(channel_t pin) {
        set(pin, !get(pin)); 
    }

    static void setMaxPWM(channel_t ch, uint8_t mx) {
        states[ch].maxPWM = mx >> (8-PWM_BITS);
    }

private:

    struct state_t {
        uint8_t maxPWM;///< divided by 16 (actual max is 12bits) 
        bool curVal:1;
        bool fade:1;
    } __attribute__((packed));

    static state_t states[CH_OUT_COUNT];
    
    static constexpr uint32_t TRANS_TIME = 100; // ms
    static constexpr uint8_t RES = 8;

    static constexpr uint8_t PWM_BITS = 8;
    static constexpr uint8_t PWM_BITSHIFT = 12-PWM_BITS;

    static inline uint16_t maxPWM12(channel_t ch) {
        return states[ch].maxPWM << PWM_BITSHIFT;
    }

    //static constexpr uint8_t NUM_BYTES = CH_OUT_COUNT/8;

    /**
     * Sends current values to TLC and overrides specific few channels by values provided.
     * @param ch - channels to override. Can be null
     * @param vals - values (12 bit) to override. Must be same elements as ch. Can be null
     * @param N - number of elements in ch and vals.
     * @note to speed things up, ch must have channels in ascending order
     *       and max(ch)-min(ch) must be <=MAX_CH
     */
    static void write(const channel_t *ch=nullptr, const int16_t *vals=nullptr, const uint8_t N=0) {
        digitalWrite(PIN_LATCH, LOW);
        // 24 channels per TLC5974
        channel_t s=CH_OUT_COUNT,e=0;
        if (ch!=nullptr) {
            for(int i=0; i<N; i++) { if(ch[i]<s)s=ch[i];  if(ch[i]>e)e=ch[i]; }
        }
        for (int16_t c = CH_OUT_COUNT-1; c >= 0; c--) {
            int16_t v=-1;
            if(s!=CH_OUT_COUNT && c>=s && c<=e) {
                for(int i=0; i<N; i++) if(c==ch[i]) {v=vals[i]; break;}
            }
            if(v<0) v = get(c) ? maxPWM12(c) : 0;

            // 12 bits per channel, send MSB first
            for (int8_t b = 11; b >= 0; b--) {
                digitalWrite(PIN_CLK, LOW);

                if (v & (1 << b))
                    digitalWrite(PIN_DAT, HIGH);
                else
                    digitalWrite(PIN_DAT, LOW);

                digitalWrite(PIN_CLK, HIGH);
            }
        }
        digitalWrite(PIN_CLK, LOW);

        digitalWrite(PIN_LATCH, HIGH);
        digitalWrite(PIN_LATCH, LOW);
    }

};

template<uint8_t P1, uint8_t P2, uint8_t P3>
typename TLC5947GPIO<P1,P2,P3>::state_t TLC5947GPIO<P1,P2,P3>::states[TLC5947GPIO<P1,P2,P3>::CH_OUT_COUNT];


