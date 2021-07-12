#pragma once

#include <Arduino.h>

/**
 * Class to indicates program state with a LED.
 * Normally the LED blinks with slowly.
 * If `fire` call is made, the LED lights up for specified time, then continues to blink.
 */
template<int PIN, uint32_t UPDATE_INTL=1>
class LedBlinker {
public:
    static void begin() {
        pinMode(PIN, OUTPUT);
        setIntl(1000);
        fire(50,0);        
    }

    /**
     * Turns on (or off) the LED for specified time. 
     * After that, the LED continues to blink (even if it was stopped)
     */ 
    static void fire(uint32_t ms, uint8_t val=1) {
        val = val;
        digitalWrite(PIN, val);  
        curInterval = ms/UPDATE_INTL;
        ticks = 0; // reset cycle
    }

    /**
     * The LED continues to blink with previously set interval
     */ 
    static void resume() {
        if(curInterval!=0) return;
        fire(0,0); // LDE will be turned on the next time `loop` is called
    }

    /**
     * The LED turns off and stops blinking
     */ 
    static void pause() {
        digitalWrite(PIN, LOW); 
        curInterval = 0; // turn off blink
    }

    /**
     * Updates LED state
     */ 
    static void loop() {
        ticks++;
        if(curInterval!=0 && ticks>=curInterval) {
            val = 1-val;
            digitalWrite(PIN, val);
            curInterval = interval;
            ticks = 0;
        }
        
    }

    /**
     * Sets blink half-period in ms.
     */ 
    static void setIntl(uint32_t intl) { interval = intl/UPDATE_INTL; }

private:
    static uint16_t curInterval;
    static uint16_t ticks;
    static uint8_t val;
    static uint16_t interval;
};

template<int PIN, uint32_t UPDATE_INTL>
uint16_t LedBlinker<PIN, UPDATE_INTL>::curInterval;
template<int PIN, uint32_t UPDATE_INTL>
uint16_t LedBlinker<PIN, UPDATE_INTL>::ticks;
template<int PIN, uint32_t UPDATE_INTL>
uint8_t LedBlinker<PIN, UPDATE_INTL>::val;
template<int PIN, uint32_t UPDATE_INTL>
uint16_t LedBlinker<PIN, UPDATE_INTL>::interval;