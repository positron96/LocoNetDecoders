#pragma once

#include <Arduino.h>

/**
 * Class to indicates program state with a LED.
 * Normally the LED blinks with slowly.
 * If `fire` call is made, the LED lights up for specified time, then continues to blink.
 */
template<int PIN>
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
        ledVal = val;
        digitalWrite(PIN, ledVal);  
        ledNextUpdate = millis()+ms;
    }

    /**
     * The LED continues to blink with previously set interval
     */ 
    static void resume() {
        if(ledNextUpdate!=0) return;
        fire(0,0); // LDE will be turned on the next time `loop` is called
    }

    /**
     * The LED turns off and stops blinking
     */ 
    static void pause() {
        digitalWrite(PIN, LOW); 
        ledNextUpdate = 0; // turn off blink
    }

    /**
     * Updates LED state
     */ 
    static void loop() {
        if(ledNextUpdate!=0 && millis()>ledNextUpdate) {
            ledVal = 1-ledVal;
            digitalWrite(PIN, ledVal);
            ledNextUpdate = ledInterval + millis();
        }
    }

    /**
     * Sets blink half-period in ms.
     */ 
    static void setIntl(uint32_t intl) { ledInterval = intl; }

private:
    static uint32_t ledNextUpdate;
    static uint8_t ledVal;
    static uint32_t ledInterval;
};

template<int PIN>
uint32_t LedBlinker<PIN>::ledNextUpdate;
template<int PIN>
uint8_t LedBlinker<PIN>::ledVal;
template<int PIN>
uint32_t LedBlinker<PIN>::ledInterval;