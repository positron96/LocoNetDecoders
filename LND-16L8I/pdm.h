/**
 * Pulse density modulation,
 * also known as Bresenham algorithm for line rasterization.
 * @see https://matthewearl.github.io/2015/03/05/efficient-pwm/
 * @see https://en.wikipedia.org/wiki/Pulse-density_modulation
 */

#pragma once

#include <Arduino.h>

template<uint8_t N_CH, uint8_t P=255>
class PDM {
public:
    static void init() {
        static_assert( N_CH<=32, "Channel count must be less than 32");
        out = 0;
        for(int i=0; i<N_CH; i++) target[i] = 0;
    }

    static void tick() {
        for(int ch=0; ch<N_CH; ch++) {
            if(err[ch] < target[ch]) {
                bitSet(out, ch);
                err[ch] += P-target[ch];
            } else {
                bitClear(out, ch);
                err[ch] -= target[ch];
            }
        }
    }

    static uint32_t values() {
        return out;
    }

    static uint8_t get(uint8_t ch) { return target[ch]; }

    static void set(uint8_t ch, uint8_t val) {
        target[ch] = val;
    }

private:
    static uint32_t out;
    static uint8_t target[N_CH];
    static uint8_t err[N_CH];
};


template<uint8_t N_CH, uint8_t P>
uint8_t PDM<N_CH,P>::target[N_CH];

template<uint8_t N_CH, uint8_t P>
uint8_t PDM<N_CH,P>::err[N_CH];

template<uint8_t N_CH, uint8_t P>
uint32_t PDM<N_CH,P>::out;