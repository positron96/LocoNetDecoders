#pragma once

#include <stdint.h>
#undef min
#undef max
#include <etl/vector.h>
#include <etl/type_traits.h>

/*
class OD {
    static void set(uint16_t pin, uint8_t val);
    static void set(uint16_t pin0, uint8_t val, const etl::array &ofs);
    static uint8_t get(uint16_t pin);
    static void toggle(uint16_t pin);
}*/

constexpr uint16_t BLINK_DUR = 1000;

template<class OutputDriver>
class Mast {
public:
    uint16_t busAddr;
    uint16_t ch;
    uint8_t curAspect;
    uint8_t nheads;
    uint32_t lastChangeTime;
public:

    Mast(uint16_t busAddress, uint16_t outputChannel, uint8_t nheads)
        : busAddr(busAddress)
        , ch(outputChannel)
        , nheads(nheads) 
    {
        
    }

    void setAspect(uint8_t aspect) {
        curAspect = aspect;
        lastChangeTime = 0;
        switch(nheads) {
            case 1: { // 1-head
                set1head();
                break;
            }
            case 2: { // 2-head
                set2head();
                break;
            }
            case 3: { // 3-head
                set3head();
                break;
            }
        }
    }

    uint8_t getAspect() { return curAspect; }

    void tick() {
        if(lastChangeTime==0) return;
        switch(nheads) {
            case 1: { // 1-head
                tick1head();
                break;
            }
            case 2: { // 2-head
                tick2head();
                break;
            }
            case 3: { // 3-head
                tick3head();
                break;
            }
        }
    }

private:

    void set1head() {
        switch(curAspect) {
            case 2:
                lastChangeTime = millis();
            case 1: OutputDriver::set(ch, 1); break;
            case 3: OutputDriver::set(ch, 0); break;
        }
        
    }

    void tick1head() {
        if(millis() - lastChangeTime > BLINK_DUR) {
            if(curAspect==2) {
                OutputDriver::toggle(ch);
                lastChangeTime = millis();
            }
        }
    }


    void set2head() {
        switch(curAspect) {
            case 3:
            case 4: 
                lastChangeTime = millis();
            case 1: 
                OutputDriver::set(ch+1, 0); 
                OutputDriver::set(ch, 1); 
                break;
            case 2:
                OutputDriver::set(ch, 0); 
                OutputDriver::set(ch+1, 1); 
                break;
            case 5: 
                OutputDriver::set2(ch, 0, 0,1); 
                break;
        }
    }

    void tick2head() {
        if(millis() - lastChangeTime > BLINK_DUR) {
            if(curAspect==3) {
                OutputDriver::toggle(ch);
            } else
            if(curAspect==4) {
                OutputDriver::toggle(ch);
                OutputDriver::toggle(ch+1);
            }
            lastChangeTime = millis();
        }

    }


    void set3head() {
        switch(curAspect) {
            case 8:
                lastChangeTime = millis();
            case 1:
                OutputDriver::set2(ch, 0, 0,1 ); 
                OutputDriver::set(ch+2, 1); 
                break;
            case 5: 
                lastChangeTime = millis();
            case 2:
                OutputDriver::set2(ch, 0, 1,2 ); 
                OutputDriver::set(ch, 1); 
                break;
            case 6:
                lastChangeTime = millis();
            case 3:
                OutputDriver::set2(ch, 0, 0,2); 
                OutputDriver::set(ch+1, 1); 
                break;
            case 7:
                lastChangeTime = millis();
                OutputDriver::set2(ch, 0, 1,2); 
                OutputDriver::set(ch, 1); 
                break;
            case 9:
                lastChangeTime = millis();
                OutputDriver::set(ch+1, 0); 
                OutputDriver::set2(ch, 1, 0,2); 
                break;
            case 10: 
                OutputDriver::set2(ch, 0, 0,1);
                OutputDriver::set(ch+2, 0 );
                break;
        }
    }

    void tick3head() {
        if(millis() - lastChangeTime > BLINK_DUR) {
            switch(curAspect) {
                case 5:
                    OutputDriver::toggle(ch);
                    break;
                case 6:
                    OutputDriver::toggle(ch+1);
                    break;
                case 7:
                    if(OutputDriver::get(ch)) {
                        OutputDriver::toggle(ch);
                        OutputDriver::toggle(ch+2);
                    } else {
                        OutputDriver::toggle(ch+2);
                        OutputDriver::toggle(ch);
                    }
                    break;
                case 8: 
                    OutputDriver::toggle(ch+2); 
                    break;
            }
            lastChangeTime = millis();
        }
    }

    /*using fn = etl::add_pointer( void(Mast&) )::type;
    static fn tickFns[];
    static fn setFns[];*/

};




template<uint8_t MAX_MASTS, class OutputDriver>
class MastManager {

public:

    int16_t findAddr(uint16_t addr) {
        for(size_t i=0; i<masts.size(); i++) {
            if(masts[i].busAddr == addr) {
                return i;
            }
        }
        return -1;
    }

    void setAspect(uint16_t idx, uint8_t aspect) {
        masts[idx].setAspect(aspect);
    }

    uint8_t getAspect(uint16_t idx) {
        return masts[idx].getAspect();
    }

    using TMast = Mast<OutputDriver>;

    void addMast(uint16_t busAddr, uint16_t outputAddr, uint8_t nheads) {
        masts.push_back( TMast{busAddr, outputAddr, nheads} ) ;
    }

    void tick() {
        for(auto &m: masts) m.tick();
    }

private:
    
    etl::vector<TMast, MAX_MASTS> masts;
};

