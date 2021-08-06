#pragma once

#include <stdint.h>
#undef min
#undef max
#include <etl/vector.h>
#include <etl/type_traits.h>
#include <EEPROM.h>
#include "SerialUtils.h"

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
    struct info_t {
        uint16_t addr:11;
        uint8_t nheads:3;
    };

    using ch_t = typename OutputDriver::channel_t;
    info_t info;
    ch_t ch;
    uint8_t curAspect;
    uint32_t lastChangeTime;

    static constexpr int EEPROM_REQUIRED = sizeof(info_t);

public:
    Mast(uint16_t busAddress, ch_t outputChannel, uint8_t nheads)
        : info{busAddress,nheads}, ch(outputChannel), curAspect(0), lastChangeTime(0)

    {

    }

    Mast(int &eepromaddr, ch_t &startCh):
        ch(startCh), curAspect(0), lastChangeTime(0) 
    {
        EEPROM.get<info_t>(eepromaddr, info);
        eepromaddr += EEPROM_REQUIRED;
        startCh += nheads();
    }

    void save(int &eepromAddr) const {
        EEPROM.put<info_t>(eepromAddr, info);
        eepromAddr += 2;
    }

    uint8_t nheads() const {
        return info.nheads;
    }

    uint16_t busAddr() const {
        return info.addr;
    }

    void setAspect(uint8_t aspect) {
        if(curAspect==aspect) return;
        curAspect = aspect;
        lastChangeTime = 0;
        switch(nheads() ) {
            case 1: set1head(); break;
            case 2: set2head(); break;
            case 3: set3head(); break;
        }
    }

    uint8_t getAspect() const { return curAspect; }

    void tick() {
        if(lastChangeTime==0) return;
        switch(nheads() ) {
            case 1: tick1head(); break;
            case 2: tick2head(); break;
            case 3: tick3head(); break;
        }
    }

    uint8_t getAspectCount() {
        switch(nheads()) {
            case 1: return 3;
            case 2: return 5;
            case 3: return 9;
            default: return 0;
        }
    }

private:

    void set1head() {
        switch(curAspect) {
            case 0: OutputDriver::set(ch, 0); break;
            case 2:
                lastChangeTime = millis();
            case 1: OutputDriver::set(ch, 1); break;
            
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
            case 0: 
                OutputDriver::template setn<2>(ch, false, 0,1); 
                break;
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
        }
    }

    void tick2head() {
        if(millis() - lastChangeTime > BLINK_DUR) {
            switch(curAspect) {
            case 3:
                OutputDriver::toggle(ch);
                break;
            case 4:
                if(OutputDriver::get(ch)) {
                    OutputDriver::set(ch+1, 1);
                    OutputDriver::set(ch, 0);
                } else {
                    OutputDriver::set(ch, 1);
                    OutputDriver::set(ch+1, 0);
                }
                break;
            }
            lastChangeTime = millis();
        }

    }


    void set3head() {
        switch(curAspect) {
            case 0: {
                OutputDriver::template setn<3>(ch, false, 0,1,2);
                break;
            }
            case 4:
                lastChangeTime = millis();
            case 1:
                OutputDriver::template setn<2>(ch, false, 1,2); 
                OutputDriver::set(ch, true); 
                break;
            case 5: 
                lastChangeTime = millis();
            case 2:
                OutputDriver::template setn<2>(ch, false, 0,2); 
                OutputDriver::set(ch+1, true); 
                break;
            case 6:
                lastChangeTime = millis();
            case 3:
                OutputDriver::template setn<2>(ch, false, 0,1); 
                OutputDriver::set(ch+2, true); 
                break;
            case 7:
                lastChangeTime = millis();
                OutputDriver::template setn<2>(ch, false, 1,2); 
                OutputDriver::set(ch, true); 
                break;
            case 8:
                lastChangeTime = millis();
                OutputDriver::set(ch+1, false); 
                OutputDriver::template setn<2>(ch, true, 0,2); 
                break;
        }
    }

    void tick3head() {
        if(millis() - lastChangeTime > BLINK_DUR) {
            switch(curAspect) {
                case 4:
                    OutputDriver::toggle(ch);
                    break;
                case 5:
                    OutputDriver::toggle(ch+1);
                    break;
                case 6: 
                    OutputDriver::toggle(ch+2); 
                    break;
                case 7:
                    if(OutputDriver::get(ch)) {
                        OutputDriver::set(ch+2, 1);
                        OutputDriver::set(ch, 0);
                    } else {
                        OutputDriver::set(ch, 1);
                        OutputDriver::set(ch+2, 0);
                    }
                    break;
            }
            lastChangeTime = millis();
        }
    }

};

using mast_idx_t = uint8_t;

template<mast_idx_t MAX_MASTS, class OutputDriver>
class MastManager {

public:
    using TMast = Mast<OutputDriver>;
    using MastsVector = etl::vector<TMast, MAX_MASTS>;
    using ch_t = typename TMast::ch_t;
    static constexpr uint8_t mast_idx_t_size = sizeof(mast_idx_t);

    int16_t findAddr(uint16_t addr) {
        for(mast_idx_t i=0; i<masts.size(); i++) {
            if(masts[i].busAddr() == addr) {
                return i;
            }
        }
        return -1;
    }

    void setAspect(mast_idx_t idx, uint8_t aspect) {
        masts[idx].setAspect(aspect);
    }

    uint8_t getAspect(mast_idx_t idx) {
        return masts[idx].getAspect();
    }

    bool addMast(uint16_t busAddr, uint8_t nheads) {
        ch_t ch=0;
        if(masts.size()>0) {
            const TMast &last = masts.back();
            ch = last.ch + last.nheads();
        }
        masts.push_back( TMast{busAddr, ch, nheads} ) ;
        return true;
    }

    void deleteLastMast() {
        if(masts.size()>0) masts.pop_back();
    }

    void clear() {
        masts.clear();
    }

    MastsVector & getMasts() { return masts; }


    static constexpr int EEPROM_REQUIRED = mast_idx_t_size + MAX_MASTS*TMast::EEPROM_REQUIRED;
    static constexpr uint8_t EEPROM_VER = 1 ^ EEPROM_REQUIRED;

    bool load(int eepromAddr) {
        clear();

        mast_idx_t count;
        EEPROM.get<mast_idx_t>(eepromAddr, count);
        if(count>MAX_MASTS) return false;
        eepromAddr += mast_idx_t_size;
        ch_t ch=0;
        for(mast_idx_t i=0; i<count; i++) {
            TMast t{eepromAddr, ch};
            masts.push_back( t );
        }
        return true;
    }

    void reset() {   clear();    }

    size_t size() { return masts.size(); }

    bool save(int eepromAddr) {
        EEPROM.put<mast_idx_t>(eepromAddr, masts.size() );
        eepromAddr += mast_idx_t_size;
        for(const auto &m: masts) {
            m.save(eepromAddr);
        }
        return true;
    }

    void tick() {
        for(auto &m: masts) m.tick();
    }

private:
    MastsVector masts;
};

