#pragma once

#include <Arduino.h>

class SerialReader {
public:

    constexpr SerialReader():buf{0},  bufLen(0), parts{nullptr}, nParts(0) {
        
    }

    bool checkSerial(Stream &ser) {

        while (ser.available()) {
            char ch = (char)ser.read();
            switch(ch) {
                case '\n':
                case '\r': ch='\n'; break;
                default: if(bufLen<BUF_SIZE) buf[bufLen++] = ch; else bufLen = BUF_SIZE+1;
            }
            if(ch=='\n') {
                if(bufLen==BUF_SIZE+1) { bufLen=0; return false; } // discard all data if it overflowed buffer
                if(bufLen==0) return false;
                buf[bufLen]=0;
                bool t = tryParseResponse(buf, bufLen);
                bufLen = 0;
                return t;
            }
        }

        return false;

    }

    void drop() { nParts=0; }

    uint8_t bufParts() const { return nParts; }
    char* bufPart(uint8_t n) const { 
        if(n<nParts) return parts[n];
        return nullptr;
    }

private:
    static constexpr uint8_t BUF_SIZE = 20;
    char buf[BUF_SIZE+1];
    uint8_t bufLen;

    static constexpr uint8_t MAX_PARTS = 4;
    char* parts[MAX_PARTS]; // command of at most 3 parts
    uint8_t nParts;

    bool tryParseResponse(char* str, uint8_t sz) {
        nParts = 0;
        for(uint8_t p=0; p<sz; p++) if(str[p]==' ') str[p]=0;

        for(uint8_t p=0; p<sz; p++) {
            if( str[p]!=' ' && (p==0 || str[p-1]==0) ) {
                parts[nParts++] = &str[p];
                if(nParts==MAX_PARTS) break;
            }
        }
        return true;
    }
};