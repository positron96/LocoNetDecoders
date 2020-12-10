#include <LocoNet.h>
#include <SPI.h>
#include <MsTimer2.h>

#include "pdm.h"

constexpr int PIN_OE = 6;
constexpr int PIN_LE = 7;
constexpr int PIN_LED = 10;
constexpr int PIN_BT = 2;

constexpr int PIN_TX = 9;
constexpr int PIN_RX = 8;

constexpr int ADDR_COUNT = 16;

LocoNetSystemVariableClass sv;

constexpr uint16_t SV_ADDR_FADING = SV_ADDR_USER_BASE;
constexpr uint16_t SV_ADDR_RESET = 255;//SV_ADDR_USER_BASE + 250;

constexpr uint16_t DEFAULT_ADDR = 10;
constexpr uint16_t DEFAULT_SERIAL = 0x0001;
constexpr uint8_t ID_MANFR = 13; // DIY
constexpr uint8_t ID_DEVLPR = 4;  
constexpr uint8_t ID_PRODUCT = 2;
constexpr uint8_t ID_SWVER = 1;

uint16_t startAddr;
bool fade;
using pdm = PDM<ADDR_COUNT>;

uint8_t maxVals[ADDR_COUNT] = { 0 };

bool configMode = false;
uint16_t configVar = 0;
uint32_t ledNextUpdate;
uint8_t ledVal;

constexpr uint16_t BUTTON_LONG_PRESS_DURATION = 3000;

constexpr int LED_INTL_NORMAL = 1000;
constexpr int LED_INTL_CONFIG1 = 400;
constexpr int LED_INTL_CONFIG2 = 150;

void ledFire(uint32_t, uint8_t);
void ledStop();

void timerTick();

void setup() {
    Serial.begin(115200);
    Serial.println(F("LND-16L8I - LocoNet accessory decoder with 16 LEDs and 8 inputs"));
    
    pinMode(PIN_BT, INPUT);
    
    pinMode(PIN_LED, OUTPUT);
    ledFire(50,0);
    
    pinMode(PIN_OE, OUTPUT);
    digitalWrite(PIN_OE, LOW);
    pinMode(PIN_LE, OUTPUT);
    
    pdm::init();
    for(int ch=0; ch<ADDR_COUNT; ch++) maxVals[ch]=255;
    maxVals[0] = 30;
    SPI.begin();
    
    LocoNet.init(PIN_TX);  

    sv.init(ID_MANFR, ID_DEVLPR, ID_PRODUCT, ID_SWVER);
    uint16_t serial = 
          sv.readSVStorage(SV_ADDR_SERIAL_NUMBER_H)<<8 
        | sv.readSVStorage(SV_ADDR_SERIAL_NUMBER_L);

    if(serial != DEFAULT_SERIAL) {  
        Serial.println(F("Writing factory defaults") );
        sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, DEFAULT_SERIAL>>8);
        sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_L, DEFAULT_SERIAL & 0xFF);
        sv.writeSVNodeId(DEFAULT_ADDR);
    }

    startAddr = sv.readSVNodeId(); 
    fade = sv.readSVStorage(SV_ADDR_FADING) != 0;
    
    Serial.println(F("Init done"));
    Serial.print(F("Address is "));
    Serial.println(startAddr);
    Serial.print(F("Fade is "));
    Serial.println(fade?"On":"Off");

    MsTimer2::set(1, timerTick); // 1000 fps
    MsTimer2::start();
}

int hex2int(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}


constexpr uint32_t TRANS_TIME = 100; // transition time, ms
constexpr uint8_t RES = TRANS_TIME/25; // "frames" for full transition


void timerTick() {
    pdm::tick();
    digitalWrite(PIN_LE, LOW);
    SPI.transfer16( pdm::values() & 0xFFFF ); 
    digitalWrite(PIN_LE, HIGH);
}

void changeOutput(uint8_t ch, uint8_t val) {
    uint8_t to = val>0 ? maxVals[ch] : 0;
    uint8_t from = pdm::get(ch);

    Serial.print("Setting channel ");
    Serial.print(ch);
    Serial.print(" to ");
    Serial.println(to);

    if( to == from ) return;

    if(fade) {
        for(int i=0; i<RES; i++) {
            uint8_t pwm = from + (to-from)*i/RES;
            pdm::set(ch, pwm);
            delay(TRANS_TIME/RES);
        }
    }
    pdm::set(ch, to);
}

void ledFire(uint32_t ms, uint8_t val=1) {
    ledVal = val;
    digitalWrite(PIN_LED, ledVal);  
    ledNextUpdate = millis()+ms;
}
void ledOn() {
    if(ledNextUpdate!=0) return;
    ledFire(0,0);
}
void ledStop() {
    digitalWrite(PIN_LED, LOW); 
    ledNextUpdate = 0; // turn off blink
}

void checkButton() {
    static uint8_t lastBt;
    static long btPressTime = 0;
    uint8_t bt = 1-digitalRead(PIN_BT); // it's inverted
    if(lastBt==0 && bt==1) {
        //Serial.println("Button down");
        btPressTime = millis();
        if(configMode) Serial.println("Quitting config mode");
        configMode=false;
        delay(5); // simple debounce
        ledStop();
    }
    if(bt==1 && btPressTime!=0) {
        if(millis()-btPressTime>BUTTON_LONG_PRESS_DURATION && !configMode) { 
            configMode=true;
            configVar=0;
            Serial.println("Entering config mode");
            ledOn(); // start blink again
        }
    }
    if(bt==0 && lastBt==1) {
        //Serial.println(String("Button was down for ")+(millis()-btPressTime));
        btPressTime = 0;
        delay(5); // simple debounce
        ledOn(); // start blink again
    }
    lastBt = bt;
}

void loop() {
    static bool deferredProcessingNeeded = false;

    lnMsg *msg = LocoNet.receive() ;
    if ( msg!=nullptr ) {
        
        if (!LocoNet.processSwitchSensorMessage(msg)) {
            SV_STATUS svStatus = sv.processMessage(msg);
            Serial.print("LNSV processMessage - Status: ");
            Serial.println(svStatus);
            deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);
        } else {
        }
    }

    if(deferredProcessingNeeded)
        deferredProcessingNeeded = (sv.doDeferredProcessing() != SV_OK);

    checkButton();

    if(ledNextUpdate!=0 && millis()>ledNextUpdate) {
        ledVal = 1-ledVal;
        digitalWrite(PIN_LED, ledVal);
        
        if(!configMode) {
            ledNextUpdate = LED_INTL_NORMAL; 
        } else {
            ledNextUpdate = configVar==0 ? LED_INTL_CONFIG1 : LED_INTL_CONFIG2;
        }
        ledNextUpdate += millis();
    }

    /*
    static long nextInRead = 0;
    static int lastV=0;
    if(millis()>nextInRead) {
        int v = 1-digitalRead(PIN_BT);
        if(v!=lastV) {
            //LocoNet.reportSensor(11, v==1);// it's pulled up when idle.
            lastV = v;
            nextInRead = millis()+10;
        }
    }
    */

    
    if (Serial.available()>0) {
        uint8_t ch = hex2int(Serial.read());
        if(ch>=0 && ch<16) {
            changeOutput(ch, pdm::get(ch)>0?0:1 ); 
        }
    }

    /*
     
    static long nextUpdate = millis();
    if(millis()> nextUpdate ) {
        for(int i=0; i<16; i++) {
            if(h2[i]<led_pwm[i]) {
                bitSet(output, i);
                h2[i] += P-led_pwm[i];
            } else {
                bitClear(output, i);
                h2[i] -= led_pwm[i];
            }
        }
        //Serial.println(String("i=0")+" pwm="+led_pwm[0]+" h2="+h2[0]);
        digitalWrite(PIN_LE, LOW);
        SPI.transfer16( output ); 
        digitalWrite(PIN_LE, HIGH);
        nextUpdate = millis()+1;
    }
    */
    /*for (int channel = 0; channel < 16; channel++) {
        digitalWrite(PIN_LE, LOW);
        SPI.transfer16( (uint16_t) (1 << channel) ); 
        digitalWrite(PIN_LE, HIGH);
        delay(250);
        Serial.println( (uint16_t)1<<channel, BIN);
    }*/

}


// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages  
// OPC_SW_REQ
void notifySwitchRequest( uint16_t addr, uint8_t out, uint8_t dir ) {
    bool on = out!=0;
    bool thrown = dir==0;
    /*Serial.print("Switch Request: ");
    Serial.print(addr, DEC);
    Serial.print(':');
    Serial.print(dir ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(out ? "On" : "Off");*/

    if(!on) return;

    if(!configMode) {
        if(addr >= startAddr && addr<startAddr+ADDR_COUNT) {
            uint8_t ch = (addr-startAddr); // requested pin
            uint8_t val = thrown?1:0;
            changeOutput(ch, val);    
            reportChannelState(ch);
        }
        return;
    }

    // set addr
    if(thrown) {
        if(configVar==0) {
            configVar = addr;
            Serial.print("Var=");
            Serial.println(configVar);
        } else {
            startAddr = addr;
            //sv.writeSvNodeId(startAddr);
            Serial.print("Changed start address to ");
            Serial.println(startAddr);
            configVar = 0;
        }
        ledFire(2000);
    }


}

void reportChannelState(uint8_t ch) {
    uint16_t addr = startAddr+ch;
    addr -= 1;
    lnMsg txMsg;
    txMsg.srp.command = OPC_SW_REP;
    txMsg.srp.sn1 = addr & 0x7F;
    txMsg.srp.sn2 = ((addr >> 7) & 0x0F)  
        |  (pdm::get(ch)>0 ? OPC_SW_REP_THROWN: OPC_SW_REP_CLOSED)  ;
    LocoNet.send(&txMsg);
}


void notifySVChanged(uint16_t offs){
    Serial.print("notifySVChanged: SV");
    Serial.print(offs);
    Serial.print("->");
    Serial.println(sv.readSVStorage(offs));
    switch(offs) {
        case SV_ADDR_RESET: 
            // this will factory reset on reset
            sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, 10); 
            sv.writeSVStorage(SV_ADDR_RESET, 0); 
            break;
        case SV_ADDR_FADING:
            fade = sv.readSVStorage(SV_ADDR_FADING)!=0;
            break;   
    }
}
