#include <LocoNet.h>

#include "SerialUtils.h"
#include "MastManager.h"
#include "PCA9685GPIO.h"

constexpr int PIN_OE = 4;
constexpr int PIN_VEN = 3;

constexpr int PIN_LED = 10;
constexpr int PIN_BT = 2;

constexpr int PIN_TX = 9;
constexpr int PIN_RX = 8;

constexpr int ADDR_OUT_COUNT = 16;
constexpr int ADDR_IN_COUNT = 8;

constexpr bool INPUT_PULLUP_EN = true;
constexpr int PIN_IN[ADDR_IN_COUNT] = {11, 12, A0, A1, A2, A3, 7, 6};

using PCADriver = PCA9685GPIO<PIN_OE>;
using TMastManager = MastManager<PCADriver::CH_OUT_COUNT/2, PCADriver>;
TMastManager masts;


uint16_t startInAddr;

bool configMode = false;
uint16_t configVar = 0;
uint32_t ledNextUpdate;
uint8_t ledVal;

constexpr uint16_t INPUT_DEBOUNCE_INTL = 10;

constexpr uint16_t BUTTON_LONG_PRESS_DURATION = 3000;

constexpr int LED_INTL_NORMAL = 1000;
constexpr int LED_INTL_CONFIG1 = 400;
constexpr int LED_INTL_CONFIG2 = 150;

uint8_t EEPROM_VER = PCADriver::EEPROM_VER + TMastManager::EEPROM_VER;

void ledFire(uint32_t, uint8_t);
void ledStop();
//static void sendOutput(uint8_t ch, uint16_t val);

void setup() {
    Serial.begin(115200);
    Serial.println(F("LND-L - LocoNet accessory decoder with 16 LEDs and 8 inputs"));
    
    pinMode(PIN_BT, INPUT);
    
    pinMode(PIN_LED, OUTPUT);
    ledFire(50,0);

    for(int i=0; i<ADDR_IN_COUNT; i++) {
        pinMode(PIN_IN[i], INPUT_PULLUP_EN ? INPUT_PULLUP : INPUT);
    }

    pinMode(PIN_VEN, OUTPUT);
    digitalWrite(PIN_VEN, HIGH); // disable 5Vo
    PCADriver::init();

    static_assert( 1 + PCADriver::EEPROM_REQUIRED + TMastManager::EEPROM_REQUIRED < E2END, "EEPROM size exceeded");
    uint8_t ver = EEPROM.read(0);
    int addr = 1;
    if(ver == EEPROM_VER) {
        PCADriver::load(addr);
        masts.load(addr);
    } else {
        Serial<<=F("Bad EEPROM, using default values");
        PCADriver::reset();
        //PCADriver::save(addr);

        masts.reset();
        masts.addMast(10, 3);
        //masts.save(addr);
    } 

    digitalWrite(PIN_VEN, LOW); // enable 5Vo
    
    LocoNet.init(PIN_TX);  

    //startOutAddr = sv.readSVNodeId(); 
    startInAddr = 10;
    
    Serial<< F("Input start address is ") <<= startInAddr;
    int i=0; 
    for(const auto &m: masts) {
        Serial<<F("Mast ")<<i<<F("; address=")<<m.busAddr()<<F("; output=")<<=m.ch;
        i++;
    }
    Serial<<= F("Init done");
}

int hex2int(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

// static inline void sendOutput(uint8_t ch, uint16_t val) {
//     pwm.setPin(ch, val, true);
// }

/*
void changeOutput(uint8_t ch, uint8_t val) {
    if(bitRead(output, ch)==val)  return;

    Serial<<F("Setting channel ")<<ch<<F(" to ")<<=val;

    int16_t dst = val * maxOutputVals[ch];
    if(fade) {
        int16_t src = (dst==0)?maxOutputVals[ch] : 0;
        //Serial.println(String("src=")+src+"; dst="+dst);
        for(uint8_t i=0; i<RES; i++) {
            uint16_t t = src + (dst-src)*i/RES;
            sendOutput(ch, t);
            delay(TRANS_TIME/RES);
        }
    }
    bitWrite(output, ch, val);
    sendOutput(ch, dst);
    reportChannelState(ch);
}*/

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
    if(bt==1 && lastBt==0) {
        //Serial.println("Button down");
        btPressTime = millis();
        if(configMode) Serial<<=F("Quitting config mode");
        configMode=false;
        delay(5); // simple debounce
        ledStop();
    }
    if(bt==1 && btPressTime!=0) {
        if(millis()-btPressTime>BUTTON_LONG_PRESS_DURATION && !configMode) { 
            configMode=true;
            configVar=0;
            Serial<<=F("Entering config mode");
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


void checkInputs() {
    static uint8_t lastIns[ADDR_IN_COUNT] = {0};
    static uint32_t primeTime[ADDR_IN_COUNT] = {0};
    for(int i=0; i<ADDR_IN_COUNT; i++) {
        uint8_t in = digitalRead(PIN_IN[i]);
        if(INPUT_PULLUP_EN) { 
            in = 1-in; // it's inverted
        }

        if(lastIns[i] != in && primeTime[i]==0) {
            primeTime[i] = millis();
            lastIns[i] = in;
        }
        if( (primeTime[i] != 0) && (millis()-primeTime[i]>=INPUT_DEBOUNCE_INTL) ) {
            if(lastIns[i] == in) {
                // in = jitter-free value
                uint16_t inAddr = startInAddr+i;
                Serial<<F("Input ")<<i<<"(addr "<<inAddr<<F(") changed to ")<<=in;
                ledFire(100);
                LocoNet.reportSensor(inAddr, in);
            }
            lastIns[i] = in;
            primeTime[i] = 0;
        }
    }
}

/** 
 * @see https://github.com/JMRI/JMRI/blob/master/java/src/jmri/jmrix/loconet/LNCPSignalMast.java#L71 
 *      for packet processing example
 * @see https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf for NMRA packet
 * @see https://github.com/JMRI/JMRI/blob/master/java/src/jmri/NmraPacket.java#L652 for NMRA packet address
*/
void processImmPacket(sendPktMsg &m) {
    if(m.val7f != 0x7F) return;
    uint8_t len = (m.reps & 0b01110000)>> 4;
    //Serial<<F("Imm packet of len ")<<=len;
    if(len!=3) return;
    uint8_t d[3] = { m.im1, m.im2, m.im3 };
    for(uint8_t i=0; i<3; i++) {
        if(m.dhi & (1<<i) ) d[i] |= 0b10000000;
    }
    if(d[0]>>6 != 0b10 && d[1]>>7 != 1) return; // not an accessory packet
    uint16_t addr =(( d[0] & 0b00111111)<<2 // mid
                  | ( d[1] & 0b110)>>1  // low
                  | (~d[1] & 0b01110000)<<(8-4) // high
                  ) + 1;
    Serial<<F("Imm address = ")<<=addr;
    //if(addr>=startOutAddr && addr<=startOutAddr+ADDR_OUT_COUNT) {
        uint8_t aspect = d[2]; 
        Serial<<F("aspect=")<<=aspect;
        ledFire(100);
        int16_t idx = masts.findAddr(addr);
        if(idx>=0)
            masts.setAspect(idx, aspect);

        //uint8_t ch = (addr-startOutAddr)*4 + aspect; 
        //uint8_t val = 1;
        //changeOutput(ch, val);    
    //}
}

void loop() {

    lnMsg *msg = LocoNet.receive() ;
    if ( msg!=nullptr ) {

        if(msg->data[0] == OPC_IMM_PACKET) {
            processImmPacket(msg->sp);
        } else if (!LocoNet.processSwitchSensorMessage(msg)) {
        } else {
        }
    }

    checkButton();

    checkInputs();

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

    masts.tick();

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
        int t = Serial.read();
        switch(t) {

            case 'h':
                Serial.println("HIGH");
                digitalWrite(PIN_OE, HIGH); // disable LED driver    
                break;
            case 'l':
                Serial.println("LOW");
                digitalWrite(PIN_OE, LOW);     
                break;
            case 'o':
                Serial.println("OUT");
                pinMode(PIN_OE, OUTPUT);
                break;
            case 'i':
                Serial.println("IN");
                pinMode(PIN_OE, INPUT);
                break;
            case ' ':
                masts.setAspect(0, 1-masts.getAspect(0) );    
                break;
            default:
                uint8_t ch = hex2int(t);
                if(ch>=0 && ch<16) {
                    //PCADriver::toggle(ch);
                    masts.setAspect(0, ch );    
                }
                break;

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
    
    Serial.print("Switch Request: ");
    Serial.print(addr, DEC);
    Serial.print(':');
    Serial.print(dir ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(out ? "On" : "Off");

    if(!on) return;

    if(!configMode) {
        uint16_t idx = masts.findAddr(addr);
        if(idx>=0) {
            ledFire(100);
            masts.setAspect(idx, thrown?1:0);
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
            //startOutAddr = addr;
            //sv.writeSvNodeId(startOut);
            Serial.print("Changed start address to ");
            //Serial.println(startOutAddr);
            configVar = 0;
        }
        ledFire(2000);
    }

}

/*
void reportChannelState(uint8_t ch) {
    uint16_t addr = startOutAddr+ch;
    addr -= 1;
    lnMsg txMsg;
    txMsg.srp.command = OPC_SW_REP;
    txMsg.srp.sn1 = addr & 0x7F;
    txMsg.srp.sn2 = ((addr >> 7) & 0x0F)  
        |  (bitRead(output,ch) ? OPC_SW_REP_THROWN: OPC_SW_REP_CLOSED)  ;
    LocoNet.send(&txMsg);
}
*/

