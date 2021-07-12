#include <LocoNet.h>

#include "SerialUtils.h"
#include "MastManager.h"
#include "PCA9685GPIO.h"
#include "SerialReader.h"
#include "LedBlinker.h"

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

using LED = LedBlinker<PIN_LED>;

using PCADriver = PCA9685GPIO<PIN_OE>;
using TMastManager = MastManager<PCADriver::CH_OUT_COUNT/2, PCADriver>;
TMastManager masts;

struct eeprom_cfg_t {
    uint8_t ver;
    uint16_t inAddr;
} __attribute__((packed));

uint16_t startInAddr;

bool configMode = false;
uint16_t configVar = 0;

constexpr uint16_t INPUT_DEBOUNCE_INTL = 10;

constexpr uint16_t BUTTON_LONG_PRESS_DURATION = 3000;

constexpr int LED_INTL_NORMAL = 1000;
constexpr int LED_INTL_CONFIG1 = 400;
constexpr int LED_INTL_CONFIG2 = 150;

constexpr int EEPROM_HEADER_SIZE = sizeof(eeprom_cfg_t);
constexpr int EEPROM_OUTPUTS_START = EEPROM_HEADER_SIZE;
constexpr int EEPROM_MASTS_START = EEPROM_OUTPUTS_START + PCADriver::EEPROM_REQUIRED;

constexpr uint8_t EEPROM_VER = PCADriver::EEPROM_VER ^ TMastManager::EEPROM_VER ^ EEPROM_HEADER_SIZE;

//static void sendOutput(uint8_t ch, uint16_t val);

void setup() {
    Serial.begin(115200);
    Serial.println(F("LND-L - LocoNet accessory decoder with 16 LEDs and 8 inputs"));
    
    pinMode(PIN_BT, INPUT);
    
    LED::begin();
    
    for(int i=0; i<ADDR_IN_COUNT; i++) {
        pinMode(PIN_IN[i], INPUT_PULLUP_EN ? INPUT_PULLUP : INPUT);
    }

    pinMode(PIN_VEN, OUTPUT);
    digitalWrite(PIN_VEN, HIGH); // disable 5Vo
    PCADriver::initHw();
    digitalWrite(PIN_VEN, LOW); // enable 5Vo

    static_assert( EEPROM_HEADER_SIZE + PCADriver::EEPROM_REQUIRED + TMastManager::EEPROM_REQUIRED < E2END, 
        "EEPROM size exceeded");
    eeprom_cfg_t cc;
    EEPROM.get<eeprom_cfg_t>(0, cc);
    if(cc.ver == EEPROM_VER) {
        startInAddr = cc.inAddr;  
        PCADriver::load(EEPROM_OUTPUTS_START);
        masts.load(EEPROM_MASTS_START);
    } else {
        Serial<<=F("Bad EEPROM, loading default values");
        startInAddr = 10;
        PCADriver::reset();
        masts.reset();
        //save();
    } 
    
    
    LocoNet.init(PIN_TX);  

    Serial<< F("Input start address is ") <<= startInAddr;
    listMasts();
    Serial<<= F("Init done");
}

void save() {
    eeprom_cfg_t cc{ .ver=EEPROM_VER, .inAddr=startInAddr };

    EEPROM.put<eeprom_cfg_t>(0, cc);;
    PCADriver::save(EEPROM_OUTPUTS_START);
    masts.save(EEPROM_MASTS_START);
}

void listMasts() {
    int i=0; 
    Serial<<F("Masts count: ")<<=masts.size();
    for(const auto &m: masts.getMasts() ) {
        Serial<<F("Mast ")<<i<<F(" addr:")<<m.busAddr()<<F(" channel:")<<m.ch<<F(" aspect:")<<=m.getAspect();
        i++;
    }
}

int hex2int(const char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

void setConfigMode(uint8_t stage, uint16_t var=0) {
    switch(stage) {
    case 0:
        if(configMode) Serial<<=F("Quitting config mode");
        configMode = false;
        LED::setIntl(LED_INTL_NORMAL);
        break;
    case 1:
        Serial<<=F("Waiting for cfg var");
        configMode = true;
        configVar = 0;
        LED::setIntl(LED_INTL_CONFIG1);
        break;
    case 2:
        Serial<<=F("Waiting for cfg value");
        configMode = true;
        configVar = var;
        Serial<<F("Selected var ")<<=configVar;
        LED::setIntl(LED_INTL_CONFIG2);
        break;
    }
}

void checkButton() {
    static uint8_t lastBt;
    static long btPressTime = 0;
    uint8_t bt = 1-digitalRead(PIN_BT); // it's inverted
    if(bt==1 && lastBt==0) {
        //Serial.println("Button down");
        btPressTime = millis();
        setConfigMode(0);
        delay(5); // simple debounce
        LED::pause();
    }
    if(bt==1 && btPressTime!=0) {
        if(millis()-btPressTime>BUTTON_LONG_PRESS_DURATION && !configMode) { 
            setConfigMode(1);
            Serial<<=F("Entering config mode");
            LED::resume(); // start blink again
        }
    }
    if(bt==0 && lastBt==1) {
        //Serial.println(String("Button was down for ")+(millis()-btPressTime));
        btPressTime = 0;
        delay(5); // simple debounce
        LED::resume(); // start blink again
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
                Serial<<F("Input ")<<i<<F("(addr ")<<inAddr<<F(") changed to ")<<=in;
                LED::fire(100);
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
    uint8_t aspect = d[2]; 
    Serial<<F("aspect=")<<=aspect;
    LED::fire(100);
    int16_t idx = masts.findAddr(addr);
    if(idx>=0)
        masts.setAspect(idx, aspect);

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

    LED::loop();

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

    static SerialReader ser;
    static mast_idx_t cmast=0;

    if(ser.checkSerial(Serial)) {
        char* cmd = ser.bufPart(0);
        if(strlen(cmd)==1) {
            int aspect = hex2int(cmd[0]);
            if(aspect>=0 && aspect < masts.getMasts()[cmast].getAspectCount() ) {
                masts.setAspect(cmast, aspect);
                Serial<<F("Set mast ")<<cmast<<':'<<=aspect;
            } else { Serial<<F("Bad aspect: ")<<=aspect;}
        } else
        if(strcmp(cmd, "mast")==0) {
            cmast = atoi(ser.bufPart(1));
            Serial<<F("Current mast idx ")<<=cmast;
        } else 
        if(strcmp(cmd, "addmast")==0) {
            int nh = atoi(ser.bufPart(1));
            int addr = atoi(ser.bufPart(2));
            masts.addMast(addr, nh);
            Serial<<F("Added mast addr=")<<addr<<F("; heads=")<<=nh;
        } else
        if(strcmp(cmd, "delmast")==0) {
            masts.deleteLastMast();
            Serial<<=F("Deleted last mast");
        } else
        if(strcmp(cmd, "clearmasts")==0) {
            masts.reset();
            Serial<<=F("Deleted all masts");
        } else
        if(strcmp(cmd, "listmasts")==0) {
            listMasts();
        } else
        if(strcmp(cmd, "br")==0) {
            PCADriver::channel_t ch = atoi(ser.bufPart(1));
            uint16_t mx = atoi(ser.bufPart(2));
            PCADriver::setMaxPWM( ch, mx );
            Serial<<F("Set max PWM for channel ")<<ch<<':'<<=mx;
        } else
        if(strcmp(cmd, "reset")==0) {
            Serial<<=F("Loading defaults");
            PCADriver::reset();
            masts.reset();
        } else 
        if(strcmp(cmd, "save")==0) {
            Serial<<=F("Saving to EEPROM");
            save();
        } else

        if(strcmp(cmd, "off")==0) {
            Serial<<=F("All masts off");
            for(auto& m: masts.getMasts() ) {
                m.setAspect(0);
            }
        } else 
        if(strcmp(cmd, "ch")==0)  {
            int ch = atoi(ser.bufPart(1));
            int v = atoi(ser.bufPart(2));
            Serial<<F("Setting ch ")<<ch<<F(" to ")<<=v;
            PCADriver::set(ch, v!=0);
        } else 

        if(strcmp(cmd, "inaddr")==0) {
            int addr = atoi(ser.bufPart(1));
            Serial<<F("Input start address set to ")<<=addr;
            startInAddr = addr;
        } else

        Serial<<=F("Unknown command");

    }

}


// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages  
// OPC_SW_REQ
void notifySwitchRequest( uint16_t addr, uint8_t out, uint8_t dir ) {
    bool on = out!=0;
    bool thrown = dir==0;
    
    Serial<<F("Switch Request: ")<<addr<<':'<< (dir?'C':'T') << ':' <<= (out?'+':'-');

    if(!on) return;

    if(!configMode) {
        for(auto &m: masts.getMasts() ) {
            if(addr >= m.busAddr() && addr<m.busAddr() + m.getAspectCount() ) {
                LED::fire(100);
                m.setAspect(addr-m.busAddr());
            }
        }
        return;
    } else {

        // set addr
        if(thrown) {
            if(configVar==0) {
                setConfigMode(2, addr);
            } else {
                //startOutAddr = addr;
                //sv.writeSvNodeId(startOut);
                //Serial.print(F("Changed start address to "));
                //Serial.println(startOutAddr);
                setConfigMode(0);
            }
            LED::fire(2000);
        }
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

