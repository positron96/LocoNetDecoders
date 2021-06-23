#include <LocoNet.h>

#include "PCA9685Driver.h"

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
template<class T> inline Print &operator <<=(Print &obj, T arg) { obj.println(arg); return obj; }


constexpr int PIN_VEN = 3;
constexpr int PIN_OEn = 4;

constexpr int PIN_LED = 10;
constexpr int PIN_BT = 2;

constexpr int PIN_TX = 9;
constexpr int PIN_RX = 8;

constexpr int ADDR_OUT_COUNT = 16;
constexpr int ADDR_IN_COUNT = 8;

constexpr bool INPUT_PULLUP_EN = true;
constexpr int PIN_IN[ADDR_IN_COUNT] = {11, 12, A0, A1, A2, A3, 7, 6};

PCA9685Driver pwm = PCA9685Driver(0x40);

LocoNetSystemVariableClass sv;

constexpr uint16_t SV_ADDR_FADING = SV_ADDR_USER_BASE;
constexpr uint16_t SV_ADDR_RESET = 255;//SV_ADDR_USER_BASE + 250;

constexpr uint16_t DEFAULT_ADDR = 10;
constexpr uint16_t DEFAULT_SERIAL = 0x0001;
constexpr uint8_t ID_MANFR = 13; // DIY
constexpr uint8_t ID_DEVLPR = 4;  
constexpr uint8_t ID_PRODUCT = 2;
constexpr uint8_t ID_SWVER = 1;

uint16_t startOutAddr;
uint16_t output;
constexpr int CH_OUT_COUNT = 16;
uint16_t maxOutputVals[CH_OUT_COUNT];

bool fade;

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

void ledFire(uint32_t, uint8_t);
void ledStop();
static void sendOutput(uint8_t ch, uint16_t val);

void setup() {
    Serial.begin(115200);
    Serial.println(F("LND-16L8I - LocoNet accessory decoder with 16 LEDs and 8 inputs"));
    
    pinMode(PIN_BT, INPUT);
    
    pinMode(PIN_LED, OUTPUT);
    ledFire(50,0);

    pinMode(PIN_VEN, OUTPUT);
    digitalWrite(PIN_VEN, HIGH); // disable LED driver    

    for(int i=0; i<ADDR_IN_COUNT; i++) {
        pinMode(PIN_IN[i], INPUT_PULLUP_EN ? INPUT_PULLUP : INPUT);
    }

    pinMode(PIN_OEn, OUTPUT);
    digitalWrite(PIN_OEn, HIGH); // disable pca
    
    Wire.begin();
    pwm.softResetAll();
    pwm.begin();
    //pwm.setPWMFreq(1600);
    //pwm.write8(  PCA9685_MODE1, pwm.read8(PCA9685_MODE1) | MODE1_AI  );
    
    sendOutput(PCA9685_ALL_LEDS, 0);
    pwm.setOpenDrainOutput();
    pwm.setInvertMode(true);
    pwm.setOeMode(PCA9685Driver::OeMode::HIGHZ);
    //pwm.begin();

    pwm.wakeup();    
    
    //Serial.println( pwm.read8(PCA9685_MODE1), BIN );
    //Serial.println( pwm.read8(PCA9685_MODE2), BIN );
    
    for(int i=0; i<CH_OUT_COUNT; i++) {
        maxOutputVals[i] = 1024;
    }
    //maxOutputVals[1] = 128;

    digitalWrite(PIN_VEN, LOW); // enable LED driver
    digitalWrite(PIN_OEn, LOW); // enable pca
    
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

    startOutAddr = sv.readSVNodeId(); 
    startInAddr = startOutAddr;
    fade = sv.readSVStorage(SV_ADDR_FADING) != 0;
    
    Serial<< F("Output start address is ") <<= startOutAddr;
    Serial<< F("Input start address is ") <<= startInAddr;
    Serial<< F("Fade is ") <<= fade?"On":"Off";

    Serial<<= F("Init done");
}

int hex2int(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

constexpr uint32_t TRANS_TIME = 100; // ms
constexpr uint8_t RES = 8;

static inline void sendOutput(uint8_t ch, uint16_t val) {
    pwm.setPWM(ch, val);
}

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
    if(bt==1 && lastBt==0) {
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
    Serial<<"Imm packet of len "<<=len;
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
    Serial<<"Imm address = "<<=addr;
    if(addr>=startOutAddr && addr<=startOutAddr+ADDR_OUT_COUNT) {
        uint8_t aspect = d[2]; 
        Serial<<"aspect="<<=aspect;
        ledFire(100);

        uint8_t ch = (addr-startOutAddr)*4 + aspect; 
        uint8_t val = 1;
        changeOutput(ch, val);    
    }
}

void loop() {
    static bool deferredProcessingNeeded = false;

    lnMsg *msg = LocoNet.receive() ;
    if ( msg!=nullptr ) {

        if(msg->data[0] == OPC_IMM_PACKET) {
            processImmPacket(msg->sp);
                        
        } else
        if (!LocoNet.processSwitchSensorMessage(msg)) {
            SV_STATUS svStatus = sv.processMessage(msg);
            Serial<<F("LNSV processMessage - Status: ")<<=svStatus;
            deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);
        } else {
        }
    }

    if(deferredProcessingNeeded)
        deferredProcessingNeeded = (sv.doDeferredProcessing() != SV_OK);

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
                Serial.println("OE HIGH");
                digitalWrite(PIN_OEn, HIGH); // disable LED driver    
                break;
            case 'l':
                Serial.println("OE LOW");
                digitalWrite(PIN_OEn, LOW);     
                break;
            case 'o':
                Serial.println("OE OUT");
                pinMode(PIN_OEn, OUTPUT);
                break;
            case 'i':
                Serial.println("OE IN");
                pinMode(PIN_OEn, INPUT);
                break;
            case 'r':
                break;
            case 'e':
                Serial<<="setting restart flag";
                pwm.restart();
                break;
            case 'z':  Serial<<="HighZ mode";  pwm.setOeMode(PCA9685Driver::OeMode::HIGHZ);  break;
            case 'w':  Serial<<="Wakeup";  pwm.wakeup();   break;
            case 'd':  Serial<<="Opendrain";  pwm.setOpenDrainOutput();   break;
            case 'R':  Serial<<="softreset";  pwm.softResetAll();   break;


/*
            case 'h':
                Serial.println("VEN HIGH");
                digitalWrite(PIN_VEN, HIGH); // disable 5Vo    
                break;
            case 'l':
                Serial.println("VEN LOW");
                digitalWrite(PIN_VEN, LOW);     
                break;
            case 'o':
                Serial.println("VEN OUT");
                pinMode(PIN_VEN, OUTPUT);
                break;
            case 'i':
                Serial.println("VEN IN");
                pinMode(PIN_VEN, INPUT);
                break;*/
            case ' ':
                changeOutput(0, 1-bitRead(output, 0));    
                break;
            default:
                uint8_t ch = hex2int(t);
                if(ch>=0 && ch<16) {
                    changeOutput(ch, 1-bitRead(output, ch));    
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
        if(addr >= startOutAddr && addr<startOutAddr+ADDR_OUT_COUNT) {
            ledFire(100);

            uint8_t ch = (addr-startOutAddr); // requested pin
            uint8_t val = thrown?1:0;
            changeOutput(ch, val);    
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
            startOutAddr = addr;
            //sv.writeSvNodeId(startOut);
            Serial.print("Changed start address to ");
            Serial.println(startOutAddr);
            configVar = 0;
        }
        ledFire(2000);
    }

}

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
