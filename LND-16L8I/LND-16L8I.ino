#include <LocoNet.h>

#include <Adafruit_PWMServoDriver.h>

constexpr int PIN_OE = 6;

constexpr int PIN_LED = 10;
constexpr int PIN_BT = 2;

constexpr int PIN_TX = 9;
constexpr int PIN_RX = 8;

constexpr int ADDR_OUT_COUNT = 16;
constexpr int ADDR_IN_COUNT = 8;

constexpr bool INPUT_PULLUP_EN = false;
constexpr int PIN_IN[ADDR_IN_COUNT] = {A0, A1, A2, A3, A5, A4, 5, 4};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

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
uint16_t maxOutputVals[ADDR_OUT_COUNT];

bool fade;

uint16_t startInAddr;

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

void setup() {
    Serial.begin(115200);
    Serial.println(F("LND-16L8I - LocoNet accessory decoder with 16 LEDs and 8 inputs"));
    
    pinMode(PIN_BT, INPUT);
    
    pinMode(PIN_LED, OUTPUT);
    ledFire(50,0);

    pinMode(PIN_OE, OUTPUT);
    digitalWrite(PIN_OE, HIGH); // disable LED driver    

    for(int i=0; i<ADDR_IN_COUNT; i++) {
        pinMode(PIN_IN[i], INPUT_PULLUP_EN ? INPUT_PULLUP : INPUT);
    }
    
    pwm.begin();
    pwm.setPWMFreq(1600);
    pwm.setOutputMode(false); // open drain
    for(int i=0; i<ADDR_OUT_COUNT; i++) {
        sendOutput(i, 0);
        maxOutputVals[i] = 4096;
    }

    digitalWrite(PIN_OE, LOW); // enable LED driver
    
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
    
    Serial.print(F("Output start address is "));
    Serial.println(startOutAddr);
    Serial.print(F("Input start address is "));
    Serial.println(startInAddr);
    Serial.print(F("Fade is "));
    Serial.println(fade?"On":"Off");

    Serial.println(F("Init done"));
}

int hex2int(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

// https://matthewearl.github.io/2015/03/05/efficient-pwm/
constexpr uint32_t TRANS_TIME = 100; // ms
constexpr uint8_t BITS = 2;
constexpr uint8_t RES = 1<<BITS; // 4 pwm values
constexpr uint8_t P = RES-1;

static inline void sendOutput(uint8_t ch, uint16_t val) {
    pwm.setPin(ch, val, true);
}

void changeOutput(uint8_t ch, uint8_t val) {
    if(bitRead(output, ch)==val)  return;

    Serial.print(F("Setting channel "));
    Serial.print(ch);
    Serial.print(F(" to "));
    Serial.println(val);

    if(fade) {
        uint16_t dst = val * maxOutputVals[ch];
        uint16_t src = (dst==0)?maxOutputVals[ch] : 0;
        for(int i=0; i<RES; i++) {
            uint16_t t = src + (dst-src)*i/RES;
            sendOutput(ch, t);
            delay(TRANS_TIME);
        }
    }
    bitWrite(output, ch, val);
    sendOutput(ch, val==0 ? 0 : maxOutputVals[ch]);
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
        if( (primeTime[i] != 0) && (millis()-primeTime[i]>=10) ) {
            if(lastIns[i] == in) {
                // in = jitter-free value
                uint16_t inAddr = startInAddr+i;
                Serial.print(F("Input pin "));
                Serial.print(inAddr);
                Serial.print(F(" changed to "));
                Serial.println(in);
                ledFire(100);
                LocoNet.reportSensor(inAddr, in);
            } 
            lastIns[i] = in;
            primeTime[i] = 0;
        }
    }
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
        uint8_t ch = hex2int(Serial.read());
        if(ch>=0 && ch<16) {
            changeOutput(ch, 1-bitRead(output, ch));    
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
