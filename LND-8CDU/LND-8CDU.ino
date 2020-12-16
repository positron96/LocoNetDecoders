#include <LocoNet.h>

constexpr int PIN_TX = 9;
constexpr int PIN_RX = 8;

constexpr int PIN_BT = 12;

constexpr int PIN_ADC0 = A0; 
constexpr int PIN_ADC1 = A1;
constexpr int PIN_VCAP = PIN_ADC0;
constexpr int PIN_VCC = PIN_ADC1;

constexpr int PIN_CHARGE = A3;

constexpr int PIN_LED = LED_BUILTIN;

constexpr uint16_t VCC_MAX_DROP = 500; // mV
// 9V at VOUT, 3V at ADC (3/5*1024=614 ticks), 
// 9000 / (9000/3/5000*1024) = 5000*3/1024
constexpr uint16_t ADC2MV = 15;
constexpr uint16_t MIN_VCC = 7000; // arduino works at 7-12V, do not allow VCC to fall lower than this.
volatile uint16_t vccLevel=0;
volatile uint16_t vccLevelAvg=0;
volatile uint16_t capLevel=0;

constexpr int ADDR_COUNT = 4;

constexpr int PINS_O[] = {4,5,6,7,  10,11,A4,A5 };
constexpr int PIN_COUNT = 8;
bool states[PIN_COUNT];

LocoNetSystemVariableClass sv;

constexpr uint16_t SV_ADDR_PULSE_DURATION = SV_ADDR_USER_BASE;
constexpr uint16_t SV_ADDR_PULSE_COUNT = SV_ADDR_USER_BASE + 1;
constexpr uint16_t SV_ADDR_RESET = 255;//SV_ADDR_USER_BASE + 250;

constexpr uint16_t DEFAULT_ADDR = 6;
constexpr uint16_t DEFAULT_SERIAL = 0x0001;
constexpr uint8_t ID_MANFR = 13;
constexpr uint8_t ID_DEVLPR = 4;
constexpr uint8_t ID_PRODUCT = 1;
constexpr uint8_t ID_SWVER = 1;

uint16_t startAddr = DEFAULT_ADDR;
uint8_t pulseDurationMs;
uint8_t pulseCount;

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
    Serial.println(F("LND-8CDU - LocoNet accessory decoder with 8 CDU outputs"));
    
    pinMode(PIN_BT, INPUT);
    for (int pin: PINS_O) pinMode(pin, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    ledFire(50,0);

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
        sv.writeSVStorage(SV_ADDR_PULSE_DURATION, 10);
        sv.writeSVStorage(SV_ADDR_PULSE_COUNT, 1);
    }

    startAddr = sv.readSVNodeId(); 
    pulseDurationMs = 50;//sv.readSVStorage(SV_ADDR_PULSE_DURATION);
    pulseCount = sv.readSVStorage(SV_ADDR_PULSE_COUNT);  
    
    Serial.println(F("Init done"));
    Serial.print(F("Address: ")); Serial.println(startAddr);
    Serial.print(F("Pulse duration (ms): ")); Serial.println(pulseDurationMs);
    Serial.print(F("Pulse count: ")); Serial.println(pulseCount);

    // output toggle pin

    pinMode(PIN_CHARGE, OUTPUT);
    digitalWrite(PIN_CHARGE, LOW);

    // ADC
    ADMUX = bit(REFS0) | 0;//(adcCh & 0xF);  
    // Auto trigger + Int En + div 128
    //ADCSRA = bit(ADATE) | bit(ADIE) | 0x7;
    // Int En + div 128
    ADCSRA = bit(ADIE) | 0x7;
    // Free running
    ADCSRB = 0;
    // Enable + Start conversion
    ADCSRA |= bit(ADEN) | bit(ADSC) ;
}

int8_t hex2int(char ch) {
    if (ch >= '0' && ch <= '9') { return ch - '0'; }
    if (ch >= 'A' && ch <= 'F') { return ch - 'A' + 10; }
    if (ch >= 'a' && ch <= 'f') { return ch - 'a' + 10; }
    return -1;
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
        if(configMode) Serial.println(F("Quitting config mode"));
        configMode=false;
        delay(5); // simple debounce
        ledStop();
    }
    if(bt==1 && btPressTime!=0) {
        if(millis()-btPressTime>BUTTON_LONG_PRESS_DURATION && !configMode) { 
            configMode=true;
            configVar=0;
            Serial.println(F("Entering config mode"));
            ledOn(); // start blink again
        }
    }
    if(bt==0 && lastBt==1) {
        uint32_t dur = millis() - btPressTime;
        btPressTime = 0;
        delay(5); // simple debounce
        ledOn(); // start blink again

        //Serial.println(String("Button was down for ")+dur);

        if(dur<1000) {
            // do something useful, e.g. send LocoNet or trigger
            pulsePinFull(0);
        }
    }
    lastBt = bt;
}

void loop() {

    if (Serial.available()>0) {
        int8_t ch = hex2int(Serial.read());
        if(ch>=0 && ch<ADDR_COUNT*2) {
            pulsePinFull(ch);
        } ;//else notifySVChanged(SV_ADDR_RESET);
    }

    static bool deferredProcessingNeeded = false;
    
    // Check for any received LocoNet packets
    lnMsg *msg = LocoNet.receive() ;
    if ( msg!=nullptr ) {

        // If this packet was not a Switch or Sensor Message then print a new line
        if (!LocoNet.processSwitchSensorMessage(msg)) {
            SV_STATUS svStatus = sv.processMessage(msg);
            Serial.print(F("LNSV processMessage - Status: "));
            Serial.println(svStatus);
            deferredProcessingNeeded = (svStatus == SV_DEFERRED_PROCESSING_NEEDED);
        } else {
        }
    }

    if(deferredProcessingNeeded)
        deferredProcessingNeeded = (sv.doDeferredProcessing() != SV_OK);

    checkButton();

    static uint32_t lastMs = millis();
    if(millis()-lastMs > 25) {    
        lastMs = millis();
        updateVoltages();
        /*Serial.print( vccLevel );
        Serial.print(',');
        Serial.println( capLevel ); */
    }
    
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

}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages  OPC_INPUT_REP
/*
void notifySensor( uint16_t addr, uint8_t State ) {
  Serial.print("Sensor: ");
  Serial.print(addr, DEC);
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}
*/

/**
 * Sped up version of digitalWrite
 */
inline void togglePower(bool v) {
    //digitalWrite(PIN_CHARGE, v?HIGH:LOW);
    static uint8_t bit = digitalPinToBitMask(PIN_CHARGE);
    static volatile uint8_t *port = portOutputRegister( digitalPinToPort(PIN_CHARGE) ); 
    if (v) {
        *port |= bit;
    } else {
        *port &= ~bit;
    }
}


constexpr uint8_t N_CH = 2;
volatile bool haveAdc = false;
constexpr int ADC_SLOWDOWN = 1;


ISR(ADC_vect) {  
    
    if(ADC_SLOWDOWN>1) {
        static uint8_t prescl = 0;
        prescl++;
        if(prescl==ADC_SLOWDOWN) { prescl=0; } else return;
    }
    
    static uint8_t curCh = 0; 
    uint32_t mV = ADCL;
    mV |= ADCH<<8;
    mV *= ADC2MV;
    
    if(curCh==1) { // VCC
        vccLevel = mV;
        togglePower( mV > vccLevelAvg - (int16_t)VCC_MAX_DROP);
    } else {  // CAP
        uint32_t t = capLevel;
        if(t==0) t = mV;
        capLevel = (60*t + 40*mV) / 100;
    }

    curCh++;
    if(curCh==N_CH) {curCh=0; haveAdc = true;}
    
    ADMUX = (ADMUX & B11110000) | curCh;
    //ADMUX = ADMUX ~ 0x3; // flip bits 0 and 1
    ADCSRA |= bit(ADSC) ;
}

void updateVoltages() {
    static uint32_t periodStart = millis();
    constexpr uint32_t PERIOD = 20000;
    
    if(vccLevel > vccLevelAvg) vccLevelAvg = vccLevel;
    if(millis()-periodStart > PERIOD) {
        vccLevelAvg = vccLevel;
        periodStart = millis();
    }

    static uint32_t lastPrint = millis();
    if(millis()-lastPrint>25 && ((uint32_t)capLevel*100 < (uint32_t)vccLevelAvg*99) ) {
        Serial.println(String(vccLevel)+", "+vccLevelAvg+", "+capLevel+", "+(digitalRead(PIN_CHARGE)*5000));
        lastPrint=millis();
    }
}

inline void waitCharge() {
    updateVoltages();
    //Serial.println( String("waitCharge(); Vcap=")+capLevel+"; Vcc="+vccLevel );
    while( (uint32_t)capLevel*100 < (uint32_t)vccLevelAvg*98 ) {
        delay(1);
        updateVoltages();
    } 
}

inline void pulsePin(uint8_t pin) {
    //Serial.println(String("pulsePin: ")+vccLevel+", "+vccLevelAvg+", "+capLevel);
    digitalWrite(PINS_O[pin], HIGH);
    ledFire(100);
    long startTime = millis();
    while(millis()-startTime < pulseDurationMs && vccLevel>MIN_VCC ) {}
    digitalWrite(PINS_O[pin], LOW);
    Serial.println( String("pulsePin(); duration(ms): ")+(millis()-startTime) );
}

inline void pulsePinFull(uint8_t ipin) {
    for(int i=0; i<pulseCount; i++) {
        waitCharge();
        pulsePin(ipin);
    }
}

void reportSwitchState(uint16_t addr) {
    int ipin = (addr-startAddr)*2; // "Closed" pin
    addr -= 1;
    lnMsg txMsg;
    txMsg.srp.command = OPC_SW_REP;
    txMsg.srp.sn1 = addr & 0x7F;
    txMsg.srp.sn2 = ((addr >> 7) & 0x0F)  
        |  (states[ipin  ] ? OPC_SW_REP_CLOSED : 0)
        |  (states[ipin+1] ? OPC_SW_REP_THROWN : 0);
    LocoNet.send(&txMsg);
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

    if(configMode) {
        if(configVar == 0) {
            configVar = addr;      
        } else {
            switch(configVar) {
                case 1:
                    startAddr = addr;
                    sv.writeSVNodeId(startAddr);
                    break;
                default:
                    sv.writeSVStorage(SV_ADDR_USER_BASE+configVar-1, addr);
                    break;
            }
            configVar=0;
            ledFire(2000);
        }
        return;
    }

    if(addr >= startAddr && addr<startAddr+ADDR_COUNT) {
        int ipin = (addr-startAddr)*2; // requested pin
        int ipin_compl; // complementary pin
        if(thrown) { ipin_compl = ipin+1; } else {ipin_compl=ipin; ipin++;}
        states[ipin] = true;
        states[ipin_compl] = false;
        digitalWrite(PINS_O[ipin_compl], LOW); // turn off complementary pin
        
        pulsePinFull(ipin);
        
        reportSwitchState(addr);
    }
}

void(* reboot) (void) = 0;

void notifySVChanged(uint16_t offs){
    Serial.print(F("notifySVChanged: SV"));
    Serial.print(offs);
    Serial.print(":=");
    Serial.println(sv.readSVStorage(offs));
    switch(offs) {
        case SV_ADDR_RESET: 
            sv.writeSVStorage(SV_ADDR_SERIAL_NUMBER_H, 0xFF); 
            sv.writeSVStorage(SV_ADDR_RESET, 0);
            delay(10);
            reboot();
            break;
        case SV_ADDR_PULSE_DURATION:
            pulseDurationMs = sv.readSVStorage(SV_ADDR_PULSE_DURATION);
            break; 
        case SV_ADDR_PULSE_COUNT:
            pulseCount = sv.readSVStorage(SV_ADDR_PULSE_COUNT);  
            break;      
    }
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch out Report messages  
// OPC_SW_REP and !LnPacket->srp.sn2 & OPC_SW_REP_INPUTS)
/*
void notifySwitchoutsReport( uint16_t addr, uint8_t Closedout, uint8_t Thrownout) {
  Serial.print("Switch outs Report: ");
  Serial.print(addr, DEC);
  Serial.print(": Closed - ");
  Serial.print(Closedout ? "On" : "Off");
  Serial.print(": Thrown - ");
  Serial.println(Thrownout ? "On" : "Off");
}
*/

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Sensor Report messages
// if OPC_SW_REP and LnPacket->srp.sn2 & OPC_SW_REP_INPUTS
/*
void notifySwitchReport( uint16_t addr, uint8_t State, uint8_t Sensor ) {
  Serial.print("Switch Sensor Report: ");
  Serial.print(addr, DEC);
  Serial.print(':');
  Serial.print(Sensor ? "Switch" : "Aux");
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
}
*/

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch State messages
// OPC_SW_STATE
/*
void notifySwitchState( uint16_t addr, uint8_t out, uint8_t dir ) {
  Serial.print("Switch State: ");
  Serial.print(addr, DEC);
  Serial.print(':');
  Serial.print(dir ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(out ? "On" : "Off");
}
*/
