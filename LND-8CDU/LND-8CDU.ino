#include <LocoNet.h>

constexpr int PIN_TX = 9;
constexpr int PIN_RX = 8;

constexpr int PIN_ADC0 = A0; 
constexpr int PIN_ADC1 = A1;
constexpr int PIN_VCAP = PIN_ADC0;
constexpr int PIN_VCC = PIN_ADC1;

constexpr int PIN_LED = LED_BUILTIN;

// 9V at VOUT, 3V at ADC (3/5*1024=614 ticks), 
// 9000 / (9000/3/5000*1024) = 5000*3/1024
constexpr uint16_t ADC2MV = 15;
constexpr uint16_t MIN_VCC = 7000;
uint16_t vccLevel=0;
uint16_t capLevel=0;

constexpr int ADDR_COUNT = 4;

constexpr int PINS_O[] = {4,5,6,7,  10,11,A4,A5 };
constexpr int PIN_COUNT = 8;
bool states[PIN_COUNT];

uint8_t resetCnt __attribute__ ((section (".noinit")));
uint32_t initCode __attribute__ ((section (".noinit")));
constexpr uint32_t magic = 0xCAFEBABE;

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

constexpr int LED_INTL_NORMAL = 1000;
constexpr int LED_INTL_CONFIG1 = 500;
constexpr int LED_INTL_CONFIG2 = 250;

void ledFire(uint32_t, uint8_t);
void ledStop();

void setup() {
  Serial.begin(115200);
  Serial.println(F("LND-8CDU - LocoNet accessory decoder with 8 CDU outputs"));
  
  if(initCode!=magic) {
    resetCnt = 0;
    initCode = magic;
  } else {
    resetCnt++;
  }
  Serial.print(F("Reset count: "));  Serial.println(resetCnt);
  if(resetCnt>=3) {
    configMode = true;
    Serial.println(F("Entering config mode"));
  }

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
  pulseDurationMs = 6;//sv.readSVStorage(SV_ADDR_PULSE_DURATION);
  pulseCount = 2; //sv.readSVStorage(SV_ADDR_PULSE_COUNT);  
  
  Serial.println(F("Init done"));
  Serial.print(F("Address: ")); Serial.println(startAddr);
  Serial.print(F("Pulse duration (ms): ")); Serial.println(pulseDurationMs);
  Serial.print(F("Pulse count: ")); Serial.println(pulseCount);
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
void ledStop() {
  digitalWrite(PIN_LED, LOW); 
  ledNextUpdate = 0; // turn off blink
}


void loop() {
  static bool deferredProcessingNeeded = false;

  if(millis()>5000 && resetCnt!=0) {
    resetCnt=0; // wait 5 sec for user to press reset button
    Serial.println(F("Resetting resetCnt to 0"));
  }

  if (Serial.available()>0) {
    int8_t ch = hex2int(Serial.read());
    if(ch>=0 && ch<ADDR_COUNT*2) {
      pulsePinFull(ch);
    } ;//else notifySVChanged(SV_ADDR_RESET);
  }
  
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

void updateVoltages() {
  uint32_t v;
  uint32_t t;

  static uint32_t periodStart = millis();
  static uint16_t maxVcc = 0;
  constexpr uint32_t PERIOD = 10000;
  
  v = analogRead(PIN_VCC)*ADC2MV;
  if(v>maxVcc) maxVcc=v;
  if(v>vccLevel) vccLevel=v;
  if(millis()-periodStart > PERIOD) {
    vccLevel = maxVcc;
    maxVcc = v;
    periodStart = millis();
  }
  /*t = vccLevel;
  if(t==0) vccLevel = v;
  else vccLevel = (99*t + 1*v) / 100;*/

  v = analogRead(PIN_VCAP)*ADC2MV ;
  t = capLevel;
  if(t==0) capLevel = v;
  else capLevel = (60*t + 40*v) / 100;

  static uint32_t lastPrint = millis();
  if(millis()-lastPrint>25) {
    Serial.println(String(vccLevel)+","+capLevel);
    lastPrint=millis();
  }
}

inline void waitCharge() {
  updateVoltages();
  //Serial.println( String("waitCharge(); Vcap=")+capLevel+"; Vcc="+vccLevel );
  while((uint32_t)capLevel*100 < (uint32_t)vccLevel*98) {
    delay(1);
    updateVoltages();
  } 
}

inline void pulsePin(uint8_t pin) {
  //Serial.println( String("pulsePin(); Vcap=")+capLevel+"; Vcc="+vccLevel );
  digitalWrite(PINS_O[pin], HIGH);
  ledFire(100);
  long startTime = millis();
  while(millis()-startTime < pulseDurationMs
    && (uint16_t)analogRead(PIN_VCC)*ADC2MV>MIN_VCC ) {}
  digitalWrite(PINS_O[pin], LOW);
  //Serial.println( String("pulsePin(); duration(ms): ")+(millis()-startTime) );
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
