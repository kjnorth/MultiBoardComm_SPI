/** @author Kodiak North
 * @date 05/21/2020
 * test project with NRF24l01 module in PTX mode, to 
 * communicate with another NRF24l01 module in PRX mode
 */

#include <Arduino.h>
#include "ConfigPTX.h"
#include "..\lib\DataLog\DataLog.h"
#include "nRF24L01.h"
#include "RF24.h"

void InitRadio(void);
bool IsConnected(void);
void UpdateTruckData(void);

TX_TO_RX ttr;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

void setup() {
  Serial.begin(115200);
  LogInfo("SPI Master boots up\n");
  
  InitRadio();
}

unsigned long curTime = 0;
unsigned long preLogTime = 0;
unsigned long preSendTime = 0;
unsigned long lastReceiveTime = 0;
static RX_TO_TX rtt;
void loop() {
  curTime = millis();

  // right front nano comm
  static uint8_t send = 0xA0;
  static uint8_t rec = 0x00;

  int32_t ires = 0;
  float receivedFloat = (float)(ires/1000.0);
  if (curTime - preLogTime >= 1000) {
    LogInfo(F("switchStatus 0x%X, solenoid Status 0x%X, isConnected %d, "),
                rtt.SwitchStatus, rtt.SolenoidStatus, IsConnected(), rec);
    LogInfo("received float ", receivedFloat, 3, true);
    preLogTime = curTime;
  }

  // truck comm
  IsConnected();
  if (curTime - preSendTime >= 10) { // write data to truck PRX at 100Hz frequency
    UpdateTruckData();
    preSendTime = curTime;
  }
}

void InitRadio(void) {
  ttr.Phase = 0xA4;
  ttr.LEDControl = 0xB7;
  ttr.FrontEncoder = 157291;

#ifdef RF_USE_IRQ_PIN
  pinMode(RF_IRQ_PIN, INPUT);
#endif

  if (!radio.begin())
    Serial.println("PTX failed to initialize");
  else {
    // RF24 library begin() function enables PTX mode
    radio.setAddressWidth(5); // set address size to 5 bytes
    radio.setRetries(1, 5); // set 5 retries with 500us delays in between
    radio.setChannel(RF_CHANNEL); // set communication channel
    radio.setPayloadSize(NUM_TTR_BYTES); // set payload size to number of bytes being SENT
    radio.enableAckPayload(); // enable payload attached to ACK from PRX
    radio.setPALevel(RF24_PA_LOW); // set power amplifier level. Using LOW for tests on bench. Should use HIGH on PL/Truck
    radio.setDataRate(RF24_1MBPS); // set data rate to most reliable speed
    radio.openWritingPipe(RF_PTX_WRITE_ADDR); // open the writing pipe on the address we chose
    Serial.println("PTX initialization successful");
  }
}

bool IsConnected(void) {
  static bool conn = false;
  if (curTime - lastReceiveTime >= 250 && conn) {
    LogInfo("connection to PRX is lost!\n");
    conn = false;
  }
  else if (lastReceiveTime > 0 && curTime - lastReceiveTime < 250 && !conn) {
    LogInfo("established connection to PRX\n");
    conn = true;
  }
  return conn;
}

void UpdateTruckData(void) {
#ifdef RF_USE_IRQ_PIN  
  if (LOW == digitalRead(RF_IRQ_PIN)) {
    bool tx_ok=false, tx_fail=false, rx_ready=false;
    radio.whatHappened(tx_ok, tx_fail, rx_ready);
    if (tx_ok || rx_ready) {
#endif      
      if (radio.isAckPayloadAvailable()) {
        radio.read(&rtt, NUM_RTT_BYTES);
        lastReceiveTime = curTime;
      }
#ifdef RF_USE_IRQ_PIN      
    }
  }
#endif

  ttr.Phase++;
  ttr.LEDControl++;
  ttr.FrontEncoder++;
#ifdef RF_USE_IRQ_PIN    
  radio.startFastWrite(&ttr, NUM_TTR_BYTES, 0);
#else    
  radio.writeFast(&ttr, NUM_TTR_BYTES, 0);
#endif   
}