/** @author Kodiak North
 * @date 05/21/2020
 * test project with NRF24l01 module in PRX mode, to 
 * communicate with another NRF24l01 module in PTX mode
 */

#include <Arduino.h>
#include "ConfigPRX.h"
#include "..\lib\DataLog\DataLog.h"
#include "nRF24L01.h"
#include "RF24.h"

bool IsConnected(void);

RX_TO_TX rtt;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

void setup() {
  Serial.begin(115200);

  rtt.SwitchStatus = 1;//0x9D;
  rtt.SolenoidStatus = 0x5E;
  // rtt.Count = 0;

#ifdef RF_USE_IRQ_PIN
  pinMode(RF_IRQ_PIN, INPUT);
#endif

  if (!radio.begin())
    Serial.println("PRX failed to initialize");
  else {
    // RF24 library begin() function modified to enable PRX mode
    radio.setAddressWidth(5); // set address size to 5 bytes
    radio.setChannel(RF_CHANNEL); // set communication channel
    radio.setPayloadSize(NUM_TTR_BYTES); // set payload size to number of bytes being RECEIVED
    radio.enableAckPayload(); // enable ability to add payload sent with auto ACK
    radio.enableDynamicAck();
    radio.setPALevel(RF24_PA_LOW); // set power amplifier level. Using LOW for tests on bench. Should use HIGH on PL/Truck
    radio.setDataRate(RF24_1MBPS); // set data rate to most reliable speed
    radio.openReadingPipe(0, RF_PRX_READ_ADDR);
    radio.writeAckPayload(0, &rtt, NUM_RTT_BYTES);
    radio.startListening(); // PRX now needs to start listening for packets
    Serial.println("PRX initialization successful");
  }
}

unsigned long curTime = millis();
static unsigned long preLogTime = curTime;
static unsigned long lastReceiveTime = curTime;
static TX_TO_RX ttr;
void loop() {
  curTime = millis();
  IsConnected();

#ifdef RF_USE_IRQ_PIN  
  if (LOW == digitalRead(RF_IRQ_PIN)) {
    bool tx_ok=false, tx_fail=false, rx_ready=false;
    radio.whatHappened(tx_ok, tx_fail, rx_ready);
    if (rx_ready || radio.available()) {
#else 
    if (radio.available()) {
#endif      
      radio.read(&ttr, NUM_TTR_BYTES);
      lastReceiveTime = curTime;
      /** @note in Truck code, will want to update the ack payload every
       * iteration of the main loop */
      // rtt.SwitchStatus++;
      rtt.SolenoidStatus++;
      // rtt.Count++;
      radio.writeAckPayload(0, &rtt, NUM_RTT_BYTES);
    }
#ifdef RF_USE_IRQ_PIN
  }
#endif

  if (curTime - preLogTime >= 1000) {
    LogInfo(F("phase 0x%X, ledControl 0x%X, encoder %ld, isConnected %d\n"),
              ttr.Phase, ttr.LEDControl, ttr.FrontEncoder, IsConnected());
    preLogTime = curTime;
  }
}

bool IsConnected(void) {
  static bool conn = false;
  if (curTime - lastReceiveTime >= 250 && conn) {
    LogInfo("connection to PTX is lost!\n");
    conn = false;
  }
  else if (lastReceiveTime > 0 && curTime - lastReceiveTime < 250 && !conn) {
    LogInfo("established connection to PTX\n");
    conn = true;
  }
  return conn;
}