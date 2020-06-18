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

#define NANO_RF Serial1
#define NANO_RF_UADDR 0xE9 // nano right front unique address
#define NANO_RF_UACK 0x5F // nano right front unique acknowledgement

#define M5_STOP_CMD 0xF0
#define M5_FORWARD_CMD 0xF1
#define M5_REVERSE_CMD 0xF2

void InitRadio(void);
bool IsConnected(void);
void UpdateTruckData(void);
uint16_t GetCRC16(unsigned char *buf, int nBytes);

TX_TO_RX ttr;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

void setup() {
  Serial.begin(115200);
  NANO_RF.begin(115200);
  LogInfo("Master boots up\n");
  
  InitRadio();
}

unsigned long curTime = 0;
unsigned long preLogTime = 0;
unsigned long preSendTime = 0;
unsigned long lastReceiveTime = 0;
static RX_TO_TX rtt;
void loop() {
  curTime = millis();

  static uint8_t send = 0xA0;
  static uint8_t rec = 0x00;
  IsConnected();
  if (curTime - preSendTime >= 10) { // write data to truck PRX at 100Hz frequency
    // truck comm
    UpdateTruckData();
    // right front nano comm
    if (NANO_RF.availableForWrite()) {
      // write 1 byte ADDR, 1 byte CMD
      unsigned char buf[3];
      buf[0] = NANO_RF_UADDR; buf[1] = M5_FORWARD_CMD;
      uint16_t crc = GetCRC16(buf, 2);
      buf[3] = crc;
      NANO_RF.write(buf, 3);
      unsigned long t = micros();
      while (!NANO_RF.available()) {};
      uint8_t recByte = NANO_RF.read();
      if (recByte == NANO_RF_UACK) {
        LogInfo("transmission OK - respond time %lu\n", micros()-t);
      }
      else {
        LogInfo("ERROR - incorrect byte received!\n");
      }

    }
    preSendTime = curTime;
  }

  int32_t ires = 0;
  float receivedFloat = (float)(ires/1000.0);
  if (curTime - preLogTime >= 1000) {
    LogInfo(F("switchStatus 0x%X, solenoid Status 0x%X, isConnected %d, "),
                rtt.SwitchStatus, rtt.SolenoidStatus, IsConnected(), rec);
    LogInfo("received float ", receivedFloat, 3, true);
    preLogTime = curTime;
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

// Calculates CRC16 of nBytes of data in byte array message
uint16_t GetCRC16(unsigned char *buf, int nBytes) {
	uint16_t crc = 0;
	for (int byte = 0; byte < nBytes; byte++) {
		crc = crc ^ ((unsigned int)buf[byte] << 8);
		for (unsigned char bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}
 	return crc;
}