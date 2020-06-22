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

#define COMM_BUS Serial1
// NOTE: if you throw these in an enum, make sure to start it at 1!!!! or greater since cmds start at 1
#define NANO_RF_UADDR 0xE9 // nano right front unique address
#define NANO_RF_UACK 0x5F // nano right front unique acknowledgement
#define ACK_CMD_NOT_HANDLED 0x3D // any sub device will respond with this byte if cmd isn't handled
#define NUM_RETRYS 2
#define TX_TIMEOUT_US 25 // microseconds

// all sub devices receive commands in the format of this typedef
typedef struct {
  uint8_t devAddr;
  uint8_t command;
  uint16_t crc;
} sub_packet_t;
// sub_packet_t *newCmd = (sub_packet_t*)malloc(sizeof(sub_packet_t));
// ConfigCmd(newCmd, NANO_RF_UADDR, M5_FORWARD);
// do stuff
// free(newCmd);

// start cmd at 1 since timeout response is 0
typedef enum {
  SOLS_DISABLE=1, SOLA_ENABLE, SOLC_ENABLE, SOLE_ENABLE,
  LASER_DISABLE, LASER_ENABLE,
  M5_STOP, M5_FORWARD, M5_REVERSE,
  TR_LATCH, TR_UNLATCH,
  GET_PITCH, GET_ROLL, GET_SOL_STATUS, GET_SW_STATUS,
} sub_dev_cmd_t;

void InitRadio(void);
bool IsConnected(void);
void UpdateTruckData(void);
void SendNanoRFData(void);
uint8_t ReadByteOrTimeout(uint8_t timeoutUs); // timeout in microseconds
void ConfigCmd(sub_packet_t *newCmd, uint8_t uAddr, sub_dev_cmd_t cmdByte);
uint16_t GetCRC16(unsigned char *buf, int nBytes);

TX_TO_RX ttr;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

void setup() {
  Serial.begin(115200);
  COMM_BUS.begin(115200);
  LogInfo("Master boots up\n");
  InitRadio();

  sub_packet_t *newCmd = (sub_packet_t*)malloc(sizeof(sub_packet_t));
  ConfigCmd(newCmd, NANO_RF_UADDR, M5_FORWARD);
  // do stuff
  // uint32_t start = micros();
  // uint16_t crc = GetCRC16((unsigned char *)newCmd, 2);
  // uint32_t finis = micros();
  // LogInfo("time to get crc %lu\n", finis-start);

  // will want to copy newCmd in buf like this to send each byte individually
  // unsigned char *buf = (unsigned char *)newCmd;
  // uint16_t crcConstruct = (uint16_t)(*(buf+2) | (*(buf+3)<<8));
  // LogInfo(F("test1 0x%X 0x%X 0x%X\n"), *buf, *(buf+1), crcConstruct);
  free(newCmd);
}

unsigned long curTime = 0;
unsigned long preLogTime = 0;
unsigned long preSendTime = 0;
unsigned long lastReceiveTime = 0;
static RX_TO_TX rtt;
void loop() {
  curTime = millis();

  static bool cmdReceived = false;
  IsConnected();
  if (curTime - preSendTime >= 100) { // write data to truck PRX at 100Hz frequency
    // truck comm
    UpdateTruckData();
    // right front nano comm
    SendNanoRFData();
    preSendTime = curTime;
  }

  if (curTime - preLogTime >= 1000) {
    // LogInfo(F("switchStatus 0x%X, solenoid Status 0x%X, isConnected %d\n"),
    //             rtt.SwitchStatus, rtt.SolenoidStatus, IsConnected());
    preLogTime = curTime;
  }
}

void SendNanoRFData(void) {
  // test vars
  static int count = 0;
  static int addrSuccesses = 0;
  static int cmdSuccesses = 0;
  static int cmdCorrupts = 0;
  static int cmdFails = 0;
  static int addrFails = 0;
  static int addrTrys = 0;
  static int cmdTrys = 0;
  // *********

  uint32_t txStart = micros();
  uint8_t trys = NUM_RETRYS;
  bool ackReceived = false;
  bool ok = false;
  while (trys--) {
    addrTrys++;
    // send sub dev address, wait for ack before sending cmd
    COMM_BUS.write(NANO_RF_UADDR);
    if (ReadByteOrTimeout(TX_TIMEOUT_US) == NANO_RF_UACK) {
      addrSuccesses++;
      ackReceived = true;
      break;
    }
  }
  if (ackReceived) {
    trys = NUM_RETRYS;
    ackReceived = false;
    while (trys--) {
      cmdTrys++;
      // send sub dev command, wait for ack
      COMM_BUS.write(M5_FORWARD);
      uint8_t rec = ReadByteOrTimeout(TX_TIMEOUT_US);
      if (rec == NANO_RF_UACK) {
        // LogInfo("Tx OK\n");
        cmdSuccesses++;
        ok = true;
        ackReceived = true;
        break;
      }
      else if (rec == ACK_CMD_NOT_HANDLED) {
        ackReceived = true;
        cmdCorrupts++;
      }
    }
    if (!ackReceived) {
      cmdFails++;
    }
  }
  else {
    addrFails++;
  }
  count++;
  // if (count % 100 == 0) {
  //   if (ok) {
  //     LogInfo("good tx time us %lu\n", micros()-txStart);
  //   }
  //   LogInfo(F("addrSuccesses %d, cmdSuccesses %d, errors: cmdCorrupts %d, cmdFails %d, addrFails %d, trys: addrTrys %d, cmdTrys %d\n"),
  //             addrSuccesses, cmdSuccesses, cmdCorrupts, cmdFails, addrFails, addrTrys, cmdTrys);
  //   count = 0;
  //   addrSuccesses = 0; cmdSuccesses = 0; cmdCorrupts = 0; cmdFails = 0; addrFails = 0; addrTrys = 0; cmdTrys = 0;
  // }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 09:40:27 
 * @Desc: reads a byte if available in the timeout duration
 * @Param - timeoutUs: the timeout duration in us
 * @Return: byte read, or 0 if timeout
 */
uint8_t ReadByteOrTimeout(uint8_t timeoutUs) {
  uint32_t start = micros();
  while (!COMM_BUS.available()) {
    if (micros() - start >= timeoutUs) {
      return 0;
    }
  }
  return COMM_BUS.read();
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-19 09:31:10 
 * @Desc: sets newCmd member variables and computes crc
 * @Param - newCmd: pointer to command struct
 * @Param - addr: unique address of sub device
 * @Param - cmdByte: byte corresponding to sub device command
 */
void ConfigCmd(sub_packet_t *newCmd, uint8_t uAddr, sub_dev_cmd_t cmdByte) {
  newCmd->devAddr = uAddr;
  newCmd->command = cmdByte;
  newCmd->crc = GetCRC16((unsigned char *)newCmd, sizeof(sub_packet_t)-2);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-19 08:52:33 
 * @Desc: Initializes the NRF24L01 module that communicates with the truck 
 */
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