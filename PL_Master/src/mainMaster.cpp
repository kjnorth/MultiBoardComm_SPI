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

#define LF_SUB_SS_PIN 2
#define RF_SUB_SS_PIN 3
#define LR_SUB_SS_PIN 4
#define RR_SUB_SS_PIN 5
#define TX_BTN_IN 7
#define COMM_BUS Serial1
#define ACK_SUCCESS 0x5F // universal acknowledgement from sub devices
#define ACK_CMD_NOT_HANDLED 0x83 // universal ack if command isn't handled
#define ACK_CRC_NOT_MATCHED 0xD7 // universal ack if CRC doesn't match
#define NUM_RETRYS 3
#define TIMEOUT_US_FRONT_SUB 500 // microseconds
#define TIMEOUT_US_REAR_SUB 2000 // microseconds // longer for rear devs since they communicate with rclaws

typedef enum {
  LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR
} sub_dev_t;

// all sub devices receive commands in the format of this typedef
typedef struct {
  uint8_t command;
  uint16_t crc;
} sub_dev_packet_t;

// start cmd at 1 since timeout response is 0
typedef enum {
  SOLS_DISABLE=1, SOLA_ENABLE, SOLC_ENABLE, SOLE_ENABLE,
  LASER_DISABLE, LASER_ENABLE,
  M5_STOP, M5_FORWARD, M5_REVERSE,
  TR_LATCH, TR_UNLATCH,
  GET_PITCH, GET_ROLL, GET_SOL_STATUS, GET_SW_STATUS,
} sub_dev_cmd_t;

typedef enum {
  ERROR=0xE0, CRC_ERROR, CMD_ERROR, SUCCESS,
} sub_dev_response_t;

void InitRadio(void);
bool IsConnected(void);
void UpdateTruckData(void);
sub_dev_response_t WriteCmd(sub_dev_t device, sub_dev_cmd_t cmd);
uint16_t AssertSSLine_GetTimeout(sub_dev_t device);
void ClearSSLine(sub_dev_t device);
sub_dev_response_t ReadByteOrTimeout(uint16_t timeoutUs); // timeout in microseconds
void ConfigPacket(sub_dev_packet_t *newCmd, sub_dev_cmd_t cmdByte);
uint16_t GetCRC16(unsigned char *buf, int nBytes);
bool isTxBtnPressedEvent(void);

TX_TO_RX ttr;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

void setup() {
  Serial.begin(115200);
  COMM_BUS.begin(115200);
  LogInfo("Master boots up\n");
  pinMode(LF_SUB_SS_PIN, OUTPUT);
  pinMode(RF_SUB_SS_PIN, OUTPUT);
  pinMode(LR_SUB_SS_PIN, OUTPUT);
  pinMode(RR_SUB_SS_PIN, OUTPUT);
  digitalWrite(LF_SUB_SS_PIN, LOW);
  digitalWrite(RF_SUB_SS_PIN, LOW);
  digitalWrite(LR_SUB_SS_PIN, LOW);
  digitalWrite(RR_SUB_SS_PIN, LOW);
  pinMode(TX_BTN_IN, INPUT_PULLUP);
  InitRadio();
}

unsigned long curTime = 0;
unsigned long preLogTime = 0;
unsigned long preSendTime = 0;
unsigned long lastReceiveTime = 0;
static RX_TO_TX rtt;
void loop() {
  curTime = millis();

  IsConnected();
  if (curTime - preSendTime >= 100) { // write data to truck PRX at 100Hz frequency
    // truck comm
    // UpdateTruckData();
    // right front nano comm
    if (isTxBtnPressedEvent()) { // ensure function is called no faster than 200 Hz
      static bool flip = true;
      sub_dev_response_t response;
      if (flip) {
        response = WriteCmd(RIGHT_REAR, M5_FORWARD);
      }
      else {
        response = WriteCmd(RIGHT_REAR, M5_STOP);
      }
      flip = !flip;
      switch (response) {
        case ERROR:
          LogInfo("Tx ERROR\n");
          break;
        case SUCCESS:
          LogInfo("Tx SUCCESS\n");
          break;
        case CMD_ERROR:
          LogInfo("Tx CMD ERROR\n");
          break;
        case CRC_ERROR:
          LogInfo("Tx CRC ERROR\n");
          break;
        default:
          LogInfo("unhandled response received 0x%X\n", response);
          break;
      }
    }
    preSendTime = curTime;
  }

  if (curTime - preLogTime >= 1000) {
    // LogInfo(F("switchStatus 0x%X, solenoid Status 0x%X, isConnected %d\n"),
    //             rtt.SwitchStatus, rtt.SolenoidStatus, IsConnected());
    preLogTime = curTime;
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 11:34:59 
 * @Desc: writes a command to a sub device
 * @Param - device: device to communicate with
 * @Param - cmd: the command being sent
 */
sub_dev_response_t WriteCmd(sub_dev_t device, sub_dev_cmd_t cmd) {
  sub_dev_packet_t *packet = (sub_dev_packet_t*)malloc(sizeof(sub_dev_packet_t));
  ConfigPacket(packet, cmd);
  uint8_t trys = NUM_RETRYS;
  uint16_t timeout = AssertSSLine_GetTimeout(device);
  sub_dev_response_t response = ERROR; // default to no response error
  // if no response, or CRC doesn't match, keep trying
  while (trys-- && (response <= CRC_ERROR)) {
    COMM_BUS.write((unsigned char *)packet, sizeof(sub_dev_packet_t));
    response = ReadByteOrTimeout(timeout);
  }
  free(packet);
  ClearSSLine(device);
  return response;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 09:40:27 
 * @Desc: reads a byte if available in the timeout duration
 * @Param - timeoutUs: the timeout duration in us
 * @Return: byte read, or 0 if timeout
 */
sub_dev_response_t ReadByteOrTimeout(uint16_t timeoutUs) {
  uint32_t start = micros();
  while (!COMM_BUS.available()) {
    if (micros() - start >= timeoutUs) {
      return ERROR;
    }
  }
  return (sub_dev_response_t)(COMM_BUS.read());
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 10:37:09 
 * @Desc: asserts the SS line for the device passed and
 * returns device-specific timeout value
 * @Param - device: the sub device to communicate with
 * @Return: the timeout for the specific device
 * @Note: timeouts are longer for rear sub devices since
 * they communicate with roboclaws
 */
uint16_t AssertSSLine_GetTimeout(sub_dev_t device) {
  uint16_t timeout = 0;
  switch (device) {
    case LEFT_FRONT:
      digitalWrite(LF_SUB_SS_PIN, HIGH);
      timeout = TIMEOUT_US_FRONT_SUB;
      break;
    case RIGHT_FRONT:
      digitalWrite(RF_SUB_SS_PIN, HIGH);
      timeout = TIMEOUT_US_FRONT_SUB;
      break;
    case LEFT_REAR:
      digitalWrite(LR_SUB_SS_PIN, HIGH);
      timeout = TIMEOUT_US_REAR_SUB;
      break;
    case RIGHT_REAR:
      digitalWrite(RR_SUB_SS_PIN, HIGH);
      timeout = TIMEOUT_US_REAR_SUB;
      break;
    default:
      LogInfo("AssertSSLine_GetTimeout - ERROR: device passed not handled\n");
      break;
  }
  return timeout;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 10:51:53 
 * @Desc: clears the SS line for the device passed
 * @Param - device: the sub device to halt communications with
 */
void ClearSSLine(sub_dev_t device) {
  switch (device) {
    case LEFT_FRONT:
      digitalWrite(LF_SUB_SS_PIN, LOW);
      break;
    case RIGHT_FRONT:
      digitalWrite(RF_SUB_SS_PIN, LOW);
      break;
    case LEFT_REAR:
      digitalWrite(LR_SUB_SS_PIN, LOW);
      break;
    case RIGHT_REAR:
      digitalWrite(RR_SUB_SS_PIN, LOW);
      break;
    default:
      LogInfo("ClearSSLine - ERROR: device passed not handled\n");
      break;
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-19 09:31:10 
 * @Desc: sets newCmd member variables and computes crc
 * @Param - newCmd: pointer to command struct
 * @Param - addr: unique address of sub device
 * @Param - cmdByte: byte corresponding to sub device command
 */
void ConfigPacket(sub_dev_packet_t *newCmd, sub_dev_cmd_t cmdByte) {
  newCmd->command = cmdByte;
  newCmd->crc = GetCRC16((unsigned char *)newCmd, sizeof(sub_dev_packet_t)-2);
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

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 15:59:12 
 * @Desc: checks if the PTX is connected to the PRX
 * @Return: true if connected, false if not 
 */
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

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 16:00:03 
 * @Desc: sends/reads data to/from the truck 
 */
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

bool isTxBtnPressedEvent(void) {
  uint8_t status = 0;
  static uint8_t preStatus = 0;
  uint8_t event = 0;
  static uint8_t preEvent = 0;
  bool returnVal = false;

  status = !digitalRead(TX_BTN_IN); // negate since low side switch
  if (status & preStatus) { // btn pressed for 2 cycles
    event = 1;
  }
  else if (~status & ~preStatus) { // btn released for 2 cycles
    event = 0;
  }
  preStatus = status;

  if (event != preEvent) {
    if (event == 1) // only return true on event when btn is pressed
      returnVal = true;
    preEvent = event;
  }
  else {
    returnVal = false;
  }

  return returnVal;
}