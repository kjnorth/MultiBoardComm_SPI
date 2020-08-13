#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "..\lib\DataLog\DataLog.h"
#include "..\lib\RoboClaw\RoboClaw.h"

#define SUB_DEV_SS_PIN 2
#define SUB_DEV_TX_PIN 3
#define SUB_DEV_RX_PIN 4
#define SUB_DEV_IRQ_PIN 5

#define ROBOCLAW_ADDRESS 0x80
#define ROBOCLAW_TIMEOUT_US 3500 // microseconds
#define ROBOCLAW_TX_PIN 6
#define ROBOCLAW_RX_PIN 7

// all sub devices receive commands in the format of this typedef
typedef struct {
  uint8_t command;
  uint16_t crc;
} subdev_cmd_packet_t;

// sub devices send pitch and roll data in this form
typedef struct {
  float data;
  uint16_t crc;
} subdev_float_packet_t;

// all sub devices send switch or solenoid status' in this form
typedef struct {
  uint8_t data;
  uint16_t crc;
} subdev_byte_packet_t;
 
// commands sent from master board
typedef enum {
  // front sub dev cmds
  NONE, INIT,
  SOLS_DISABLE, SOLA_ENABLE, SOLC_ENABLE, SOLE_ENABLE,
  LASER_DISABLE, LASER_ENABLE,
  GET_PITCH, GET_ROLL, GET_SOL_STATUS,
  // rear sub dev cmds
  INIT_LR, INIT_RR,
  M5_STOP, M5_FORWARD, M5_REVERSE,
  TR_LATCH, TR_UNLATCH,
  GET_SW_STATUS,
} subdev_cmd_t;

typedef enum {
  UNKNOWN=0xE0, ERROR, CRC_ERROR, CMD_ERROR, SUCCESS, DATA_INCOMING,
} subdev_response_t;

void InitRoboclaw(void);
void ReadMasterCmd(void);
void SendFloat(float data);
uint16_t GetCRC16(unsigned char *buf, int nBytes);

SoftwareSerial rclawSerial(ROBOCLAW_RX_PIN, ROBOCLAW_TX_PIN); // Rx, Tx - roboclaw serial port
RoboClaw rclaw(&rclawSerial, ROBOCLAW_TIMEOUT_US);

SoftwareSerial masterSerial(SUB_DEV_RX_PIN, SUB_DEV_TX_PIN); // Rx, Tx - master board serial bus

float testFloat;
unsigned long curTime = 0;
static unsigned long preTime = 0;

#define TEST 0
void setup() {
  Serial.begin(115200);
  LogInfo("Right Front Nano software begins\r\n");
  InitRoboclaw();
  pinMode(SUB_DEV_SS_PIN, INPUT);
  pinMode(SUB_DEV_IRQ_PIN, OUTPUT);
  digitalWrite(SUB_DEV_IRQ_PIN, LOW);
  masterSerial.begin(115200);
  /** NOTE: Only ONE software serial port can be listening at a time,
   * so must start rclaw port listening when asking for encoders / 
   * motor current etc */
  masterSerial.listen();
}

void loop() {
  curTime = millis();
  ReadMasterCmd();

  if (curTime - preTime >= 1000) {
    preTime = curTime;
    // LogInfo("byte received 0x%X\r\n", rec);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 15:56:09 
 * @Desc: retrieves commands from the master board 
 */
void ReadMasterCmd(void) {
  if (digitalRead(SUB_DEV_SS_PIN) == HIGH) {
    // communication to this device is OPEN
    if (masterSerial.available()) {
      // data from the master is available
      subdev_cmd_packet_t *packet = (subdev_cmd_packet_t*)malloc(sizeof(subdev_cmd_packet_t));
      masterSerial.readBytes((uint8_t *)packet, sizeof(subdev_cmd_packet_t));
      uint16_t crcCalc = GetCRC16((unsigned char *)packet, sizeof(subdev_cmd_packet_t)-2);
      static float data = -3.29;

      if (crcCalc == packet->crc) {
        // got uncorrupted data
        switch (packet->command) {
          case INIT:
            masterSerial.write(SUCCESS);
            break;
          case M5_STOP:
            rclawSerial.listen();
            rclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
            masterSerial.listen();
            masterSerial.write(SUCCESS);
            break;
          case M5_FORWARD:
            rclawSerial.listen();
            rclaw.ForwardM2(ROBOCLAW_ADDRESS, 25);
            masterSerial.listen();
            masterSerial.write(SUCCESS);
            break;
          case M5_REVERSE:
            rclawSerial.listen();
            rclaw.BackwardM2(ROBOCLAW_ADDRESS, 25);
            masterSerial.listen();
            masterSerial.write(SUCCESS);
            break;
          case GET_PITCH:
            masterSerial.write(DATA_INCOMING);
            SendFloat(data++);
            break;
          default:
            masterSerial.write(CMD_ERROR);
            break;
        }
      }
      else {
        // data got corrupted
        masterSerial.write(CRC_ERROR);
      }
      free(packet);
    }
  }
  // else {} // communication to this device is CLOSED
}

void SendFloat(float data) {
  subdev_float_packet_t *floatPacket = (subdev_float_packet_t*)malloc(sizeof(subdev_float_packet_t));
  floatPacket->data = data;
  floatPacket->crc = GetCRC16((unsigned char *)floatPacket, sizeof(subdev_float_packet_t)-2);
  masterSerial.write((unsigned char *)floatPacket, sizeof(subdev_float_packet_t));
  free(floatPacket);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 15:55:08 
 * @Desc: initializes the roboclaw motor controller 
 */
void InitRoboclaw(void) {
  char version[256] = {0};
  rclawSerial.listen();
  rclaw.begin(57600);
  // Note: rclaw Motor1 is the Rear Motor, Motor2 is the Front Motor
	// send motor stop commands to be safe
	rclaw.ForwardM1(ROBOCLAW_ADDRESS,0);
	rclaw.ForwardM2(ROBOCLAW_ADDRESS,0);
	// set maximum current limits to 45 amps
	if (!rclaw.SetM1MaxCurrent(ROBOCLAW_ADDRESS, 4500) ||
		!rclaw.SetM2MaxCurrent(ROBOCLAW_ADDRESS, 4500))
		LogInfo("Error setting RCLAW max currents\r\n");
	uint32_t m1MaxCurrent, m2MaxCurrent;
	if (!rclaw.ReadM1MaxCurrent(ROBOCLAW_ADDRESS, m1MaxCurrent) ||
		!rclaw.ReadM2MaxCurrent(ROBOCLAW_ADDRESS, m2MaxCurrent) ||
		!rclaw.ReadVersion(ROBOCLAW_ADDRESS, version))
		LogInfo("Error reading RCLAW data\r\n");
	else {
		LogInfo(F("RoboClaw max currents, M1 %u [A] "),
							m1MaxCurrent/100);
		LogInfo(F("M2 %u [A], "), m2MaxCurrent/100);
		LogInfo(F("firmware version is %s\r\n"), version);
	}
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