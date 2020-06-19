#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "..\lib\DataLog\DataLog.h"
#include "..\lib\RoboClaw\RoboClaw.h"

#define NANO_RF_UADDR 0xE9 // nano right front unique address
#define NANO_RF_UACK 0x5F // nano right front unique acknowledgement
#define ACK_CMD_NOT_HANDLED 0x3D // any sub device will respond with this byte if cmd isn't handled
#define ACK_CRC_NOT_MATCHED 0x4E

// all sub devices receive commands in the format of this typedef
typedef struct {
  uint8_t devAddr;
  uint8_t command;
  uint16_t crc;
} sub_packet_t;

// commands sent from master board
typedef enum {
  SOLS_DISABLE, SOLA_ENABLE, SOLC_ENABLE, SOLE_ENABLE,
  LASER_DISABLE, LASER_ENABLE,
  M5_STOP, M5_FORWARD, M5_REVERSE,
  TR_LATCH, TR_UNLATCH,
  GET_PITCH, GET_ROLL, GET_SOL_STATUS, GET_SW_STATUS,
} sub_dev_cmd_t;

void InitRoboclaw(void);
void RecMasterData(void);
uint16_t GetCRC16(unsigned char *buf, int nBytes);

#define ROBOCLAW_ADDRESS 0x80
SoftwareSerial rclawSerial(5, 4); // Rx, Tx - roboclaw serial port
RoboClaw rclaw(&rclawSerial, 3500); // note: timeout is in microseconds

SoftwareSerial masterSerial(3, 2); // Rx, Tx - master board serial port

uint8_t rec, send;
float testFloat;
unsigned long curTime = 0;
static unsigned long preTime = 0;

#define TEST 0
void setup() {
  Serial.begin(9600);
  masterSerial.begin(115200);
  LogInfo("Right Front Nano software begins\n");
//   InitRoboclaw();
  testFloat = -2.464;
}

void loop() {
  curTime = millis();
  RecMasterData();
  // if (masterSerial.available()) {
  //   sub_packet_t *recCmd = (sub_packet_t*)malloc(sizeof(sub_packet_t));
  //   masterSerial.readBytes((uint8_t *)recCmd, sizeof(sub_packet_t));
  //   LogInfo(F("addr 0x%X, cmd 0x%X, crc 0x%X\n"), recCmd->devAddr, recCmd->command, recCmd->crc);
  //   free(recCmd);
  // }
  // else {
  //   // LogInfo("no data available\n");
  // }

  // delay(100);

  if (curTime - preTime >= 1000) {
    preTime = curTime;
    // LogInfo("byte received 0x%X\n", rec);
    testFloat += 0.001;
  }
}

void RecMasterData(void) {
  if (masterSerial.available()) {
    sub_packet_t *recCmd = (sub_packet_t*)malloc(sizeof(sub_packet_t));
    masterSerial.readBytes((uint8_t *)recCmd, sizeof(sub_packet_t));
    uint16_t crcCalc = GetCRC16((unsigned char *)recCmd, 2);
    if (crcCalc == recCmd->crc) {
      // got uncorrupted data
      if (recCmd->devAddr == NANO_RF_UADDR) {
        // cmd packet sent to this device
        switch (recCmd->command) {
          case M5_STOP:
            // rclaw.ForwardM2(ROBOCLAW_ADDRESS, 0);
            masterSerial.write(NANO_RF_UACK);
            break;
          case M5_FORWARD:
            // rclaw.ForwardM2(ROBOCLAW_ADDRESS, 25);
            masterSerial.write(NANO_RF_UACK);
            break;
          case M5_REVERSE:
            // rclaw.BackwardM2(ROBOCLAW_ADDRESS, 25);
            masterSerial.write(NANO_RF_UACK);
            break;
          default:
            masterSerial.write(ACK_CMD_NOT_HANDLED);
            break;
        }
      }
      else {
          LogInfo("wrong dev addr\n");
      }
    }
    else {
      // LogInfo("crc not match\n");
      masterSerial.write(ACK_CRC_NOT_MATCHED);
    }
    free(recCmd);
  }
}

void InitRoboclaw(void) {
  char version[256] = {0};
  rclaw.begin(57600);
  // Note: rclaw Motor1 is the Rear Motor, Motor2 is the Front Motor
	// send motor stop commands to be safe
	rclaw.ForwardM1(ROBOCLAW_ADDRESS,0);
	rclaw.ForwardM2(ROBOCLAW_ADDRESS,0);
	// set maximum current limits to 45 amps
	if (!rclaw.SetM1MaxCurrent(ROBOCLAW_ADDRESS, 4500) ||
		!rclaw.SetM2MaxCurrent(ROBOCLAW_ADDRESS, 4500))
		LogInfo("Error setting RCLAW max currents\n");
	uint32_t m1MaxCurrent, m2MaxCurrent;
	if (!rclaw.ReadM1MaxCurrent(ROBOCLAW_ADDRESS, m1MaxCurrent) ||
		!rclaw.ReadM2MaxCurrent(ROBOCLAW_ADDRESS, m2MaxCurrent) ||
		!rclaw.ReadVersion(ROBOCLAW_ADDRESS, version))
		LogInfo("Error reading RCLAW data\n");
	else {
		LogInfo(F("RoboClaw max currents, M1 %u [A] "),
							m1MaxCurrent/100);
		LogInfo(F("M2 %u [A], "), m2MaxCurrent/100);
		LogInfo(F("firmware version is %s\n"), version);
		rclaw.SetM1DefaultAccel(ROBOCLAW_ADDRESS, 0xF);
		rclaw.SetM2DefaultAccel(ROBOCLAW_ADDRESS, 0xF);
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