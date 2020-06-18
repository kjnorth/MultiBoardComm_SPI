#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "..\lib\DataLog\DataLog.h"
#include "..\lib\RoboClaw\RoboClaw.h"

#define ROBOCLAW_ADDRESS 0x80
SoftwareSerial mySerial(3, 2); // RX, TX
RoboClaw rclaw(&mySerial, 3500);
void InitRoboclaw(void);

uint8_t rec, send;
float testFloat;
unsigned long curTime = 0;
static unsigned long preTime = 0;

#define TEST 0
void setup() {
  Serial.begin(9600);
  LogInfo("Right Front Nano software begins\n");
//   InitRoboclaw();
  testFloat = -2.464;
}

void loop() {
  curTime = millis();
  if (curTime - preTime >= 1000) {
    preTime = curTime;
    LogInfo("byte received 0x%X\n", rec);
    testFloat += 0.001;
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