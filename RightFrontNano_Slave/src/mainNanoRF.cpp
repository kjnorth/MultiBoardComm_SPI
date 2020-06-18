#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "..\lib\DataLog\DataLog.h"
#include "..\lib\RoboClaw\RoboClaw.h"

#define ROBOCLAW_ADDRESS 0x80
SoftwareSerial mySerial(3, 2); // RX, TX
RoboClaw rclaw(&mySerial, 3500);
void InitRoboclaw(void);

volatile uint8_t rec, send; // volatile since accessed in interrupt
volatile float testFloat;
unsigned long curTime = 0;
static unsigned long preTime = 0;

static int32_t temp = 0.0;
static float fres = 0.0;
#define TEST 0
void setup() {
  Serial.begin(9600);
  LogInfo("Right Front Nano software begins\n");
//   InitRoboclaw();

  pinMode(MISO, OUTPUT); // MISO is an output in slave mode
  SPCR |= _BV(SPE); // turn on SPI slave mode
  SPI.attachInterrupt();

  send = 0x00;
  testFloat = -2.464;
#if TEST
  float num_m = testFloat;
  Serial.print("start falt_m: ");
  Serial.println(num_m, 3);
  int32_t temp_mm = (int32_t)(num_m*1000);
  uint8_t b0 = (uint8_t)((temp_mm & 0x000000FF) >> 0);
  uint8_t b1 = (uint8_t)((temp_mm & 0x0000FF00) >> 8);
  uint8_t b2 = (uint8_t)((temp_mm & 0x00FF0000) >> 16);
  uint8_t b3 = (uint8_t)((temp_mm & 0xFF000000) >> 24);
  int32_t alt_mm = 0;
  alt_mm |= (uint32_t)b3 << 24;
  alt_mm |= (uint32_t)b2 << 16;
  alt_mm |= (uint32_t)b1 << 8;
  alt_mm |= (uint32_t)b0 << 0;
  float falt_m = (float)(alt_mm/1000.0);
  Serial.print("deconstruct, reconstruct falt_m: ");
  Serial.println(falt_m, 3);
#endif
}

ISR (SPI_STC_vect) {
  static uint8_t b0,b1,b2,b3;
  static int32_t ires = 0;
  rec = SPDR;
  switch (rec) {
    case 0xA0:
      temp = (int32_t)(testFloat*1000);
      b0=0; b1=0; b2=0; b3=0; ires = 0;
      send = (uint8_t)((temp & 0x000000FF) >> 0); // sets float byte 0
      b0 = send;
      break;
    case 0xA1:
      // sends float byte 0
      send = (uint8_t)((temp & 0x0000FF00) >> 8); // sets float byte 1
      b1 = send;
      break;
    case 0xA2:
      // sends float byte 1
      send = (uint8_t)((temp & 0x00FF0000) >> 16); // sets float byte 2
      b2 = send;
      break;
    case 0xA3:
      // sends float byte 2
      send = (uint8_t)((temp & 0xFF000000) >> 24); // sets float byte 3
      b3 = send;
      break;
    case 0xA4:
      // sends float byte 3
      send = 0; // could also set float byte 0 of roll data here!!
      // ires |= (uint32_t)b3 << 24;
      // ires |= (uint32_t)b2 << 16;
      // ires |= (uint32_t)b1 << 8;
      // ires |= (uint32_t)b0 << 0;
      // LogInfo(F("b0 0x%X b1 0x%X b2 0x%X b3 0x%X\n"), b0,b1,b2,b3);
      // fres = (float)(ires/1000.0);
      break;
    default:
      send = 0x99;
      break;
  }
  SPDR = send;
}

void loop() {
  curTime = millis();
  if (curTime - preTime >= 2000) {
    // LogInfo("float expected ", (float)(temp/1000.0), 3);
    // LogInfo(" float reconstructed ", fres, 3, true);
    preTime = curTime;

    /** NOTE: need the noInterrupts / interrupts whenever updating
     * pitch / roll data! */
    noInterrupts();
    testFloat += 0.001;
    interrupts();
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