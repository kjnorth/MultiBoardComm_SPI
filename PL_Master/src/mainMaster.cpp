/** @author Kodiak North
 * @date 05/21/2020
 * test project with NRF24l01 module in PTX mode, to 
 * communicate with another NRF24l01 module in PRX mode
 */

#include <Arduino.h>
#include <util/atomic.h>
#include "ConfigPTX.h"
#include "..\lib\DataLog\DataLog.h"
#include "TimerConfig.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "SubDevice.h"
#include "FrontSubDevice.h"
#include "RearSubDevice.h"

#define TX_BTN_IN 23

void InitRadio(void);
void IsConnected(void);
void UpdateTruckData(void);
bool isTxBtnPressedEvent(void);
void InitTimer1ISR(unsigned int freqHz);

TX_TO_RX ttr;
RX_TO_TX rtt;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

FrontSubDev leftFront(LEFT_FRONT, LF_SUBDEV_SS_PIN, F_SUBDEV_TIMEOUT_US);
FrontSubDev rightFront(RIGHT_FRONT, RF_SUBDEV_SS_PIN, F_SUBDEV_TIMEOUT_US);
RearSubDev leftRear(LEFT_REAR, LR_SUBDEV_SS_PIN, R_SUBDEV_TIMEOUT_US);
RearSubDev rightRear(RIGHT_REAR, RR_SUBDEV_SS_PIN, R_SUBDEV_TIMEOUT_US);

void setup() {
  Serial.begin(115200);
  LogInfo("Master boots up\n");
  COMM_BUS.begin(115200);
  pinMode(TX_BTN_IN, INPUT_PULLUP);
  InitRadio();
  TimerConfig::InitTimer1ISRs(100, 300, 1200);
  TimerConfig::StartTimer1ISRs((A | B | C));
}

volatile uint16_t isrCount1A = 0;
volatile uint16_t isrCount1B = 0;
volatile uint16_t isrCount1C = 0;

ISR(TIMER1_COMPA_vect) {
  // UpdateTruckData();
  isrCount1A++;
  // OCR1A += regA;
  OCR1A += timer1.aPeriod;
}

ISR(TIMER1_COMPB_vect) {
  isrCount1B++;
  // OCR1B += regB;
  OCR1B += timer1.bPeriod;
}

ISR(TIMER1_COMPC_vect) {
  isrCount1C++;
  // OCR1C += regC;
  OCR1C += timer1.cPeriod;
}

unsigned long curTime = 0;
unsigned long preLogTime = 0;
unsigned long preSendTime = 0;
volatile bool truckConnStatus = false;
void loop() {
  curTime = millis();

  IsConnected();
  if (curTime - preSendTime >= 10) { // run loop at 100Hz frequency
    preSendTime = curTime;
    // right front nano comm
    if (isTxBtnPressedEvent()) { // ensure function is called no faster than 200 Hz
      // if (rightFront.ReadAttitude()) {
      //   LogInfo("data: pitch ", rightFront.GetPitch(), 2);
      //   LogInfo(", roll ", rightFront.GetRoll(), 2, true);
      // }
      //** NOTE: SAVE CODE BELOW ******************
      static bool flip = true;
      SubDev::subdev_response_t response;
      if (flip) {
        // NOTE: code was written on right front board to test m5 run/stop cmds. Then
        // I abstracted things to rightRear class before buidling a whole new project,
        // so now to run the test I write a rightRear cmd to a rightFront board.
        // response = rightFront.WriteCmd(rightFront.M5_FORWARD);
        // if (rightFront.ReadAttitude()) {
        //   LogInfo("from right front: pitch ", rightFront.GetPitch(), 2);
        //   LogInfo(", roll ", rightFront.GetRoll(), 2, true);
        // }
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          // TIMSK1 &= ~(1 << OCIE1B);
          TimerConfig::StopTimer1ISRs((A | B));
        }
      }
      else {
        // response = rightFront.WriteCmd(rightFront.M5_STOP);
        // leftFront.ReadAttitude();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          // OCR1B = TCNT1 + regB;
          // TIMSK1 |= (1 << OCIE1B);
          TimerConfig::StartTimer1ISRs((A | B));
        }
      }
      flip = !flip;
      // **********************************************/
    }
  }

  if (curTime - preLogTime >= 1000) {
    // LogInfo("pitch ", rtt.Pitch, 2);
    // LogInfo(", roll ", rtt.Roll, 2);
    // LogInfo(F(", switchStatus 0x%X, solenoid Status 0x%X, isConnected %d\n"),
    //             rtt.SwitchStatus, rtt.SolenoidStatus, truckConnStatus);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      LogInfo(F("timer counts: a %u, b %u, c %u\n"), isrCount1A, isrCount1B, isrCount1C);
      isrCount1A = 0; isrCount1B = 0; isrCount1C = 0;
    }
    preLogTime = curTime;
  }
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

  if (!radio.begin()) {
    TRUCK_DEBUG("PTX failed to initialize\n");
  }
  else {
    // RF24 library begin() function enables PTX mode
    radio.setAddressWidth(5); // set address size to 5 bytes
    /** NOTE: as TRUCK_TO_MAIN packet is increased, will have to increase
		 * the delay time between retries. Right now, 500us (1) is fine. Review
     * datasheet when increasing packet size further */
		radio.setRetries(1, 5); // set 5 retries with 500us delays in between. Increase to 10 if lots of disconnects on production board
    radio.setChannel(RF_CHANNEL); // set communication channel
    radio.enableAckPayload(); // enable payload attached to ACK from PRX
    radio.enableDynamicPayloads(); // must be enabled to receive ACK payload correctly
    radio.setPALevel(RF24_PA_HIGH); // set power amplifier level. Using LOW for tests on bench. Should use HIGH on PL/Truck
    radio.setDataRate(RF24_1MBPS); // set data rate to most reliable speed
    radio.openWritingPipe(RF_PTX_WRITE_ADDR); // open the writing pipe on the address we chose
    TRUCK_DEBUG("PTX initialization successful\n");
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 15:59:12 
 * @Desc: checks if the PTX is connected to the PRX
 * @Return: true if connected, false if not 
 */
void IsConnected(void) {
  static bool preConn = false;
  if (!truckConnStatus && preConn) {
    TRUCK_DEBUG("connection to PRX is lost!\n");
  }
  else if (truckConnStatus && !preConn) {
    TRUCK_DEBUG("established connection to PRX\n");
  }
  preConn = truckConnStatus;
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
        truckConnStatus = true;
      }
#ifdef RF_USE_IRQ_PIN      
    }
  }
  else {
    truckConnStatus = false;
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