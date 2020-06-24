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
#include "SubDevice.h"
#include "FrontSubDevice.h"

#define TX_BTN_IN 7
// #define COMM_BUS Serial1

void InitRadio(void);
bool IsConnected(void);
void UpdateTruckData(void);
bool isTxBtnPressedEvent(void);

TX_TO_RX ttr;
RF24 radio(RF_CE_PIN, RF_CSN_PIN); // Create a radio object

FrontSubDev leftFront(LEFT_FRONT, LF_SUBDEV_SS_PIN);
FrontSubDev rightFront(RIGHT_FRONT, RF_SUBDEV_SS_PIN);
// RearSubDev leftRear(LEFT_REAR, LR_SUBDEV_SS_PIN);
// RearSubDev rightRear(RIGHT_REAR, RR_SUBDEV_SS_PIN);

void setup() {
  Serial.begin(115200);
  COMM_BUS.begin(115200);
  LogInfo("Master boots up\n");
  pinMode(TX_BTN_IN, INPUT_PULLUP);
  InitRadio();
  // send simple cmds to ALL sub boards here and ensure they respond with SUCCESS
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
      // float data = ReadPitch(RIGHT_REAR);
      // if (data != REC_FLOAT_ERROR_RESPONSE) {
      //   LogInfo("data received ", data, 2, true);
      // }
      /** NOTE: SAVE CODE BELOW ******************
      static bool flip = true;
      subdev_response_t response;
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
      // **********************************************/
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