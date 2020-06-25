/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 14:16:39 
 * @Desc: defines the sub device base class 
 */

#ifndef SUB_DEVICE_H_
#define SUB_DEVICE_H_

#include <Arduino.h>
#include "ConfigPTX.h"
#include "DataLog.h"

#define COMM_BUS Serial1

// sub device unique identifier used when creating objects
typedef enum {
  LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR
} subdev_id_t;

class SubDev {
  public:
    // **** PUBLIC FUNCTIONS ****
    SubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs);
    // **** END PUBLIC FUNCTIONS ****

  protected:
    // **** PROTECTED MEMBER VARIABLES ****
    subdev_id_t m_id;
    uint8_t m_ssPin;
    uint16_t m_timeout; // this is in microseconds
    // **** END PROTECTED MEMBER VARIABLES ****

    // **** PROTECTED STRUCTS ****
    // all sub devices send switch or solenoid statuses in this form
    typedef struct {
      uint8_t data;
      uint16_t crc;
    } subdev_byte_packet_t;
    // **** END PROTECTED STRUCTS ****

    // **** PROTECTED ENUMS ****
    // all sub devices respond to commands with these bytes
    typedef enum {
      ERROR=0xE0, CRC_ERROR, CMD_ERROR, SUCCESS, DATA_INCOMING,
    } subdev_response_t;
    // TODO: this enum should really be split into two, and placed in the proper
    // derived class. I don't think it will matter if cmd values overlap once
    // you do this because it's not possible for rear subdev to write front cmd
    // since it wont exist in the scope of that class
    // commands for front and rear sub devices
    typedef enum {
      // front sub dev cmds
      INIT_LF=1, INIT_RF,
      SOLS_DISABLE, SOLA_ENABLE, SOLC_ENABLE, SOLE_ENABLE,
      LASER_DISABLE, LASER_ENABLE,
      GET_PITCH, GET_ROLL, GET_SOL_STATUS,
      // rear sub dev cmds
      INIT_LR, INIT_RR,
      M5_STOP, M5_FORWARD, M5_REVERSE,
      TR_LATCH, TR_UNLATCH,
      GET_SW_STATUS,
    } subdev_cmd_t;
    // **** END PROTECTED ENUMS ****

    // **** PROTECTED FUNCTIONS ****
    subdev_response_t WriteCmd(subdev_cmd_t cmd);
    bool RecDataOrTimeout(void);
    uint16_t GetCRC16(unsigned char *buf, int nBytes);
    // **** END PROTECTED FUNCTIONS ****

  private:
    // **** PRIVATE STRUCTS ****
    // all sub devices receive commands in the format of this struct
    typedef struct {
      uint8_t command;
      uint16_t crc;
    } subdev_cmd_packet_t;
    // **** END PRIVATE STRUCTS ****

    // **** PRIVATE FUNCTIONS ****
    subdev_response_t ReadByteOrTimeout(void);
    void AssertSSLine(void);
    void ClearSSLine(void);
    void ConfigPacket(subdev_cmd_packet_t *newCmd, subdev_cmd_t cmdByte);
    // **** END PRIVATE FUNCTIONS ****
};

#endif /* SUB_DEVICE_H_ */