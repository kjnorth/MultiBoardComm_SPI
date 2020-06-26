/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 13:07:12 
 * @Desc: defines the RearSubDev class
 * derived from SubDev class 
 */

#ifndef REAR_SUB_DEVICE_H_
#define REAR_SUB_DEVICE_H_

#include "SubDevice.h"

class RearSubDev: public SubDev {
  public:
    // **** PUBLIC FUNCTIONS ****
    RearSubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs);
    bool StopM5(void);
    bool ForwardM5(void);
    bool ReverseM5(void);
    bool LatchTR(void);
    bool UnlatchTR(void);
    bool ReadSwitchStatus(void);
    bool IsM5FrontLimitTriggered(void);
    bool IsM5RearLimitTriggered(void);
    bool IsM5DumpPosLimitTriggered(void);
    // **** END PUBLIC FUNCTIONS ****

  private:
    // **** PRIVATE MEMBER VARIABLES ****
    // 0 in bitN means swN is OFF, 1 in bitN means swN is ON
    uint8_t m_swStatus;
    // **** END PRIVATE MEMBER VARIABLES ****

    // **** PRIVATE STRUCTS ****

    // **** END PRIVATE STRUCTS ****

    // **** PRIVATE ENUMS ****
    typedef enum {
      INIT_LR=1, INIT_RR,
      M5_STOP, M5_FORWARD, M5_REVERSE,
      TR_LATCH, TR_UNLATCH,
      GET_SW_STATUS,
    } rear_subdev_cmd_t;
    // **** END PRIVATE ENUMS ****

    // **** PRIVATE FUNCTIONS ****

    // **** END PRIVATE FUNCTIONS ****
};

#endif /* REAR_SUB_DEVICE_H_ */