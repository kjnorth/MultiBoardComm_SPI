/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 15:05:28 
 * @Desc: defines the FrontSubDev class
 * derived from SubDev class
 */

// do i need to include "SubDevice.h" ???
#include "SubDevice.h"

#ifndef FRONT_SUB_DEVICE_H_
#define FRONT_SUB_DEVICE_H_

class FrontSubDev: public SubDev {
  public:
    // **** PUBLIC FUNCTIONS ****
    using SubDev::SubDev; // inherit constructor from base class
    float ReadPitch(void);
    float ReadRoll(void);
    uint8_t ReadSolenoidStatus(void);
    // **** END PUBLIC FUNCTIONS ****

  private:
    // **** PRIVATE STRUCTS ****
    // sub devices send pitch and roll data in this form
    typedef struct {
      float data;
      uint16_t crc;
    } subdev_float_packet_t;
    // **** END PRIVATE STRUCTS ****

    // **** PRIVATE FUNCTIONS ****

    // **** END PRIVATE FUNCTIONS ****
};

#endif /* FRONT_SUB_DEVICE_H_ */