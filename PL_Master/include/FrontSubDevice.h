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
    FrontSubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs);
    float ReadPitch(void);
    float ReadRoll(void);
    void DisableSolenoids(void);
    void EnableSolenoidA(void);
    void EnableSolenoidC(void);
    void EnableSolenoidE(void);
    bool ReadSolenoidStatus(void);
    bool IsSolenoidAIn(void);
    bool IsSolenoidAOut(void);
    bool IsSolenoidCIn(void);
    bool IsSolenoidCOut(void);
    bool IsSolenoidEIn(void);
    bool IsSolenoidEOut(void);
    // **** END PUBLIC FUNCTIONS ****

  private:
    // **** PRIVATE MEMBER VARIABLES ****
    uint8_t m_solStatusByte;
    uint8_t m_solAStatus; // 0 = OUT, 1 = IN
    uint8_t m_solCStatus; // 0 = OUT, 1 = IN
    uint8_t m_solEStatus; // 0 = OUT, 1 = IN
    // **** END PRIVATE MEMBER VARIABLES ****

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