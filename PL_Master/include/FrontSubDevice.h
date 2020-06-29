/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 15:05:28 
 * @Desc: defines the FrontSubDev class
 * derived from SubDev class
 */

#ifndef FRONT_SUB_DEVICE_H_
#define FRONT_SUB_DEVICE_H_

#include "SubDevice.h"

class FrontSubDev: public SubDev {
  public:
    // **** PUBLIC FUNCTIONS ****
    FrontSubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs);
    bool DisableSolenoids(void);
    bool EnableSolenoidA(void);
    bool EnableSolenoidC(void);
    bool EnableSolenoidE(void);
    bool CommandLaser(bool on_off); // true turns laser ON
    bool ReadPitch(void);
    bool ReadRoll(void);
    float GetPitch(void);
    float GetRoll(void);
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
    // 0 in bitN means solN is OUT, 1 in bitN means solN is IN
    uint8_t m_solStatus;
    float m_pitch;
    float m_roll;
    float m_pitchOffset;
    float m_rollOffset;
    // **** END PRIVATE MEMBER VARIABLES ****

    // **** PRIVATE STRUCTS ****
    // sub devices send pitch and roll data in this form
    typedef struct {
      float data;
      uint16_t crc;
    } subdev_float_packet_t;
    // **** END PRIVATE STRUCTS ****

    // **** PRIVATE ENUMS ****
    typedef enum {
      NONE, INIT,
      SOLS_DISABLE, SOLA_ENABLE, SOLC_ENABLE, SOLE_ENABLE,
      LASER_DISABLE, LASER_ENABLE,
      GET_PITCH, GET_ROLL, GET_SOL_STATUS,
    } front_subdev_cmd_t;
    // **** END PRIVATE ENUMS ****

    // **** PRIVATE FUNCTIONS ****

    // **** END PRIVATE FUNCTIONS ****
};

#endif /* FRONT_SUB_DEVICE_H_ */