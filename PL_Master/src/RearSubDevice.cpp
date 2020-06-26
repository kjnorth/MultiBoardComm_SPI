/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 14:54:25 
 * @Desc: implements the RearSubDev class
 * derived from SubDev class 
 */

#include "RearSubDevice.h"

// **** PRIVATE DEFINES ****
#define M5_FRONT_LIMIT_MASK   0x01
#define M5_REAR_LIMIT_MASK    0x02
#define M5_DUMP_POS_MASK      0x04
// **** END PRIVATE DEFINES ****

// **** PUBLIC FUNCTIONS ****
RearSubDev::RearSubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs)
  : SubDev{id, ssPin, responseTimeoutUs} {
  m_swStatus = 0;
  subdev_response_t response;
  switch (id) {
    case LEFT_REAR:
      response = WriteCmd(INIT_LR);
      break;
    case RIGHT_REAR:
      response = WriteCmd(INIT_RR);
      break;
    default:
      LogInfo("RearSubDev - ERROR id %d is not a REAR device", id);
      break;
  }
  if (response != SUCCESS) {
    LogInfo("RearSubDev - ERROR initialization of device %d failed\n", id);
  }
  else {
    LogInfo("RearSubDev - SUCCESS initialization of device %d succeeded\n", id);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:08:19 
 * @Desc: stops motor 5
 * @Return: true if cmd successful, false if failure
 */
bool RearSubDev::StopM5(void) {
  subdev_response_t response = WriteCmd(M5_STOP);
  if (response != SUCCESS) {
    LogInfo("StopM5 - ERROR stopping M5 on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:08:26 
 * @Desc: runs motor 5 forward
 * @Return: true if cmd successful, false if failure 
 */
bool RearSubDev::ForwardM5(void) {
  subdev_response_t response = WriteCmd(M5_FORWARD);
  if (response != SUCCESS) {
    LogInfo("ForwardM5 - ERROR running M5 forward on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:14:08 
 * @Desc: runs motor 5 reverse
 * @Return: true if cmd successful, false if failure 
 */
bool RearSubDev::ReverseM5(void) {
  subdev_response_t response = WriteCmd(M5_REVERSE);
  if (response != SUCCESS) {
    LogInfo("ReverseM5 - ERROR running M5 reverse on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:16:14 
 * @Desc: commands rear sub dev to
 * latch the transfer rails
 * @Return: true if cmd successful, false if failure 
 */
bool RearSubDev::LatchTR(void) {
  subdev_response_t response = WriteCmd(TR_LATCH);
  if (response != SUCCESS) {
    LogInfo("LatchTR - ERROR commanding TR latch on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:17:43 
 * @Desc: commands rear sub dev to
 * unlatch the transfer rails
 * @Return: true if cmd successful, false if failure 
 */
bool RearSubDev::UnlatchTR(void) {
  subdev_response_t response = WriteCmd(TR_UNLATCH);
  if (response != SUCCESS) {
    LogInfo("LatchTR - ERROR commanding TR unlatch on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:19:27 
 * @Desc: requests switch data from rear sub device
 * @Return: true if status read successfully, false otherwise 
 */
bool RearSubDev::ReadSwitchStatus(void) {
  subdev_response_t response = WriteCmd(GET_SW_STATUS);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadSwitchStatus - ERROR: sub dev %d not accepting switch status request, response 0x%X\n"), m_id, response);
    return false;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadSwitchStatus - ERROR: sub dev %d didn't send data after responding to request cmd\n", m_id);
    return false;
  }

  subdev_byte_packet_t *bytePacket = (subdev_byte_packet_t*)malloc(sizeof(subdev_byte_packet_t));
  COMM_BUS.readBytes((uint8_t *)bytePacket, sizeof(subdev_byte_packet_t));
  uint8_t swStatus = bytePacket->data;
  uint16_t crcRec = bytePacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)bytePacket, sizeof(subdev_byte_packet_t)-2);
  free(bytePacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadSwitchStatus - ERROR: data from sub dev %d got corrupted\n", m_id);
    return false;
  }
  m_swStatus = swStatus; // wait until we pass all error checks to update the swStatus member variable
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:29:02 
 * @Return: true if front limit is triggered,
 * false if not
 * @Note: be sure to call ReadSwitchStatus()
 * to update m_swStatus regularly 
 */
bool RearSubDev::IsM5FrontLimitTriggered(void) {
  return ((m_swStatus & M5_FRONT_LIMIT_MASK) > 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:30:29 
 * @Return: true if rear limit is triggered,
 * false if not
 * @Note: be sure to call ReadSwitchStatus()
 * to update m_swStatus regularly 
 */
bool RearSubDev::IsM5RearLimitTriggered(void) {
  return ((m_swStatus & M5_REAR_LIMIT_MASK) > 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 15:31:30 
 * @Return: true if dump pos switch is triggered,
 * false if not
 * @Note: be sure to call ReadSwitchStatus()
 * to update m_swStatus regularly 
 */
bool RearSubDev::IsM5DumpPosLimitTriggered(void) {
  return ((m_swStatus & M5_DUMP_POS_MASK) > 0);
}
// **** END PUBLIC FUNCTIONS ****