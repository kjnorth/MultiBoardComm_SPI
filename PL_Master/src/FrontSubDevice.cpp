/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 15:16:54 
 * @Desc: implements the FrontSubDev class
 * derived from SubDev class 
 */

#include "FrontSubDevice.h"

// **** PRIVATE DEFINES ****
#define SOLA_IN_MASK 0x01
#define SOLC_IN_MASK 0x02
#define SOLE_IN_MASK 0x04
// **** END PRIVATE DEFINES ****

// **** PUBLIC FUNCTIONS ****
FrontSubDev::FrontSubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs)
  : SubDev{id, ssPin, responseTimeoutUs} {
  m_solStatus = 0;
  m_pitch = 0.0;
  m_roll = 0.0;
  subdev_response_t response;
  switch (id) {
    case LEFT_FRONT:
      m_pitchOffset = LF_SUBDEV_PITCH_OFFSET;
      m_rollOffset = LF_SUBDEV_ROLL_OFFSET;
      response = WriteCmd(INIT_LF);
      break;
    case RIGHT_FRONT:
      m_pitchOffset = RF_SUBDEV_PITCH_OFFSET;
      m_rollOffset = RF_SUBDEV_ROLL_OFFSET;
      response = WriteCmd(INIT_RF);
      break;
    default:
      LogInfo("FrontSubDev - ERROR id %d is not a FRONT device", id);
      break;
  }
  if (response != SUCCESS) {
    LogInfo("FrontSubDev - ERROR initialization of device %d failed\n", id);
  }
  else {
    LogInfo("FrontSubDev - SUCCESS initialization of device %d succeeded\n", id);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:29:55 
 * @Desc: disables all solenoids on front sub device
 * @Return: true if cmd successful, false if failure 
 */
bool FrontSubDev::DisableSolenoids(void) {
  subdev_response_t response = WriteCmd(SOLS_DISABLE);
  if (response != SUCCESS) {
    LogInfo("DisableSolenoids - ERROR disabling solenoids on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:44:42 
 * @Desc: enables solenoid A on front sub device
 * @Return: true if cmd successful, false if failure
 */
bool FrontSubDev::EnableSolenoidA(void) {
  subdev_response_t response = WriteCmd(SOLA_ENABLE);
  if (response != SUCCESS) {
    LogInfo("EnableSolenoidA - ERROR enabling solenoid A on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:45:36 
 * @Desc: enables solenoid C on front sub device
 * @Return: true if cmd successful, false if failure
 */
bool FrontSubDev::EnableSolenoidC(void) {
  subdev_response_t response = WriteCmd(SOLC_ENABLE);
  if (response != SUCCESS) {
    LogInfo("EnableSolenoidC - ERROR enabling solenoid C on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:46:17 
 * @Desc: enables solenoid E on front left sub device
 * @Return: true if cmd successful, false if failure
 */
bool FrontSubDev::EnableSolenoidE(void) {
  if (m_id != LEFT_FRONT) {
    LogInfo("EnableSolenoidE - ERROR only LEFT_FRONT sub device has solenoid E, this device is %d\n", m_id);
    return false;
  }

  subdev_response_t response = WriteCmd(SOLE_ENABLE);
  if (response != SUCCESS) {
    LogInfo("EnableSolenoidE - ERROR enabling solenoid E on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 13:26:05 
 * @Desc: turns laser on front sub dev
 * on or off
 * @Param - on_off: true turns laser ON,
 * false turns laser OFF
 * @Return: true if cmd successful, false if failure
 */
bool FrontSubDev::CommandLaser(bool on_off) {
  subdev_cmd_t cmd = on_off ? LASER_ENABLE : LASER_DISABLE;
  subdev_response_t response = WriteCmd(cmd);
  if (response != SUCCESS) {
    LogInfo("CommandLaser - ERROR commanding laser on device %d\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 15:01:04 
 * @Desc: requests pitch from sub device
 * @Return: true if pitch read successfully, false otherwise
 */
bool FrontSubDev::ReadPitch() {
  subdev_response_t response = WriteCmd(GET_PITCH);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadPitch - ERROR: sub dev %d not accepting pitch request, response 0x%X\n"), m_id, response);
    return false;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadPitch - ERROR: sub dev %d didn't send data after responding to request cmd\n", m_id);
    return false;
  }

  subdev_float_packet_t *floatPacket = (subdev_float_packet_t*)malloc(sizeof(subdev_float_packet_t));
  COMM_BUS.readBytes((uint8_t *)floatPacket, sizeof(subdev_float_packet_t));
  float pitch = floatPacket->data;
  uint16_t crcRec = floatPacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)floatPacket, sizeof(subdev_float_packet_t)-2);
  free(floatPacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadPitch - ERROR: data from sub dev %d got corrupted\n", m_id);
    return false;
  }
  m_pitch = pitch; // wait until we pass all error checks to update the pitch member variable
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:28:20 
 * @Desc: requests roll from sub device
 * @Return: true if roll read successfully, false otherwise
 */
bool FrontSubDev::ReadRoll() {
  subdev_response_t response = WriteCmd(GET_PITCH);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadRoll - ERROR: sub dev %d not accepting roll request, response 0x%X\n"), m_id, response);
    return false;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadRoll - ERROR: sub dev %d didn't send data after responding to request cmd\n", m_id);
    return false;
  }

  subdev_float_packet_t *floatPacket = (subdev_float_packet_t*)malloc(sizeof(subdev_float_packet_t));
  COMM_BUS.readBytes((uint8_t *)floatPacket, sizeof(subdev_float_packet_t));
  float roll = floatPacket->data;
  uint16_t crcRec = floatPacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)floatPacket, sizeof(subdev_float_packet_t)-2);
  free(floatPacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadRoll - ERROR: data from sub dev %d got corrupted\n", m_id);
    return false;
  }
  m_roll = roll; // wait until we pass all error checks to update the roll member variable
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 14:42:34 
 * @Return: pitch from most recent read
 * with offset subtracted
 * @Note: be sure to call ReadPitch() to
 * update m_pitch regularly 
 */
float FrontSubDev::GetPitch(void) {
  return m_pitch - m_pitchOffset;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 14:46:05 
 * @Return: roll from moste recent read
 * with ofsset subtracted
 * @Note: be sure to call ReadRoll() to
 * update m_roll regularly 
 */
float FrontSubDev::GetRoll(void) {
  return m_roll - m_pitchOffset;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 11:01:54 
 * @Desc: requests solenoid data from front sub device
 * @Return: true if status read successfully, false otherwise
 */
bool FrontSubDev::ReadSolenoidStatus(void) {
  subdev_response_t response = WriteCmd(GET_SOL_STATUS);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadSolenoidStatus - ERROR: sub dev %d not accepting solenoid status request, response 0x%X\n"), m_id, response);
    return false;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadSolenoidStatus - ERROR: sub dev %d didn't send data after responding to request cmd\n", m_id);
    return false;
  }

  subdev_byte_packet_t *bytePacket = (subdev_byte_packet_t*)malloc(sizeof(subdev_byte_packet_t));
  COMM_BUS.readBytes((uint8_t *)bytePacket, sizeof(subdev_byte_packet_t));
  uint8_t solStatus = bytePacket->data;
  uint16_t crcRec = bytePacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)bytePacket, sizeof(subdev_byte_packet_t)-2);
  free(bytePacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadSolenoidStatus - ERROR: data from sub dev %d got corrupted\n", m_id);
    return false;
  }
  m_solStatus = solStatus; // wait until we pass all error checks to update the solStatus member variable
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:14:42 
 * @Return: true if solenoid A is IN,
 * false if not
 * @Note: be sure to call ReadSolenoidStatus()
 * to update m_solStatus regularly
 */
bool FrontSubDev::IsSolenoidAIn(void) {
  return ((m_solStatus & SOLA_IN_MASK) > 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:18:28 
 * @Return: true if solenoid A is OUT,
 * false if not
 * @Note: be sure to call ReadSolenoidStatus()
 * to update m_solStatus regularly
 */
bool FrontSubDev::IsSolenoidAOut(void) {
  return ((m_solStatus & SOLA_IN_MASK) == 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:20:05 
 * @Return: true if solenoid C is IN,
 * false if not
 * @Note: be sure to call ReadSolenoidStatus()
 * to update m_solStatus regularly
 */
bool FrontSubDev::IsSolenoidCIn(void) {
  return ((m_solStatus & SOLC_IN_MASK) > 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:24:50 
 * @Return: true if solenoid C is OUT,
 * false if not
 * @Note: be sure to call ReadSolenoidStatus()
 * to update m_solStatus regularly
 */
bool FrontSubDev::IsSolenoidCOut(void) {
  return ((m_solStatus & SOLC_IN_MASK) == 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:26:23 
 * @Return: true if solenoid E is IN,
 * false if not
 * @Note: be sure to call ReadSolenoidStatus()
 * to update m_solStatus regularly
 */
bool FrontSubDev::IsSolenoidEIn(void) {
  if (m_id != LEFT_FRONT) {
    LogInfo("IsSolenoidEIn - ERROR only LEFT_FRONT sub device has solenoid E, this device is %d\n", m_id);
    return false;
  }
  return ((m_solStatus & SOLE_IN_MASK) > 0); 
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:29:25 
 * @Return: true if solenoid E is OUT,
 * false if not
 * @Note: be sure to call ReadSolenoidStatus()
 * to update m_solStatus regularly
 */
bool FrontSubDev::IsSolenoidEOut(void) {
  if (m_id != LEFT_FRONT) {
    LogInfo("IsSolenoidEIn - ERROR only LEFT_FRONT sub device has solenoid E, this device is %d\n", m_id);
    return false;
  }
  return ((m_solStatus & SOLE_IN_MASK) == 0);
}
// **** END PUBLIC FUNCTIONS ****