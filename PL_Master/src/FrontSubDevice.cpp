/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 15:16:54 
 * @Desc: implements the FrontSubDev class
 * derived from SubDev class 
 */

#include "FrontSubDevice.h"

#define SOLA_IN_MASK 0x01
#define SOLC_IN_MASK 0x02
#define SOLE_IN_MASK 0x04

// **** PUBLIC FUNCTIONS ****
FrontSubDev::FrontSubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs)
  : SubDev{id, ssPin, responseTimeoutUs} {
  m_solStatusByte = 0;
  m_solAStatus = 0;
  m_solCStatus = 0;
  m_solEStatus = 0;
  // TODO: Send INIT cmd here!!!!
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 15:01:04 
 * @Desc: requests pitch from sub device
 * @Return: pitch data or REC_FLOAT_ERROR_RESPONSE
 * if error occurred
 */
float FrontSubDev::ReadPitch() {
  subdev_response_t response = WriteCmd(GET_PITCH);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadPitch - ERROR: sub dev %d not accepting pitch request, response 0x%X\n"), m_id, response);
    return REC_FLOAT_ERROR_RESPONSE;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadPitch - ERROR: sub dev %d didn't send data after responding to request cmd\n", m_id);
    return REC_FLOAT_ERROR_RESPONSE;
  }

  subdev_float_packet_t *floatPacket = (subdev_float_packet_t*)malloc(sizeof(subdev_float_packet_t));
  COMM_BUS.readBytes((uint8_t *)floatPacket, sizeof(subdev_float_packet_t));
  float pitch = floatPacket->data;
  uint16_t crcRec = floatPacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)floatPacket, sizeof(subdev_float_packet_t)-2);
  free(floatPacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadPitch - ERROR: data from sub dev %d got corrupted\n", m_id);
    return REC_FLOAT_ERROR_RESPONSE;
  }
  return pitch;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:28:20 
 * @Desc: requests roll from sub device
 * @Return: pitch data or REC_FLOAT_ERROR_RESPONSE
 * if error occurred
 */
float FrontSubDev::ReadRoll() {
  subdev_response_t response = WriteCmd(GET_PITCH);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadRoll - ERROR: sub dev %d not accepting roll request, response 0x%X\n"), m_id, response);
    return REC_FLOAT_ERROR_RESPONSE;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadRoll - ERROR: sub dev %d didn't send data after responding to request cmd\n", m_id);
    return REC_FLOAT_ERROR_RESPONSE;
  }

  subdev_float_packet_t *floatPacket = (subdev_float_packet_t*)malloc(sizeof(subdev_float_packet_t));
  COMM_BUS.readBytes((uint8_t *)floatPacket, sizeof(subdev_float_packet_t));
  float roll = floatPacket->data;
  uint16_t crcRec = floatPacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)floatPacket, sizeof(subdev_float_packet_t)-2);
  free(floatPacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadRoll - ERROR: data from sub dev %d got corrupted\n", m_id);
    return REC_FLOAT_ERROR_RESPONSE;
  }
  return roll;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:29:55 
 * @Desc: disables all solenoids on front sub device 
 */
void FrontSubDev::DisableSolenoids(void) {
  subdev_response_t response = WriteCmd(SOLS_DISABLE);
  if (response != SUCCESS) {
    LogInfo("DisableSolenoids - ERROR disabling solenoids on device %d\n", m_id);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:44:42 
 * @Desc: enables solenoid A on front sub device 
 */
void FrontSubDev::EnableSolenoidA(void) {
  subdev_response_t response = WriteCmd(SOLA_ENABLE);
  if (response != SUCCESS) {
    LogInfo("EnableSolenoidA - ERROR enabling solenoid A on device %d\n", m_id);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:45:36 
 * @Desc: enables solenoid C on front sub device 
 */
void FrontSubDev::EnableSolenoidC(void) {
  subdev_response_t response = WriteCmd(SOLC_ENABLE);
  if (response != SUCCESS) {
    LogInfo("EnableSolenoidC - ERROR enabling solenoid C on device %d\n", m_id);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 10:46:17 
 * @Desc: enables solenoid E on front left sub device 
 */
void FrontSubDev::EnableSolenoidE(void) {
  if (m_id != LEFT_FRONT) {
    LogInfo("EnableSolenoidE - ERROR only LEFT_FRONT sub device has solenoid E, this device is %d\n", m_id);
    return;
  }

  subdev_response_t response = WriteCmd(SOLE_ENABLE);
  if (response != SUCCESS) {
    LogInfo("EnableSolenoidE - ERROR enabling solenoid E on device %d\n", m_id);
  }
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 11:01:54 
 * @Desc: requests solenoid data from front sub device
 * and sets member variables
 * @Return: true if status read successfully, false otherwise
 */
bool FrontSubDev::ReadSolenoidStatus(void) {
  subdev_response_t response = WriteCmd(GET_SOL_STATUS);
  if (response != DATA_INCOMING) {
    LogInfo(F("ReadSolenoidStatus - ERROR: sub dev %d not accepting solenoid status request, response 0x%X\n"), m_id, response);
    return false;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadSolenoidStatus - ERROR: sub dev %d didn't send float data after responding to request cmd\n", m_id);
    return false;
  }

  subdev_byte_packet_t *bytePacket = (subdev_byte_packet_t*)malloc(sizeof(subdev_byte_packet_t));
  COMM_BUS.readBytes((uint8_t *)bytePacket, sizeof(subdev_byte_packet_t));
  m_solStatusByte = bytePacket->data;
  uint16_t crcRec = bytePacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)bytePacket, sizeof(subdev_byte_packet_t)-2);
  free(bytePacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadSolenoidStatus - ERROR: data from sub dev %d got corrupted\n", m_id);
    return false;
  }
  return true;
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:14:42 
 * @Return: true if solenoid A is IN,
 * false if not  
 */
bool FrontSubDev::IsSolenoidAIn(void) {
  return ((m_solStatusByte & SOLA_IN_MASK) > 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:18:28 
 * @Return: true if solenoid A is OUT,
 * false if not
 */
bool FrontSubDev::IsSolenoidAOut(void) {
  return ((m_solStatusByte & SOLA_IN_MASK) == 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:20:05 
 * @Return: true if solenoid C is IN,
 * false if not   
 */
bool FrontSubDev::IsSolenoidCIn(void) {
  return ((m_solStatusByte & SOLC_IN_MASK) > 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:24:50 
 * @Return: true if solenoid C is OUT,
 * false if not
 */
bool FrontSubDev::IsSolenoidCOut(void) {
  return ((m_solStatusByte & SOLC_IN_MASK) == 0);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:26:23 
 * @Return: true if solenoid E is IN,
 * false if not 
 */
bool FrontSubDev::IsSolenoidEIn(void) {
  if (m_id != LEFT_FRONT) {
    LogInfo("IsSolenoidEIn - ERROR only LEFT_FRONT sub device has solenoid E, this device is %d\n", m_id);
    return false;
  }
  return ((m_solStatusByte & SOLE_IN_MASK) > 0); 
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-25 12:29:25 
 * @Return: true if solenoid E is OUT,
 * false if not
 */
bool FrontSubDev::IsSolenoidEOut(void) {
  if (m_id != LEFT_FRONT) {
    LogInfo("IsSolenoidEIn - ERROR only LEFT_FRONT sub device has solenoid E, this device is %d\n", m_id);
    return false;
  }
  return ((m_solStatusByte & SOLE_IN_MASK) == 0);
}
// **** END PUBLIC FUNCTIONS ****