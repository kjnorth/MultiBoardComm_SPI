/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 15:16:54 
 * @Desc: implements the FrontSubDev class
 * derived from SubDev class 
 */

#include "FrontSubDevice.h"

// **** PUBLIC FUNCTIONS ****
/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 15:01:04 
 * @Desc: requests pitch from sub device
 * @Param - device: device to get pitch from
 * @Return: pitch data or REC_FLOAT_ERROR_RESPONSE
 * if error occurred
 */
float FrontSubDev::ReadPitch() {
  subdev_response_t response = WriteCmd(GET_PITCH);
  if (response != DATA_INCOMING) {
    LogInfo("ReadPitch - ERROR: sub dev not accepting pitch request, response 0x%X\n", response);
    return REC_FLOAT_ERROR_RESPONSE;
  }

  if (!RecDataOrTimeout()) {
    LogInfo("ReadPitch - ERROR: sub dev didn't send float data after responding to request cmd\n");
    return REC_FLOAT_ERROR_RESPONSE;
  }

  subdev_float_packet_t *floatPacket = (subdev_float_packet_t*)malloc(sizeof(subdev_float_packet_t));
  COMM_BUS.readBytes((uint8_t *)floatPacket, sizeof(subdev_float_packet_t));
  float pitch = floatPacket->data;
  uint16_t crcRec = floatPacket->crc;
  uint16_t crcCalc = GetCRC16((unsigned char *)floatPacket, sizeof(subdev_float_packet_t)-2);
  free(floatPacket);

  if (crcCalc != crcRec) {
    LogInfo("ReadPitch - ERROR: float data got corrupted\n");
    return REC_FLOAT_ERROR_RESPONSE;
  }
  return pitch;
}
// **** END PUBLIC FUNCTIONS ****