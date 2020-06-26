/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 14:50:40 
 * @Desc: implements the sub device base class 
 */

#include "SubDevice.h"

// **** PUBLIC FUNCTIONS ****
SubDev::SubDev(subdev_id_t id, uint8_t ssPin, uint16_t responseTimeoutUs) {
  m_id = id;
  m_ssPin = ssPin;
  m_timeout = responseTimeoutUs;
  pinMode(ssPin, OUTPUT);
  digitalWrite(ssPin, LOW);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 11:34:59 
 * @Desc: writes a command to a sub device
 * @Param - device: device to communicate with
 * @Param - cmd: the command being sent
 */
SubDev::subdev_response_t SubDev::WriteCmd(uint8_t cmd) {
  subdev_cmd_packet_t *packet = (subdev_cmd_packet_t*)malloc(sizeof(subdev_cmd_packet_t));
  ConfigPacket(packet, cmd);
  uint8_t trys = NUM_RETRYS;
  subdev_response_t response = ERROR; // default to no response error
  AssertSSLine();

  // if no response, or CRC doesn't match, keep trying
  while (trys-- && (response <= CRC_ERROR)) {
    COMM_BUS.write((unsigned char *)packet, sizeof(subdev_cmd_packet_t));
    response = ReadByteOrTimeout();
  }
  free(packet);
  ClearSSLine();
  return response;
}
// **** END PUBLIC FUNCTIONS ****

// **** PROTECTED FUNCTIONS ****
/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-24 12:02:31 
 * @Desc: waits until timeout for data available on
 * COMM_BUS after receiving DATA_INCOMING response
 * @Return: true if data received, false if timeout
 */
bool SubDev::RecDataOrTimeout(void) {
  uint32_t start = micros();
  while (!COMM_BUS.available()) {
    if (micros()-start >= TIMEOUT_US_REC_DATA)
      return false;
  }
  return true;
}

// Calculates CRC16 of nBytes of data in byte array message
uint16_t SubDev::GetCRC16(unsigned char *buf, int nBytes) {
	uint16_t crc = 0;
	for (int byte = 0; byte < nBytes; byte++) {
		crc = crc ^ ((unsigned int)buf[byte] << 8);
		for (unsigned char bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}
 	return crc;
}
// **** END PROTECTED FUNCTIONS ****

// **** PRIVATE FUNCTIONS ****
/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-22 09:40:27 
 * @Desc: reads a byte if available in the timeout duration
 * @Return: byte read, or 0 if timeout
 */
SubDev::subdev_response_t SubDev::ReadByteOrTimeout(void) {
  uint32_t start = micros();
  while (!COMM_BUS.available()) {
    if (micros() - start >= m_timeout) {
      return ERROR;
    }
  }
  return (subdev_response_t)(COMM_BUS.read());
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 10:37:09 
 * @Desc: asserts the SS line for the device passed and
 * returns device-specific timeout value
 */
void SubDev::AssertSSLine(void) {
  digitalWrite(m_ssPin, HIGH);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-23 10:51:53 
 * @Desc: clears the SS line for the device passed
 */
void SubDev::ClearSSLine(void) {
  digitalWrite(m_ssPin, LOW);
}

/** 
 * @Author: Kodiak North 
 * @Date: 2020-06-19 09:31:10 
 * @Desc: sets newCmd member variables and computes crc
 * @Param - newCmd: pointer to command struct
 * @Param - addr: unique address of sub device
 * @Param - cmdByte: byte corresponding to sub device command
 */
void SubDev::ConfigPacket(subdev_cmd_packet_t *newCmd, uint8_t cmdByte) {
  newCmd->command = cmdByte;
  newCmd->crc = GetCRC16((unsigned char *)newCmd, sizeof(subdev_cmd_packet_t)-2);
}
// **** END PRIVATE FUNCTIONS ****