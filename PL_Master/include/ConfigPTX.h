/** @author Kodiak North
 * @date 05/22/2020
 * pin definitions and other #defines for the
 * PTX module
 */

#ifndef CONFIG_PTX_H_
#define CONFIG_PTX_H_

#include <Arduino.h>
#include <SPI.h>

// **** TRUCK DEFINES ****
typedef struct {
	uint8_t		Phase;
	uint8_t		LEDControl;
	int32_t 	FrontEncoder;
} TX_TO_RX;

typedef struct {
	uint8_t		SwitchStatus;
	uint8_t		SolenoidStatus;
} RX_TO_TX;

#define NUM_TTR_BYTES sizeof(TX_TO_RX)
#define NUM_RTT_BYTES sizeof(RX_TO_TX)

/** write and read addresses must be identical on the PTX
 * when using Enhanced ShockBurst mode, and read pipe must
 * be P0 */
#define RF_PTX_WRITE_ADDR       0xA1B2C3D4E5
#define RF_PTX_READ_ADDR_P0     RF_PTX_WRITE_ADDR
#define RF_CHANNEL              120
/** uncomment RF_USE_IRQ_PIN if IRQ pin is mapped from RF
 * module to Arduino. @note that code is set up such that
 * the module's IRQ pin does not need to be mapped to an
 * interrupt pin on the Arduino */
#define RF_USE_IRQ_PIN
#define RF_IRQ_PIN              48
#define RF_CE_PIN				49
#define	RF_MISO_PIN             50
#define RF_MOSI_PIN             51
#define RF_CLK_PIN              52
#define RF_CSN_PIN				53 // slave select pin
// **** END TRUCK DEFINES ****

// **** SUB DEV DEFINES ****
#define LF_SUBDEV_SS_PIN 2
#define RF_SUBDEV_SS_PIN 3
#define LR_SUBDEV_SS_PIN 4
#define RR_SUBDEV_SS_PIN 5

#define NUM_RETRYS 3
#define TIMEOUT_US_FRONT_SUB 500 // microseconds
#define TIMEOUT_US_REAR_SUB 2000 // microseconds // longer for rear devs since they communicate with rclaws
#define TIMEOUT_US_REC_FLOAT_DATA 150 // timeout waiting for float data available after sending request cmd
#define REC_FLOAT_ERROR_RESPONSE -17349.21 // if error receiving a float, this value is returned
// **** END SUB DEV DEFINES ****

#endif /* CONFIG_PTX_H_ */