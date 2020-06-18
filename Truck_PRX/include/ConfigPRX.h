/** @author Kodiak North
 * @date 05/22/2020
 * pin definitions and other #defines for the
 * PRX module
 */

#ifndef CONFIG_PRX_H_
#define CONFIG_PRX_H_

#include <Arduino.h>

typedef struct
{
	uint8_t		Phase;
	uint8_t		LEDControl;
	int32_t 	FrontEncoder;
} TX_TO_RX;

typedef struct
{
	uint8_t		SwitchStatus;
	uint8_t		SolenoidStatus;
} RX_TO_TX;

#define NUM_TTR_BYTES sizeof(TX_TO_RX)
#define NUM_RTT_BYTES sizeof(RX_TO_TX)

/** PRX writes to the same address that it reads
 * from in Enhanced ShockBurst mode */
#define RF_PRX_READ_ADDR        0xA1B2C3D4E5
#define RF_CHANNEL              120 // this is not a pin
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
#define RF_CSN_PIN				53

#endif /* CONFIG_PRX_H_ */