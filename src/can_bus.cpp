#include "can_bus.h"
#include <mcp_can.h>
#include "esp_log.h"

// CAN1 (TWAI) Pins
#define CAN1_RX_PIN GPIO_NUM_6
#define CAN1_TX_PIN GPIO_NUM_7

// CAN2 (MCP2515) Custom SPI Pins
#define CAN2_CS_PIN GPIO_NUM_10
#define CAN2_SPI_SCK GPIO_NUM_12
#define CAN2_SPI_MISO GPIO_NUM_13
#define CAN2_SPI_MOSI GPIO_NUM_11

static const char *TAG = "can_bus";

// HSPI bus
static SPIClass CAN2_SPI(HSPI);
static MCP_CAN CAN2(&CAN2_SPI, CAN2_CS_PIN);

static bool initCan1();
static bool initCan2();

bool initCanBus() {
	// return initCan1() && initCan2();
	return initCan1();
}

/**
 * @brief Initialize and start the TWAI (CAN1) driver on the configured pins.
 *
 * This function installs the TWAI driver using the default general, timing,
 * and filter configurations (read-only mode, 250 kbit/s, accept all filters),
 * then starts the driver. It logs success or failure messages.
 *
 * @return true if the driver was installed and started successfully.
 * @return false if driver installation or startup fails.
 */
bool initCan1() {
	twai_general_config_t g_config =
		TWAI_GENERAL_CONFIG_DEFAULT(
			(gpio_num_t)CAN1_TX_PIN,
			(gpio_num_t)CAN1_RX_PIN,
			TWAI_MODE_LISTEN_ONLY
		);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to install TWAI driver");
		return false;
	}

	if (twai_start() != ESP_OK) {
		ESP_LOGE(TAG, "Failed to start TWAI driver");
		return false;
	}

	ESP_LOGI(TAG, "CAN1 (TWAI) started. Waiting for messages...");
	return true;
}

/**
 * @brief Initialize and start the MCP2515 (CAN2) driver over the custom SPI interface.
 *
 * This function configures the HSPI bus and initializes the MCP2515 controller
 * at 1 Mbit/s with a 16 MHz oscillator, then sets it to normal operation mode.
 * It logs success or failure messages.
 *
 * @return true if the MCP2515 driver was initialized and set to normal mode successfully.
 * @return false if initialization or mode setting fails.
 */
bool initCan2() {
	CAN2_SPI.begin(CAN2_SPI_SCK, CAN2_SPI_MISO, CAN2_SPI_MOSI, CAN2_CS_PIN);

	if (CAN2.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
		ESP_LOGE(TAG, "Failed to start MCP2515 driver");
		return false;
	}
	
	CAN2.setMode(MCP_NORMAL);
	ESP_LOGI(TAG, "CAN2 (MCP2515) started...");
	return true;
}

/**
 * @brief Send a CAN frame out over the MCP2515-based CAN2 bus.
 *
 * Logs the outgoing frame identifier, type (standard or extended), and data payload to Serial,
 * then transmits it using the MCP_CAN interface.
 *
 * @param message The TWAI message structure containing identifier, data length, and payload bytes.
 * @return true if the frame was transmitted successfully (CAN_OK).
 * @return false if transmission failed, logging the error code.
 */
bool writeCan2(const twai_message_t& message) {
	ESP_LOGD(TAG, "Sending message to CAN2 [0x%08lX %s]: ", message.identifier, message.extd ? "EXT" : "STD");
	
	// for (byte i = 0; i < message.data_length_code; i++) {
	// 	ESP_LOGD(TAG, "0x%02X", message.data[i]);
	// if (i < message.data_length_code - 1)
	// 	Serial.print(", ");
	// }
	// Serial.println();

	// Send the message via CAN2
	byte result = CAN2.sendMsgBuf(message.identifier, message.extd, message.data_length_code, const_cast<uint8_t*>(message.data));

	if (result == CAN_OK) {
		ESP_LOGD(TAG, "Message sent successfully via CAN2");
		return true;
	} else {
		ESP_LOGD(TAG, "Message send failed, code: %d\n", result);
		return false;
	}
}
