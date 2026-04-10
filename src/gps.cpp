#include "gps.h"

#include <cstdlib>
#include <TinyGPSPlus.h>

TinyGPSPlus g_gps;
TinyGPSCustom g_vdop(g_gps, "GPGSA", 17);

/**
 * @brief Sends a SkyTraq binary command packet to the GPS module.
 *
 * Packet format follows the command structure used by the Autosport Labs
 * GPS_2_CAN example: A0 A1, 16-bit payload length, payload bytes, XOR checksum,
 * then 0D 0A terminator.
 */
void sendSkytraqCommand(const uint8_t* payload, size_t payloadLen) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < payloadLen; ++i) {
        checksum ^= payload[i];
    }

    Serial1.write(0xA0);
    Serial1.write(0xA1);
    Serial1.write(static_cast<uint8_t>((payloadLen >> 8) & 0xFF));
    Serial1.write(static_cast<uint8_t>(payloadLen & 0xFF));
    Serial1.write(payload, payloadLen);
    Serial1.write(checksum);
    Serial1.write(0x0D);
    Serial1.write(0x0A);
}

/**
 * @brief Configures the SkyTraq GPS UART baud rate.
 *
 * The GPS module boots with a default UART baud rate of 9600. All configuration
 * commands must initially be sent at this baud rate, otherwise the module will
 * not understand them.
 *
 * The Autosport Labs GPS_2_CAN example switches the module from 9600 to 115200
 * before configuring the update frequency. This is required because higher GPS
 * update rates (e.g. 10–50 Hz) produce significantly more NMEA traffic, which
 * would exceed the throughput of 9600 baud and result in dropped or corrupted
 * data.
 */
void configureSkytraqBaudRate(uint8_t baudCode) {
    const uint8_t payload[] = {0x05, 0x00, baudCode, 0x00};
    sendSkytraqCommand(payload, sizeof(payload));
}

/**
 * @brief Configures the SkyTraq GPS output update rate in Hz.
 *
 * This command must be sent after the baud rate has been increased from the
 * default 9600 to a higher value (e.g. 115200). At higher update rates, the
 * volume of NMEA output increases proportionally, and a low baud rate will
 * cause UART buffer overflow and data loss.
 *
 */
void configureSkytraqUpdateRate(uint8_t rateHz) {
    const uint8_t payload[] = {0x0E, rateHz, 0x00};
    sendSkytraqCommand(payload, sizeof(payload));
}

/**
 * @brief Initializes the GPS module and configures baud rate and update rate.
 *
 * Initialization follows a strict sequence:
 * 1. Start UART at 9600 baud (the GPS module's default at boot)
 * 2. Send command to switch GPS to 115200 baud
 * 3. Wait for the GPS module to apply the new baud rate
 * 4. Reconfigure ESP32 UART to 115200 baud
 * 5. Configure the desired GPS update rate
 *
 * This sequence is required because the GPS will only accept configuration
 * commands at its current baud rate. Skipping the initial 9600 step will
 * result in the GPS ignoring configuration commands.
 */
bool initGps() {
    Serial1.begin(GPS_INITIAL_BAUD, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    configureSkytraqBaudRate(SKYTRAQ_BAUD_CODE_115200);
    delay(1000);
    
    Serial1.begin(GPS_OPERATING_BAUD, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    configureSkytraqUpdateRate(GPS_UPDATE_RATE);

    Serial.println("GPS ready");
    return true;
}