#pragma once

#include <TinyGPSPlus.h>

static constexpr int GPS_UART_RX_PIN = 41;
static constexpr int GPS_UART_TX_PIN = 40;
static constexpr uint32_t GPS_INITIAL_BAUD = 9600;
static constexpr uint32_t GPS_OPERATING_BAUD = 115200;
static constexpr uint8_t SKYTRAQ_BAUD_CODE_115200 = 5;
static constexpr uint32_t GPS_UPDATE_RATE = 50;

extern TinyGPSPlus g_gps;
extern TinyGPSCustom g_vdop;

void initGps();