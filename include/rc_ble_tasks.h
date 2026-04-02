#pragma once
#include <freertos/FreeRTOS.h>

constexpr UBaseType_t PRIO_BLE_GPS_NOTIFY = 4;
constexpr UBaseType_t PRIO_CAN_NOTIFY = 2;

void startRaceChronoTasks();