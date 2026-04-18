#pragma once
#include <freertos/FreeRTOS.h>

constexpr UBaseType_t PRIO_BLE_GPS_NOTIFY = 4;
constexpr UBaseType_t PRIO_BLE_CAN_NOTIFY = 3;
constexpr UBaseType_t PRIO_BLE_CAN_FILTER = 2;

void startRaceChronoTasks();