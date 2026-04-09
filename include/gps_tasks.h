#pragma once
#include <freertos/FreeRTOS.h>

constexpr UBaseType_t PRIO_GPS_READ = 4;

void startGpsTasks();
