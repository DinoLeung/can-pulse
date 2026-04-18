#pragma once
#include <freertos/FreeRTOS.h>

constexpr UBaseType_t PRIO_GPS_READ = 6;

void startGpsTasks(BaseType_t xCoreID);
