#pragma once
#include <freertos/FreeRTOS.h>

constexpr UBaseType_t PRIO_CAN_READ = 6;

void startCanTasks(BaseType_t xCoreID);