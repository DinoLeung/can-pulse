#pragma once

#include <stdint.h>
#include "FreeRTOS.h"
#include "freertos/semphr.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <freertos/queue.h>

static constexpr const char* kDeviceName = "CAN Pulse BLE";
static constexpr uint16_t kServiceUuid = 0x1FF8;
static constexpr uint16_t kCanMainCharUuid = 0x0001;
static constexpr uint16_t kCanFilterCharUuid = 0x0002;
static constexpr uint16_t kGpsMainCharUuid = 0x0003;
static constexpr uint16_t kGpsTimeCharUuid = 0x0004;

static constexpr uint16_t kInvalidBleConnId = 0xFFFF;

static constexpr size_t kMaxRequestedPids = 16;
static constexpr size_t kFilterRequestQueueSize = 8;

struct RequestedPid {
	uint32_t pid;
	uint16_t intervalMs;
	uint32_t nextDueMs;
	bool active;
};

struct PidFilterState {
	bool allowAll;
	uint16_t allowAllIntervalMs;
	RequestedPid requestedPids[kMaxRequestedPids];
	size_t requestedPidCount;
	SemaphoreHandle_t mutex;
	
	void markPidSent(size_t index, uint32_t now);
	
	bool nextDuePid(
		uint32_t now,
		size_t& requestedPidCursor,
		RequestedPid& outRequestedPid,
		size_t& outRequestedPidIndex) const;

	void snapshot(
		bool& outAllowAll,
		uint16_t& outAllowAllIntervalMs,
		RequestedPid (&outRequestedPids)[kMaxRequestedPids],
		size_t& outRequestedPidCount) const;
};

extern PidFilterState g_rcPidFilterState;
extern QueueHandle_t g_rcPidFilterRequestQueue;

extern BLEServer* g_rcBleServer;
extern BLECharacteristic* g_rcBleMainChar;
extern BLECharacteristic* g_rcBleGpsMainChar;
extern BLECharacteristic* g_rcBleGpsTimeChar;
extern volatile bool g_rcBleConnected;
extern uint16_t g_rcBleConnId;

bool initRaceChronoBle();