#include "rc_ble_tasks.h"
#include <cstdlib>
#include "esp_gatt_common_api.h"
#include "rc_ble.h"
#include "rc_ble_helper.h"
#include "gps_helper.h"
#include "can_frame_cache.h"
#include "gps_snapshot.h"

// Target 500Hz, it's shared across all the can PIDs, realistic 300Hz avg
constexpr TickType_t NotifyInterval = pdMS_TO_TICKS(2);
// 25Hz
constexpr TickType_t GpsNotifyInterval = pdMS_TO_TICKS(40);

static void raceChronoCanFilterRequestTask(void*);
static void raceChronoCanNotifyTask(void*);
static void raceChronoGpsNotifyTask(void*);

void startRaceChronoTasks() {
	xTaskCreate(raceChronoCanFilterRequestTask, "RaceChronoCanFilterRequest", 4096, nullptr, 1, nullptr);
	xTaskCreate(raceChronoCanNotifyTask, "RaceChronoCanNotify", 4096, nullptr, PRIO_CAN_NOTIFY, nullptr);
	// xTaskCreate(raceChronoGpsNotifyTask, "RaceChronoGpsNotify", 4096, nullptr, PRIO_BLE_GPS_NOTIFY, nullptr);
}

/**
 * @brief FreeRTOS task responsible for applying incoming RaceChrono filter requests.
 *
 * This task blocks on `g_rcPidFilterRequestQueue` and updates the shared
 * RaceChrono PID filter state whenever the client sends a new filter command.
 * Supported commands are:
 * - `DenyAll`: disable allow-all mode and clear the requested PID list.
 * - `AllowAll`: enable allow-all mode with the requested interval and clear
 *   any previously requested specific PIDs.
 * - `AllowOnePid`: disable allow-all mode and append a single requested PID
 *   with its transmission interval to the request list.
 *
 * State updates are protected by the filter state's internal mutex so the
 * notify task can safely snapshot and consume the configuration from another
 * FreeRTOS task.
 *
 * Scheduling notes:
 * - Newly added specific PIDs are initialized with `nextDueMs = millis()` so
 *   they are eligible for transmission immediately.
 * - The task itself is event-driven and does not perform periodic polling;
 *   it wakes only when a new queue entry is received.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
static void raceChronoCanFilterRequestTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	RcFilterRequest request{};

	while (true) {
		if (xQueueReceive(g_rcPidFilterRequestQueue, &request, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		if (xSemaphoreTake(g_rcPidFilterState.mutex, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		switch (request.command) {
		case RcFilterCommand::DenyAll:
			g_rcPidFilterState.allowAll = false;
			g_rcPidFilterState.allowAllIntervalMs = 0;
			g_rcPidFilterState.requestedPidCount = 0;
			for (size_t i = 0; i < kMaxRequestedPids; ++i) {
				g_rcPidFilterState.requestedPids[i] = RequestedPid{};
			}
			Serial.println("RaceChrono filter updated: deny all");
			break;

		case RcFilterCommand::AllowAll:
			g_rcPidFilterState.allowAll = true;
			g_rcPidFilterState.allowAllIntervalMs = request.intervalMs;
			g_rcPidFilterState.requestedPidCount = 0;
			Serial.printf(
				"RaceChrono filter updated: allow all: interval=%u ms\n",
				static_cast<unsigned>(request.intervalMs));
			break;

		case RcFilterCommand::AllowOnePid:
			g_rcPidFilterState.allowAll = false;
			g_rcPidFilterState.allowAllIntervalMs = 0;
			if (g_rcPidFilterState.requestedPidCount >= kMaxRequestedPids) {
				Serial.println("RaceChrono filter request ignored: requested PID list full");
				break;
			}

			RequestedPid requestedPid{};
			requestedPid.pid = request.pid;
			requestedPid.active = true;
			// requestedPid.intervalMs = request.intervalMs;
			// We use the notify task interval instead, so it's at best effort
			requestedPid.intervalMs = NotifyInterval;
			requestedPid.nextDueMs = millis();

			g_rcPidFilterState.requestedPids[g_rcPidFilterState.requestedPidCount] = requestedPid;
			++g_rcPidFilterState.requestedPidCount;

			Serial.printf(
				"RaceChrono filter updated: allow pid=0x%08lX\n",
				static_cast<unsigned long>(requestedPid.pid));
			break;
		}
		xSemaphoreGive(g_rcPidFilterState.mutex);
	}
}

/**
 * @brief FreeRTOS task responsible for streaming CAN frames over BLE to RaceChrono.
 *
 * This task runs at a fixed interval (`NotifyInterval`, currently 2 ms / 500 Hz)
 * and selects at most one CAN frame per iteration for transmission over the
 * RaceChrono CAN main characteristic.
 *
 * High-level flow per iteration:
 * 1. Check BLE connection state, connection ID validity, and characteristic
 *    availability. If any of these are invalid, back off and retry later.
 * 2. Query the BLE stack for the number of currently sendable packets for the
 *    active connection using `esp_ble_get_cur_sendable_packets_num()`. If no
 *    transmit buffer is available, skip this iteration.
 * 3. Snapshot the current filter configuration from `PidFilterState` to avoid
 *    holding locks during frame selection and payload building.
 * 4. Depending on mode:
 *    - Allow-all: round-robin through the cache using `allowAllCursor`.
 *    - Specific PIDs: select the next due PID via `nextDuePid()` and fetch
 *      its cached frame.
 * 5. If a frame is available, pack it into RaceChrono's 13-byte payload format
 *    (4-byte identifier + 8-byte data + implicit DLC=8) and notify via BLE.
 * 6. In specific-PID mode, update scheduling state using `markPidSent()` after
 *    a successful notify attempt.
 *
 * Concurrency model:
 * - Filter state and cache access are internally synchronized via mutexes.
 * - This task snapshots filter state up front and uses thread-safe cache
 *   accessors so it does not hold locks while notifying over BLE.
 *
 * Scheduling notes:
 * - `allowAllCursor` and `requestedPidCursor` maintain fairness across cached
 *   frames and requested PIDs respectively.
 * - The task interval (`NotifyInterval`) sets the minimum gap between notify
 *   attempts, not a guaranteed delivery interval.
 * - BLE transmit buffer availability is used as backpressure, so the task will
 *   intentionally skip ticks when the stack cannot accept another packet.
 * - When no filters are active, the task backs off for longer to reduce CPU
 *   usage.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
static void raceChronoCanNotifyTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	static size_t allowAllCursor = 0;
	static size_t requestedPidCursor = 0;
	while (true) {
		if (!g_rcBleConnected || g_rcBleConnId == kInvalidBleConnId || g_rcBleMainChar == nullptr) {
			vTaskDelayUntil(&lastWake, NotifyInterval * 100);
			continue;
		}

		// Skip the tick if there's no buffer
		const uint16_t sendablePackets = esp_ble_get_cur_sendable_packets_num(g_rcBleConnId);
		if (sendablePackets == 0) {
			vTaskDelayUntil(&lastWake, NotifyInterval);
			continue;
		}

		const uint32_t now = millis();
		bool allowAll = false;
		uint16_t allowAllIntervalMs = 0;
		RequestedPid requestedPids[kMaxRequestedPids]{};
		size_t requestedPidCount = 0;

		// Snapshot current filter state
		g_rcPidFilterState.snapshot(
			allowAll,
			allowAllIntervalMs,
			requestedPids,
			requestedPidCount);

		if (!allowAll && requestedPidCount <= 0) {
			// Tick longer to free up cpu time
			vTaskDelayUntil(&lastWake, NotifyInterval * 1000);
			continue;
		}

		bool hasFrameToSend = false;
		uint32_t framePid = 0;
		uint8_t frameData[8]{};
		size_t selectedRequestedPidIndex = kMaxRequestedPids;

		// Snapshot cache allow all mode
		if (allowAll) {
			hasFrameToSend = g_canFrameCache.getNextCachedFrame(allowAllCursor, framePid, frameData);
		}

		// Snapshot cache specific PIDs mode
		if (requestedPidCount > 0) {
			RequestedPid duePid;
			const bool hasDue = g_rcPidFilterState.nextDuePid(
				now,
				requestedPidCursor,
				duePid,
				selectedRequestedPidIndex);

			if (hasDue) {
				hasFrameToSend = g_canFrameCache.getNextRequestedCachedFrame(
					duePid,
					framePid,
					frameData);
			}
		}

		if (hasFrameToSend) {
			uint8_t payload[13];
			buildRcCanMainPayload(framePid, frameData, payload);

			g_rcBleMainChar->setValue(payload, sizeof(payload));
			g_rcBleMainChar->notify();

			// Update due time for the PID we just sent.
			if (!allowAll && selectedRequestedPidIndex < requestedPidCount) {
				g_rcPidFilterState.markPidSent(selectedRequestedPidIndex, now);
			}
		}

		vTaskDelayUntil(&lastWake, NotifyInterval);
	}
}

/**
 * @brief FreeRTOS task responsible for streaming GPS data over BLE to RaceChrono.
 *
 * This task periodically samples the shared TinyGPSPlus parser state and encodes
 * it into the two RaceChrono GPS characteristics:
 * - UUID 0x0003: GPS main payload
 * - UUID 0x0004: GPS date/hour payload
 *
 * Both characteristics are updated with the same 3-bit sync counter on every
 * transmission so the RaceChrono client can match them as one logical sample.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
static void raceChronoGpsNotifyTask(void* pvParameters) {
	(void)pvParameters;
	TickType_t lastWake = xTaskGetTickCount();
	static uint8_t gpsSyncBits = 0;

	while (true) {
		if (!g_rcBleConnected || g_rcBleGpsMainChar == nullptr || g_rcBleGpsTimeChar == nullptr) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		GpsSnapshot snapshot{};
		getGpsSnapshot(snapshot);
		const uint8_t currentSyncBits = static_cast<uint8_t>(gpsSyncBits & 0b111);

		uint8_t gpsTimePayload[3];
		buildRcGpsTimePayload(currentSyncBits, snapshot, gpsTimePayload);
		g_rcBleGpsTimeChar->setValue(gpsTimePayload, sizeof(gpsTimePayload));
		
		uint8_t gpsMainPayload[20];
		buildRcGpsMainPayload(currentSyncBits, snapshot, gpsMainPayload);
		g_rcBleGpsMainChar->setValue(gpsMainPayload, sizeof(gpsMainPayload));

		g_rcBleGpsTimeChar->notify();
		g_rcBleGpsMainChar->notify();

		gpsSyncBits = static_cast<uint8_t>((gpsSyncBits + 1) & 0b111);
		vTaskDelayUntil(&lastWake, GpsNotifyInterval);
	}
}
