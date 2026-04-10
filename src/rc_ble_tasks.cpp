#include "rc_ble_tasks.h"
#include <cstdlib>
#include "rc_ble.h"
#include "rc_ble_helper.h"
#include "gps_helper.h"
#include "can_frame_cache.h"
#include "gps_snapshot.h"

static void raceChronoCanFilterRequestTask(void*);
static void raceChronoCanNotifyTask(void*);
static void raceChronoGpsNotifyTask(void*);

void startRaceChronoTasks() {
	xTaskCreate(raceChronoCanFilterRequestTask, "RaceChronoCanFilterRequest", 4096, nullptr, 1, nullptr);
	xTaskCreate(raceChronoCanNotifyTask, "RaceChronoCanNotify", 4096, nullptr, PRIO_CAN_NOTIFY, nullptr);
	xTaskCreate(raceChronoGpsNotifyTask, "RaceChronoGpsNotify", 4096, nullptr, PRIO_BLE_GPS_NOTIFY, nullptr);
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
 * This task runs continuously and attempts to send at most one cached CAN frame
 * per loop iteration over the RaceChrono CAN main characteristic.
 *
 * High-level flow per iteration:
 * 1. Check BLE connection state and characteristic availability. If the link
 *    is not ready, back off and retry later.
 * 2. Query the BLE stack for transmit availability using
 *    `bleCanSendNotification()`. If the stack cannot currently accept another
 *    notification, skip this iteration.
 * 3. Snapshot the current filter configuration from `PidFilterState` so frame
 *    selection can be performed without holding the filter mutex.
 * 4. Depending on mode:
 *    - Allow-all: claim the next pending cached frame in round-robin order.
 *    - Specific PIDs: select the next active requested PID in round-robin
 *      order, then try to claim its pending cached frame.
 * 5. If a frame is available, pack it into RaceChrono's 13-byte CAN payload
 *    and notify it over BLE.
 * 
 * Scheduling model:
 * - This task no longer schedules transmission based on per-PID due times.
 * - Cached CAN frames are sent on a best-effort basis using latest-value
 *   semantics.
 * - A cache entry is eligible only when it has a pending update that has not
 *   yet been claimed for BLE transmission.
 * - If a CAN frame is updated multiple times before BLE can send it, older
 *   intermediate values may be overwritten and dropped. This is intentional so
 *   BLE traffic prioritizes the freshest vehicle state instead of trying to
 *   replay backlog.
 *
 *
 * Concurrency model:
 * - Filter state and cache access are internally synchronized via mutexes.
 * - This task snapshots filter state up front and uses cache accessors that
 *   safely claim pending frames without holding locks while calling BLE notify.
 *
 * Backoff behavior:
 * - When BLE is disconnected, the task sleeps longer before retrying.
 * - When no filters are active, the task also backs off to reduce CPU usage.
 * - Otherwise, the loop runs as fast as practical and is limited mainly by BLE
 *   transmit availability and scheduler execution.
 *
 * @param pvParameters Unused FreeRTOS task parameter.
 */
static void raceChronoCanNotifyTask(void* pvParameters) {
	(void)pvParameters;
	static size_t allowAllCursor = 0;
	static size_t requestedPidCursor = 0;
	while (true) {
		if (!g_rcBleConnected || g_rcBleMainChar == nullptr) {
			vTaskDelay(pdMS_TO_TICKS(500));
			continue;
		}

		if (!bleCanSendNotification()) {
			vTaskDelay(pdMS_TO_TICKS(1));
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
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		bool hasFrameToSend = false;
		uint32_t framePid = 0;
		uint8_t frameData[8]{};
		size_t selectedRequestedPidIndex = kMaxRequestedPids;

		// Snapshot cache allow all mode
		if (allowAll) {
			hasFrameToSend = g_canFrameCache.getNextCachedFrame(
				allowAllCursor,
				framePid,
				frameData);
		}

		// Snapshot cache specific PIDs mode
		if (requestedPidCount > 0) {
			RequestedPid nextPid;
			const bool hasNext = g_rcPidFilterState.nextPid(
				requestedPidCursor,
				nextPid,
				selectedRequestedPidIndex);

			if (hasNext) {
				hasFrameToSend = g_canFrameCache.getNextRequestedCachedFrame(
					nextPid,
					framePid,
					frameData);
			}
		}

		if (hasFrameToSend) {
			uint8_t payload[13];
			buildRcCanMainPayload(framePid, frameData, payload);

			g_rcBleMainChar->setValue(payload, sizeof(payload));
			g_rcBleMainChar->notify();
		}
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
	while (true) {
		if (!g_rcBleConnected || g_rcBleGpsMainChar == nullptr || g_rcBleGpsTimeChar == nullptr) {
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		if (!bleCanSendNotification()) {
			vTaskDelay(pdMS_TO_TICKS(1));
			continue;
		}
		
		GpsSnapshot snapshot{};
		bool hasNext = getGpsSnapshot(snapshot);
		
		if (!hasNext) {
			vTaskDelay(pdMS_TO_TICKS(1));
			continue;
		}
				
		uint8_t gpsTimePayload[3];
		buildRcGpsTimePayload(snapshot, gpsTimePayload);
		g_rcBleGpsTimeChar->setValue(gpsTimePayload, sizeof(gpsTimePayload));
		
		uint8_t gpsMainPayload[20];
		buildRcGpsMainPayload(snapshot, gpsMainPayload);
		g_rcBleGpsMainChar->setValue(gpsMainPayload, sizeof(gpsMainPayload));

		g_rcBleGpsMainChar->notify();
		// g_rcBleGpsTimeChar->notify();
	}
}
