#include "gps_snapshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "gps.h"

GpsSnapshotStore g_gpsSnapshotStore;

/**
 * @brief Initializes the GPS snapshot subsystem.
 *
 * Creates the module mutex on first call so snapshot reads and writes can be
 * synchronized across tasks. Safe to call multiple times.
 */
void initGpsSnapshot() {
	if (g_gpsSnapshotStore.mutex == nullptr) {
		g_gpsSnapshotStore.mutex = xSemaphoreCreateMutex();
	}
}

/**
 * @brief Parses the current VDOP value from the GPS custom field.
 *
 * Validates that a VDOP string is present, converts it to float, and rejects
 * empty, infinite, or negative values.
 *
 * @param out Receives the parsed VDOP when successful.
 * @return true when a valid VDOP value was parsed.
 * @return false when no valid VDOP value is available.
 */
bool parseVdop(float& out) {
	if (!g_vdop.isValid()) {
		return false;
	}

	const char* raw = g_vdop.value();
	if (raw == nullptr || raw[0] == '\0') {
		return false;
	}

	const float value = atof(raw);
	if (!std::isfinite(value) || value < 0.0f) {
		return false;
	}

	out = value;
	return true;
}

/**
 * @brief Advances the RaceChrono GPS date sync counter.
 *
 * The counter is a 3-bit rolling field used by the GPS time characteristic to
 * signal coarse date or hour changes.
 *
 * @param syncBits Current sync counter value.
 * @return Next counter value wrapped to 3 bits.
 */
int8_t incrementSyncBits(int8_t syncBits) {
	return static_cast<uint8_t>((syncBits + 1) & 0b111);
}

/**
 * @brief Refreshes the cached GPS snapshot from the latest parser state.
 *
 * Reads currently available GPS fields, preserves prior values when fields are
 * not valid, updates RaceChrono sync state, and marks the snapshot pending when
 * time data has advanced so a new BLE notification can be sent.
 */
void GpsSnapshotStore::update() {
    if (mutex == nullptr)
		return;
	
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return;

	const bool prevDateValid = snapshot.dateValid;
    const uint16_t prevYear = snapshot.year;
    const uint8_t prevMonth = snapshot.month;
    const uint8_t prevDay = snapshot.day;
	const bool prevTimeValid = snapshot.timeValid;
    const uint8_t prevHour = snapshot.hour;
    const uint8_t prevMinute = snapshot.minute;
    const uint8_t prevSecond = snapshot.second;
    const uint16_t prevMs = snapshot.milliseconds;
	
	snapshot.timeValid = g_gps.time.isValid();
	if (snapshot.timeValid) {
		snapshot.hour = g_gps.time.hour();
		snapshot.minute = g_gps.time.minute();
		snapshot.second = g_gps.time.second();
		snapshot.milliseconds = g_gps.time.centisecond() * 10;
	}
	
	snapshot.dateValid = g_gps.date.isValid();
	if (snapshot.dateValid) {
		snapshot.year = g_gps.date.year();
		snapshot.month = g_gps.date.month();
		snapshot.day = g_gps.date.day();
	}

	snapshot.locationValid = g_gps.location.isValid();
	if (snapshot.locationValid) {
		snapshot.latitudeDeg = g_gps.location.lat();
		snapshot.longitudeDeg = g_gps.location.lng();
	}

	if (snapshot.locationValid) {
		snapshot.fixQuality = g_gps.location.FixQuality();
	}

	snapshot.altitudeValid = g_gps.altitude.isValid();
	if (snapshot.altitudeValid) {
		snapshot.altitudeMeters = g_gps.altitude.meters();
	}

	snapshot.speedValid = g_gps.speed.isValid();
	if (snapshot.speedValid) {
		snapshot.speedKmh = g_gps.speed.kmph();
	}

	snapshot.courseValid = g_gps.course.isValid();
	if (snapshot.courseValid) {
		snapshot.courseDeg = g_gps.course.deg();
	}

	snapshot.satellitesValid = g_gps.satellites.isValid();
	if (snapshot.satellitesValid) {
		snapshot.satellites = g_gps.satellites.value();
	}

	snapshot.hdopValid = g_gps.hdop.isValid();
	if (snapshot.hdopValid) {
		snapshot.hdop = g_gps.hdop.hdop();
	}

	float vdop = 0.0f;
	if (parseVdop(vdop)) {
		snapshot.vdopValid = true;
		snapshot.vdop = vdop;
	}

	// Sync byte increments when time coarser than minute
	// https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#gps-time-characteristic-uuid-0x0004
	if (snapshot.timeValid && snapshot.dateValid &&
		(!prevTimeValid || !prevDateValid ||
		snapshot.hour != prevHour ||
		snapshot.day != prevDay ||
		snapshot.month != prevMonth ||
		snapshot.year != prevYear)) {
			syncBits = incrementSyncBits(syncBits);
		}

	pendingNotify =
        snapshot.timeValid &&
        (!prevTimeValid ||
         snapshot.hour != prevHour ||
         snapshot.minute != prevMinute ||
         snapshot.second != prevSecond ||
         snapshot.milliseconds != prevMs);
	
	xSemaphoreGive(mutex);
}

/**
 * @brief Retrieves the latest pending GPS snapshot and RaceChrono sync state.
 *
 * Returns a copy of the cached snapshot only when new time-based data is marked
 * pending. When successful, the current date sync bits are also returned for
 * GPS time characteristic encoding. Once returned, the pending flag is cleared
 * so the same snapshot is not sent repeatedly.
 *
 * @param out_snapshot Receives the latest snapshot when available.
 * @param out_syncBits Receives the current 3-bit RaceChrono date sync counter.
 * @return true when a new snapshot was returned.
 * @return false when no new snapshot is pending or the store is not ready.
 */
bool GpsSnapshotStore::get(GpsSnapshot& o_snapshot, int8_t& o_syncBits) {
	if (mutex == nullptr)
		return false;

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return false;

	bool updated = false;

	if (pendingNotify == true) {
		o_snapshot = snapshot;
		o_syncBits = syncBits;
		updated = true;
		pendingNotify = false;
	}
		
	xSemaphoreGive(mutex);
	return updated;
}