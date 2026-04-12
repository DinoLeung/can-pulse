#include "gps_snapshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "gps.h"

static GpsSnapshot gpsSnapshot;
static SemaphoreHandle_t mutex;

/**
 * @brief Initializes the GPS snapshot subsystem.
 *
 * Creates the module mutex on first call so snapshot reads and writes can be
 * synchronized across tasks. Safe to call multiple times.
 */
void initGpsSnapshot() {
	if (mutex == nullptr) {
		mutex = xSemaphoreCreateMutex();
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
int8_t incrementDateSyncBits(int8_t syncBits) {
	return static_cast<uint8_t>((syncBits + 1) & 0b111);
}

/**
 * @brief Refreshes the cached GPS snapshot from the latest parser state.
 *
 * Reads currently available GPS fields, preserves prior values when fields are
 * not valid, updates RaceChrono sync state, and marks the snapshot pending when
 * time data has advanced so a new BLE notification can be sent.
 */
void updateGpsSnapshot() {
    if (mutex == nullptr)
		return;
	
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return;
	
    GpsSnapshot next = gpsSnapshot;

	const bool prevDateValid = gpsSnapshot.dateValid;
    const uint16_t prevYear = gpsSnapshot.year;
    const uint8_t prevMonth = gpsSnapshot.month;
    const uint8_t prevDay = gpsSnapshot.day;
	const bool prevTimeValid = gpsSnapshot.timeValid;
    const uint8_t prevHour = gpsSnapshot.hour;
    const uint8_t prevMinute = gpsSnapshot.minute;
    const uint8_t prevSecond = gpsSnapshot.second;
    const uint16_t prevMs = gpsSnapshot.milliseconds;
	
	next.timeValid = g_gps.time.isValid();
	if (next.timeValid) {
		next.hour = g_gps.time.hour();
		next.minute = g_gps.time.minute();
		next.second = g_gps.time.second();
		next.milliseconds = g_gps.time.centisecond() * 10;
	}
	
	next.dateValid = g_gps.date.isValid();
	if (next.dateValid) {
		next.year = g_gps.date.year();
		next.month = g_gps.date.month();
		next.day = g_gps.date.day();
	}

	next.locationValid = g_gps.location.isValid();
	if (next.locationValid) {
		next.latitudeDeg = g_gps.location.lat();
		next.longitudeDeg = g_gps.location.lng();
	}

	if (next.locationValid) {
		next.fixQuality = g_gps.location.FixQuality();
	}

	next.altitudeValid = g_gps.altitude.isValid();
	if (next.altitudeValid) {
		next.altitudeMeters = g_gps.altitude.meters();
	}

	next.speedValid = g_gps.speed.isValid();
	if (next.speedValid) {
		next.speedKmh = g_gps.speed.kmph();
	}

	next.courseValid = g_gps.course.isValid();
	if (next.courseValid) {
		next.courseDeg = g_gps.course.deg();
	}

	next.satellitesValid = g_gps.satellites.isValid();
	if (next.satellitesValid) {
		next.satellites = g_gps.satellites.value();
	}

	next.hdopValid = g_gps.hdop.isValid();
	if (next.hdopValid) {
		next.hdop = g_gps.hdop.hdop();
	}

	float vdop = 0.0f;
	if (parseVdop(vdop)) {
		next.vdopValid = true;
		next.vdop = vdop;
	}

	// Sync byte increments when time coarser than minute
	// https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#gps-time-characteristic-uuid-0x0004
	if (next.timeValid && next.dateValid &&
		(!prevTimeValid || !prevDateValid ||
		next.hour != prevHour ||
		next.day != prevDay ||
		next.month != prevMonth ||
		next.year != prevYear)) {
			next.dateSyncBits = incrementDateSyncBits(next.dateSyncBits);
		}

	next.pendingNotify =
        next.timeValid &&
        (!prevTimeValid ||
         next.hour != prevHour ||
         next.minute != prevMinute ||
         next.second != prevSecond ||
         next.milliseconds != prevMs);
	
	gpsSnapshot = next;
	xSemaphoreGive(mutex);
}

/**
 * @brief Retrieves the latest pending GPS snapshot.
 *
 * Returns a copy of the cached snapshot only when new time-based data is marked
 * pending. Once returned, the pending flag is cleared so the same snapshot is
 * not sent repeatedly.
 *
 * @param out Receives the latest snapshot when available.
 * @return true when a new snapshot was returned.
 * @return false when no new snapshot is pending or the subsystem is not ready.
 */
bool getGpsSnapshot(GpsSnapshot& out) {
	if (mutex == nullptr)
		return false;

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return false;

	bool updated = false;

	if (gpsSnapshot.pendingNotify == true) {
		out = gpsSnapshot;
		updated = true;
		gpsSnapshot.pendingNotify = false;
	}
		
	xSemaphoreGive(mutex);
	return updated;
}