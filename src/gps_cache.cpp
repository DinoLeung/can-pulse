#include "gps_cache.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "gps.h"
#include "esp_log.h"

static const char *TAG = "gps_cache";

GpsCacheStore g_gpsCacheStore;

/**
 * @brief Initializes the GPS cache subsystem.
 *
 * Creates the module mutex on first call so cache reads and writes can be
 * synchronized across tasks. Safe to call multiple times.
 */
void initGpsCache() {
	if (g_gpsCacheStore.mutex == nullptr) {
		g_gpsCacheStore.mutex = xSemaphoreCreateMutex();
	}
	ESP_LOGI(TAG, "GPS cache ready");
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
 * @brief Refreshes the cached GPS cache from the latest parser state.
 *
 * Reads currently available GPS fields, preserves prior values when fields are
 * not valid, updates RaceChrono sync state, and marks the cache pending when
 * time data has advanced so a new BLE notification can be sent.
 */
void GpsCacheStore::update() {
    if (mutex == nullptr)
		return;
	
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return;

	const bool prevDateValid = cache.dateValid;
    const uint16_t prevYear = cache.year;
    const uint8_t prevMonth = cache.month;
    const uint8_t prevDay = cache.day;
	const bool prevTimeValid = cache.timeValid;
    const uint8_t prevHour = cache.hour;
    const uint8_t prevMinute = cache.minute;
    const uint8_t prevSecond = cache.second;
    const uint16_t prevMs = cache.milliseconds;
	
	cache.timeValid = g_gps.time.isValid();
	if (cache.timeValid) {
		cache.hour = g_gps.time.hour();
		cache.minute = g_gps.time.minute();
		cache.second = g_gps.time.second();
		cache.milliseconds = g_gps.time.centisecond() * 10;
	}
	
	cache.dateValid = g_gps.date.isValid();
	if (cache.dateValid) {
		cache.year = g_gps.date.year();
		cache.month = g_gps.date.month();
		cache.day = g_gps.date.day();
	}

	cache.locationValid = g_gps.location.isValid();
	if (cache.locationValid) {
		cache.latitudeDeg = g_gps.location.lat();
		cache.longitudeDeg = g_gps.location.lng();
	}

	if (cache.locationValid) {
		cache.fixQuality = g_gps.location.FixQuality();
	}

	cache.altitudeValid = g_gps.altitude.isValid();
	if (cache.altitudeValid) {
		cache.altitudeMeters = g_gps.altitude.meters();
	}

	cache.speedValid = g_gps.speed.isValid();
	if (cache.speedValid) {
		cache.speedKmh = g_gps.speed.kmph();
	}

	cache.courseValid = g_gps.course.isValid();
	if (cache.courseValid) {
		cache.courseDeg = g_gps.course.deg();
	}

	cache.satellitesValid = g_gps.satellites.isValid();
	if (cache.satellitesValid) {
		cache.satellites = g_gps.satellites.value();
	}

	cache.hdopValid = g_gps.hdop.isValid();
	if (cache.hdopValid) {
		cache.hdop = g_gps.hdop.hdop();
	}

	float vdop = 0.0f;
	if (parseVdop(vdop)) {
		cache.vdopValid = true;
		cache.vdop = vdop;
	}

	// Sync byte increments when time coarser than minute
	// https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#gps-time-characteristic-uuid-0x0004
	if (cache.timeValid && cache.dateValid &&
		(!prevTimeValid || !prevDateValid ||
		cache.hour != prevHour ||
		cache.day != prevDay ||
		cache.month != prevMonth ||
		cache.year != prevYear)) {
			syncBits = incrementSyncBits(syncBits);
		}

	pendingNotify =
        cache.timeValid &&
        (!prevTimeValid ||
         cache.hour != prevHour ||
         cache.minute != prevMinute ||
         cache.second != prevSecond ||
         cache.milliseconds != prevMs);
	
	xSemaphoreGive(mutex);
}

/**
 * @brief Retrieves the latest pending GPS cache and RaceChrono sync state.
 *
 * Returns a copy of the cached cache only when new time-based data is marked
 * pending. When successful, the current date sync bits are also returned for
 * GPS time characteristic encoding. Once returned, the pending flag is cleared
 * so the same cache is not sent repeatedly.
 *
 * @param o_cache Receives the latest cache when available.
 * @param o_syncBits Receives the current 3-bit RaceChrono date sync counter.
 * @return true when a new cache was returned.
 * @return false when no new cache is pending or the store is not ready.
 */
bool GpsCacheStore::get(GpsCache& o_cache, int8_t& o_syncBits) {
	if (mutex == nullptr)
		return false;

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return false;

	bool updated = false;

	if (pendingNotify == true) {
		o_cache = cache;
		o_syncBits = syncBits;
		updated = true;
		pendingNotify = false;
	}
		
	xSemaphoreGive(mutex);
	return updated;
}