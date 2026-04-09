#include "gps_snapshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "gps.h"

static GpsSnapshot gpsSnapshot;
static SemaphoreHandle_t mutex;

void initGpsSnapshot() {
	if (mutex == nullptr) {
		mutex = xSemaphoreCreateMutex();
	}
}

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

int8_t incrementDateSyncBits(int8_t syncBits) {
	return static_cast<uint8_t>((syncBits + 1) & 0b111);
}

void updateGpsSnapshot() {
    if (mutex == nullptr)
		return;
	
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return;
	
    GpsSnapshot next = gpsSnapshot;

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
	if (next.hour != prevHour ||
		next.day != prevDay ||
		next.month != prevMonth ||
		next.year != prevYear) {
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