#include "gps_snapshot.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "gps.h"

static GpsSnapshot gpsSnapshot;
static SemaphoreHandle_t mutex = nullptr;

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

void updateGpsSnapshot() {
    if (mutex == nullptr)
		return;

	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
		return;

    GpsSnapshot next{};

	next.timeValid = g_gps.time.isValid();
	if (next.timeValid) {
		next.hour = g_gps.time.hour();
		next.minute = g_gps.time.minute();
		next.second = g_gps.time.second();
		next.milliseconds = g_gps.time.centisecond() * 10;

		next.pendingNotify =
			next.hour != gpsSnapshot.hour ||
			next.minute != gpsSnapshot.minute ||
			next.second != gpsSnapshot.second ||
			next.milliseconds != gpsSnapshot.milliseconds;
	}


	if (next.pendingNotify == true) {
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
		
		gpsSnapshot = next;
	}

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