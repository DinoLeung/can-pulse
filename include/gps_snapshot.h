#pragma once

#include <stdint.h>
#include <TinyGPSPlus.h>

struct GpsSnapshot {
	bool timeValid;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t milliseconds;

	bool dateValid;
	uint16_t year;
	uint8_t month;
	uint8_t day;

	
	bool locationValid;
	double latitudeDeg;
	double longitudeDeg;
	TinyGPSLocation::Quality fixQuality;
	
	bool altitudeValid;
	double altitudeMeters;
	
	bool speedValid;
	double speedKmh;
	
	bool courseValid;
	double courseDeg;
	
	bool satellitesValid;
	uint8_t satellites;
	
	bool hdopValid;
	float hdop;
	
	bool vdopValid;
	float vdop;
	
};

struct GpsSnapshotStore {
	GpsSnapshot snapshot;
	SemaphoreHandle_t mutex;
	int8_t syncBits = 0;
	bool pendingNotify;
	
	void update();
	bool get(GpsSnapshot& o_snapshot, int8_t& o_syncBits);
};

extern GpsSnapshotStore g_gpsSnapshotStore;

void initGpsSnapshot();
