#pragma once

#include <stdint.h>
#include <TinyGPSPlus.h>

struct GpsCache {
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

struct GpsCacheStore {
	GpsCache cache;
	SemaphoreHandle_t mutex;
	int8_t syncBits = 0;
	bool pendingNotify;
	
	void update();
	bool get(GpsCache& o_cache, int8_t& o_syncBits);
};

extern GpsCacheStore g_gpsCacheStore;

void initGpsCache();
