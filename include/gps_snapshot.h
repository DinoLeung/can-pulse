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

	int8_t dateSyncBits = 0;

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

	bool pendingNotify;
};

void initGpsSnapshot();
void updateGpsSnapshot();
bool getGpsSnapshot(GpsSnapshot& out);
