#pragma once

#include <cstdlib>
#include "gps_cache.h"

enum class RcFilterCommand : uint8_t {
	DenyAll = 0,
	AllowAll = 1,
	AllowOnePid = 2,
};

struct RcFilterRequest {
	RcFilterCommand command;
	uint16_t intervalMs;
	uint32_t pid;
};

uint16_t readBe16(const uint8_t* data);
uint32_t readBe32(const uint8_t* data);

void writeBe16(const uint16_t value, uint8_t* output);
void writeBe24(const uint32_t value, uint8_t* output);
void writeBe32(const uint32_t value, uint8_t* output);

bool parseFilterRequest(const std::string& value, RcFilterRequest& out);
void buildRcCanMainPayload(uint32_t framePid, const uint8_t* frameData, uint8_t* outPayload);
void buildRcGpsMainPayload(const GpsCache& gps, const int8_t syncBits, uint8_t* outPayload);
void buildRcGpsTimePayload(const GpsCache& gps, const int8_t syncBits, uint8_t* outPayload);
