#include "rc_ble_helper.h"
#include <cstdlib>

#include "gps_helper.h"

/**
 * @brief Read a 16-bit big-endian unsigned integer from a byte buffer.
 *
 * Interprets the first two bytes at the given pointer as a big-endian
 * encoded 16-bit value (network byte order) and converts it to host order.
 *
 * @param data Pointer to at least 2 bytes of data.
 * @return Decoded 16-bit unsigned integer.
 */
uint16_t readBe16(const uint8_t* data) {
	return (static_cast<uint16_t>(data[0]) << 8) |
	       static_cast<uint16_t>(data[1]);
}

/**
 * @brief Write a 16-bit unsigned integer in big-endian byte order.
 */
void writeBe16(const uint16_t value, uint8_t* output) {
	output[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
	output[1] = static_cast<uint8_t>(value & 0xFF);
}

/**
 * @brief Write a 24-bit unsigned integer in big-endian byte order.
 */
void writeBe24(const uint32_t value, uint8_t* output) {
	output[0] = static_cast<uint8_t>((value >> 16) & 0xFF);
	output[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
	output[2] = static_cast<uint8_t>(value & 0xFF);
}

/**
 * @brief Read a 32-bit big-endian unsigned integer from a byte buffer.
 *
 * Interprets the first four bytes at the given pointer as a big-endian
 * encoded 32-bit value (network byte order) and converts it to host order.
 *
 * @param data Pointer to at least 4 bytes of data.
 * @return Decoded 32-bit unsigned integer.
 */
uint32_t readBe32(const uint8_t* data) {
	return (static_cast<uint32_t>(data[0]) << 24) |
	       (static_cast<uint32_t>(data[1]) << 16) |
	       (static_cast<uint32_t>(data[2]) << 8) |
	       static_cast<uint32_t>(data[3]);
}

/**
 * @brief Write a 32-bit signed integer in big-endian byte order.
 */
void writeBe32(const int32_t value, uint8_t* output) {
	const uint32_t raw = static_cast<uint32_t>(value);
	output[0] = static_cast<uint8_t>((raw >> 24) & 0xFF);
	output[1] = static_cast<uint8_t>((raw >> 16) & 0xFF);
	output[2] = static_cast<uint8_t>((raw >> 8) & 0xFF);
	output[3] = static_cast<uint8_t>(raw & 0xFF);
}

/**
 * @brief Parse a RaceChrono BLE filter request payload.
 *
 * Decodes a raw byte payload received from the RaceChrono client into a
 * structured `RcFilterRequest`. The payload format is command-based:
 *
 * - Command 0 (DenyAll):
 *   - Length: 1 byte
 *   - Disables all streaming.
 *
 * - Command 1 (AllowAll):
 *   - Length: 3 bytes
 *   - [1..2]: 16-bit big-endian interval in milliseconds
 *   - Enables streaming of all cached frames at the given interval.
 *
 * - Command 2 (AllowOnePid):
 *   - Length: 7 bytes
 *   - [1..2]: 16-bit big-endian interval in milliseconds
 *   - [3..6]: 32-bit big-endian CAN identifier (PID)
 *   - Requests streaming of a specific PID at the given interval.
 *
 * Invalid payload lengths or unknown command values will result in failure.
 *
 * @param value Raw payload received over BLE (binary string).
 * @param out   Output structure populated on successful parse.
 * @return true if parsing succeeded, false otherwise.
 */
bool parseFilterRequest(const std::string& value, RcFilterRequest& out) {
	if (value.empty()) {
		return false;
	}

	const uint8_t* data = reinterpret_cast<const uint8_t*>(value.data());
	const size_t len = value.length();
	const uint8_t command = data[0];

	switch (command) {
	case 0: // deny all
		if (len != 1) return false;
		out.command = RcFilterCommand::DenyAll;
		out.intervalMs = 0;
		out.pid = 0;
		return true;

	case 1: // allow all
		if (len != 3) return false;
		out.command = RcFilterCommand::AllowAll;
		out.intervalMs = readBe16(&data[1]);
		out.pid = 0;
		return true;

	case 2: // allow one pid
		if (len != 7) return false;
		out.command = RcFilterCommand::AllowOnePid;
		out.intervalMs = readBe16(&data[1]);
		out.pid = readBe32(&data[3]);
		return true;

	default:
		return false;
	}
}

/**
 * @brief Builds the 13-byte RaceChrono CAN main characteristic payload.
 *
 * Layout:
 * - bytes 0..3 : 32-bit CAN identifier (PID), little-endian
 * - bytes 4..11: 8-byte CAN data payload (DLC assumed 8)
 * - byte  12   : reserved (zeroed)
 *
 * This function simply packages a raw CAN frame into the format expected by
 * RaceChrono. No interpretation of the CAN payload is performed.
 *
 * @param framePid 32-bit CAN identifier (standard or extended already resolved).
 * @param frameData Pointer to 8 bytes of CAN frame data.
 * @param outPayload Output buffer of at least 13 bytes.
 */
void buildRcCanMainPayload(uint32_t framePid, const uint8_t* frameData, uint8_t* outPayload) {
	memset(outPayload, 0, 13);

	// 4-byte CAN identifier, little-endian
	outPayload[0] = static_cast<uint8_t>(framePid & 0xFF);
	outPayload[1] = static_cast<uint8_t>((framePid >> 8) & 0xFF);
	outPayload[2] = static_cast<uint8_t>((framePid >> 16) & 0xFF);
	outPayload[3] = static_cast<uint8_t>((framePid >> 24) & 0xFF);

	// 8 data bytes
	memcpy(&outPayload[4], frameData, 8);
}

/**
 * @brief Builds the 20-byte RaceChrono GPS main characteristic payload.
 * 
 * Layout:
 * - bytes 0..2 : Sync bits (3 bits) + time-from-hour (21 bits)
 *                timeFromHour = minute * 30000 + second * 500 + (millis / 2)
 * - byte  3    : Fix quality (2 bits) + satellites (6 bits, 0x3F = invalid)
 * - bytes 4..7 : Latitude  (deg * 1e7),  signed 32-bit, 0x7FFFFFFF = invalid
 * - bytes 8..11: Longitude (deg * 1e7),  signed 32-bit, 0x7FFFFFFF = invalid
 * - bytes 12..13: Altitude (dual-mode, 0xFFFF = invalid)
 * - bytes 14..15: Speed    (dual-mode, 0xFFFF = invalid)
 * - bytes 16..17: Bearing  (deg * 100), 0xFFFF = invalid
 * - byte  18   : HDOP (dop * 10), 0xFF = invalid
 * - byte  19   : VDOP (dop * 10), 0xFF = invalid
 *
 * Notes:
 * - Dual-mode fields (altitude, speed) use a high-precision encoding when
 *   values fit in 15 bits; otherwise a coarse encoding is used with the MSB set.
 * - All fields are validated; invalid or non-finite inputs are encoded using
 *   the protocol-defined sentinel values.
 *
 * @param syncBits 3-bit synchronization counter (0..7).
 * @param gps Snapshot of GPS data.
 * @param outPayload Output buffer of at least 20 bytes.
 */
void buildRcGpsMainPayload(const GpsSnapshot& gps, uint8_t* outPayload) {
	memset(outPayload, 0, 20);

	// Byte 0-2: Sync bits* (3 bits) and time from hour start (21 bits = (minute * 30000) + (seconds * 500) + (milliseconds / 2))
	uint32_t timeFromHour = 0;

	if (gps.timeValid && gps.minute < 60 && gps.second < 60) {
		const uint32_t minute = gps.minute;
		const uint32_t second = gps.second;
		const uint32_t millis = std::min<uint32_t>(gps.milliseconds, 999U);

		timeFromHour = minute * 30000U + second * 500U + (millis / 2U);
	}

	const uint32_t timeField =
		((static_cast<uint32_t>(gps.dateSyncBits) & 0b111) << 21) |
		(timeFromHour & 0x1FFFFF);

	writeBe24(timeField, outPayload);
	
	// Byte 3: Fix quality (2 bits), locked satellites (6 bits, invalid value 0x3F)
	uint8_t fixQuality = 0;
	uint8_t satellitesField = 0x3F;
	if (gps.locationValid) {
		fixQuality = encodeRcFixQuality(gps.fixQuality);
	}
	if (gps.satellitesValid) {
		satellitesField = std::min<uint8_t>(gps.satellites, 0b111111);
	}
	outPayload[3] = static_cast<uint8_t>(((fixQuality & 0b11) << 6) | (satellitesField & 0x3F));

	// Byte 4-7: Latitude in (degrees * 10_000_000), signed 2's complement, invalid value 0x7FFFFFFF
	// Byte 8-11: Longitude in (degrees * 10_000_000), signed 2's complement, invalid value 0x7FFFFFFF
	int32_t lat = 0x7FFFFFFF;
	int32_t lon = 0x7FFFFFFF;
	if (gps.locationValid &&
	    std::isfinite(gps.latitudeDeg) &&
	    std::isfinite(gps.longitudeDeg)) {
		const double scale = 10000000.0;
		lat = static_cast<int32_t>(lround(gps.latitudeDeg * scale));
		lon = static_cast<int32_t>(lround(gps.longitudeDeg * scale));
	}
	writeBe32(lat, &outPayload[4]);
	writeBe32(lon, &outPayload[8]);

	// Byte 12-13: Altitude (((meters + 500) * 10) & 0x7FFF) or (((meters + 500) & 0x7FFF) | 0x8000), invalid value 0xFFFF
	uint16_t altitudeField = 0xFFFF;
	if (gps.altitudeValid && std::isfinite(gps.altitudeMeters)) {
		altitudeField = encodeRcGpsAltitude(gps.altitudeMeters);
	}
	writeBe16(altitudeField, &outPayload[12]);

	// Byte 14-15: Speed in ((km/h * 100) & 0x7FFF) or (((km/h * 10) & 0x7FFF) | 0x8000), invalid value 0xFFFF
	uint16_t speedField = 0xFFFF;
	if (gps.speedValid && std::isfinite(gps.speedKmh) && gps.speedKmh >= 0.0) {
		speedField = encodeRcGpsSpeed(gps.speedKmh);
	}
	writeBe16(speedField, &outPayload[14]);

	// Byte 16-17: Bearing (degrees * 100), invalid value 0xFFFF
	uint16_t bearingField = 0xFFFF;
	if (gps.courseValid) {
		bearingField = encodeRcGpsBearing(gps.courseDeg);
	}
	writeBe16(bearingField, &outPayload[16]);

	// Byte 18: HDOP (dop * 10), invalid value 0xFF
	uint8_t hdopField = 0xFF;
	if (gps.hdopValid) {
		hdopField = encodeRcGpsDop(gps.hdop);
	}
	outPayload[18] = hdopField;

	// Byte 19: VDOP (dop * 10), invalid value 0xFF
	uint8_t vdopField = 0xFF;
	if (gps.vdopValid) {
		vdopField = encodeRcGpsDop(gps.vdop);
	}
	outPayload[19] = vdopField;
}

/**
 * @brief Builds the 3-byte RaceChrono GPS time characteristic payload.
 *
 * Layout (24 bits total):
 * - bits 23..21: sync bits (3 bits)
 * - bits 20..0 : hour-and-date field (21 bits)
 *
 * Encoding of hour-and-date field:
 * ((year - 2000) * 8928) + ((month - 1) * 744) + ((day - 1) * 24) + hour
*
 * Notes:
 * - Uses a fixed 31-day/month calendar for compactness; this is a monotonic
 *   encoding, not a true calendar representation.
 * - If date/time is invalid or out of range, only sync bits are emitted.
 * - Output is written in big-endian byte order.
 *
 * @param syncBits 3-bit synchronization counter (0..7).
 * @param gps Snapshot containing date/time fields.
 * @param outPayload Output buffer of at least 3 bytes.
 */
void buildRcGpsTimePayload(const GpsSnapshot& gps, uint8_t* outPayload) {
	memset(outPayload, 0, 3);

	const uint32_t syncField = (static_cast<uint32_t>(gps.dateSyncBits) & 0b111U) << 21;

	if (!gps.dateValid || !gps.timeValid) {
		writeBe24(syncField, outPayload);
		return;
	}

	if (gps.year < 2000 || gps.month < 1 || gps.month > 12 ||
	    gps.day < 1 || gps.day > 31 || gps.hour > 23) {
		writeBe24(syncField, outPayload);
		return;
	}

	const uint32_t year = static_cast<uint32_t>(gps.year - 2000);
	const uint32_t month = static_cast<uint32_t>(gps.month - 1);
	const uint32_t day = static_cast<uint32_t>(gps.day - 1);
	const uint32_t hour = static_cast<uint32_t>(gps.hour);
	const uint32_t hourAndDate = year * 8928U + month * 744U + day * 24U + hour;

	const uint32_t timeField = syncField | (hourAndDate & 0x1FFFFF);
	writeBe24(timeField, outPayload);
}