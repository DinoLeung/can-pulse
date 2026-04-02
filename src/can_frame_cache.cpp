#include "can_frame_cache.h"
#include <Arduino.h>

/**
 * @brief Global CAN frame cache shared across tasks.
 *
 * This object holds all cached CAN frames and the mutex used to protect
 * concurrent access.
 */
CanFrameCache g_canFrameCache;

/**
 * @brief Find an existing cache entry by CAN identifier and frame format.
 *
 * Performs a linear search through the active portion of the cache and returns
 * the first valid entry whose identifier and standard/extended flag match the
 * requested key.
 *
 * @param identifier CAN identifier to search for.
 * @param isExtended True for extended 29-bit frames, false for standard 11-bit frames.
 * @return Pointer to the matching cache entry, or nullptr if no entry exists.
 */
static CanFrameCacheEntry* findCacheEntry(uint32_t identifier, bool isExtended) {
	for (size_t i = 0; i < g_canFrameCache.count; i++) {
		CanFrameCacheEntry& entry = g_canFrameCache.entries[i];
		if (entry.valid && entry.identifier == identifier && entry.isExtended == isExtended) {
			return &entry;
		}
	}
	return nullptr;
}

/**
 * @brief Initialize the global CAN frame cache.
 *
 * Creates the cache mutex on first use, resets the entry count, and clears all
 * cache slots back to an empty state.
 */
void initCanFrameCache() {
	if (g_canFrameCache.mutex == nullptr) {
		g_canFrameCache.mutex = xSemaphoreCreateMutex();
	}
	g_canFrameCache.count = 0;
	for (size_t i = 0; i < kMaxCanFrameCacheSize; i++) {
		g_canFrameCache.entries[i].valid = false;
		g_canFrameCache.entries[i].lastUpdatedMs = 0;
		g_canFrameCache.entries[i].dlc = 0;
		g_canFrameCache.entries[i].identifier = 0;
		g_canFrameCache.entries[i].isExtended = false;
		memset(g_canFrameCache.entries[i].data, 0, sizeof(g_canFrameCache.entries[i].data));
	}
}

/**
 * @brief Insert or update a cached CAN frame.
 *
 * Looks up an existing cache entry for the given CAN identifier and frame
 * format. If none exists, a new entry is allocated from the next free slot.
 * The payload bytes, DLC, and last-seen timestamp are then updated while the
 * cache mutex is held.
 *
 * @param identifier CAN identifier of the received frame.
 * @param isExtended True for extended 29-bit frames, false for standard 11-bit frames.
 * @param dlc Data length code of the received frame. Values greater than 8 are clamped to 8.
 * @param data Pointer to the frame payload bytes. May be nullptr when dlc is 0.
 * @return true if the cache entry was updated successfully, or false if the mutex could not be taken or the cache is full.
 */
bool updateCanFrameCache(uint32_t identifier, bool isExtended, uint8_t dlc, const uint8_t* data) {
	if (xSemaphoreTake(g_canFrameCache.mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	CanFrameCacheEntry* entry = findCacheEntry(identifier, isExtended);
	if (entry == nullptr) {
		if (g_canFrameCache.count >= kMaxCanFrameCacheSize) {
			xSemaphoreGive(g_canFrameCache.mutex);
			return false;
		}
		entry = &g_canFrameCache.entries[g_canFrameCache.count++];
		entry->identifier = identifier;
		entry->isExtended = isExtended;
	}

	entry->dlc = dlc > 8 ? 8 : dlc;
	memset(entry->data, 0, sizeof(entry->data));
	if (data != nullptr && entry->dlc > 0) {
		memcpy(entry->data, data, entry->dlc);
	}
	entry->lastUpdatedMs = millis();
	entry->valid = true;
	entry->pendingNotify = true;

	xSemaphoreGive(g_canFrameCache.mutex);
	return true;
}

/**
 * @brief Claim the next pending cached CAN frame in allow-all mode.
 *
 * Iterates through the cache in a round-robin fashion using the provided
 * cursor and selects the next entry that is both valid and marked
 * `pendingNotify`.
 *
 * On selection, the frame data is copied to the output parameters and the
 * entry's `pendingNotify` flag is cleared before returning. This implements a
 * best-effort "claim on selection" model (latest-value semantics).
 *
 * The cursor is advanced to ensure fair traversal across all cached entries.
 *
 * The operation is protected by the cache mutex to ensure safe concurrent
 * access from multiple tasks. The mutex is held only for the duration of
 * selection and copy.
 *
 * @note If the subsequent BLE notify fails, the update may be lost because the
 *       entry has already been claimed (pending flag cleared). This is
 *       intentional for freshness-first behavior.
 *
 * @param cursor          In/out cursor used for round-robin traversal.
 * @param outFramePid     Output CAN identifier of the selected frame.
 * @param outFrameData    Output buffer (8 bytes) containing frame payload.
 * @return true if a pending frame was found and claimed, false otherwise.
 */
bool CanFrameCache::getNextCachedFrame(
	size_t& cursor,
	uint32_t& outFramePid,
	uint8_t (&outFrameData)[8]) {
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	bool found = false;

	if (count > 0) {
		if (cursor >= count) {
			cursor = 0;
		}

		for (size_t checked = 0; checked < count; ++checked) {
			const size_t index = (cursor + checked) % count;
			if (!entries[index].valid || !entries[index].pendingNotify)
				continue;

			outFramePid = entries[index].identifier;
			memcpy(outFrameData, entries[index].data, 8);
			// mark cache entry seen
			entries[index].pendingNotify = false;
			cursor = (index + 1) % count;
			found = true;
			break;
		}
	}

	xSemaphoreGive(mutex);
	return found;
}

/**
 * @brief Claim the cached CAN frame for a requested PID.
 *
 * Searches the cache for a valid entry matching the requested PID that is
 * marked `pendingNotify`. If found, copies the identifier and payload into
 * the output parameters and clears the entry's `pendingNotify` flag.
 *
 * Scheduling (e.g., round-robin or interval selection) is handled by the
 * caller (e.g., PidFilterState). This function performs lookup + claim only.
 *
 * The operation is protected by the cache mutex to ensure safe concurrent
 * access from multiple tasks.
 *
 * @note Uses best-effort "claim on selection" semantics. If the subsequent
 *       BLE notify fails, the update may be dropped.
 *
 * @param requestedPid   Requested PID to look up in the cache.
 * @param outFramePid    Output CAN identifier of the selected frame.
 * @param outFrameData   Output buffer (8 bytes) containing frame payload.
 * @return true if a matching pending frame was found and claimed, false otherwise.
 */
bool CanFrameCache::getNextRequestedCachedFrame(
	const RequestedPid& requestedPid,
	uint32_t& outFramePid,
	uint8_t (&outFrameData)[8]) {
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	bool found = false;

	for (size_t cacheIndex = 0; cacheIndex < count; ++cacheIndex) {
		if (!entries[cacheIndex].valid) continue;

		if (entries[cacheIndex].identifier != requestedPid.pid || !entries[cacheIndex].pendingNotify)
			continue;

		outFramePid = entries[cacheIndex].identifier;
		memcpy(outFrameData, entries[cacheIndex].data, 8);
		// mark cache entry seen
		entries[cacheIndex].pendingNotify = false;
		found = true;
		break;
	}

	xSemaphoreGive(mutex);
	return found;
}