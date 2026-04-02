#include "rc_ble.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include "esp_gatt_common_api.h"
#include <freertos/queue.h>
#include "rc_ble_helper.h"

static BLEService* createRaceChronoService();
static void createCanMainCharacteristic(BLEService* service);
static void createCanFilterCharacteristic(BLEService* service);
static void createGpsMainCharacteristic(BLEService* service);
static void createGpsTimeCharacteristic(BLEService* service);
static void startRaceChronoAdvertising();

PidFilterState g_rcPidFilterState;
QueueHandle_t g_rcPidFilterRequestQueue;

BLEServer* g_rcBleServer;
BLECharacteristic* g_rcBleMainChar;
BLECharacteristic* g_rcBleGpsMainChar;
BLECharacteristic* g_rcBleGpsTimeChar;
volatile bool g_rcBleConnected;
uint16_t g_rcBleConnId;

/**
 * @brief BLE server callbacks used to track RaceChrono connection state.
 *
 * This class hooks into the ESP32 BLE server lifecycle events. When the
 * RaceChrono app connects, `onConnect()` sets the global connection flag so
 * other tasks know a client is present. When the client disconnects,
 * `onDisconnect()` clears the flag and restarts advertising so the device
 * becomes discoverable again.
 */
class ServerCallbacks : public BLEServerCallbacks {
	public:
	void onConnect(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
		g_rcBleConnected = true;
		g_rcBleConnId = param->connect.conn_id;
		Serial.println("RaceChrono connected");
	}
	void onDisconnect(BLEServer* server) override {
		g_rcBleConnected = false;
		g_rcBleConnId = kInvalidBleConnId;
		Serial.println("RaceChrono disconnected");
		RcFilterRequest msg{RcFilterCommand::DenyAll, 0, 0};
		xQueueSend(g_rcPidFilterRequestQueue, &msg, 0);
		server->getAdvertising()->start();
	}
};

/**
 * @brief Callback handler for the RaceChrono CAN filter characteristic.
 *
 * RaceChrono writes commands to characteristic 0x0002 to control which CAN
 * frames the device should transmit over BLE. For now this implementation
 * only logs the write length, but later this will parse filter commands such
 * as "allow PID", "deny all", or "allow all" with a requested interval.
 */
class FilterCallbacks : public BLECharacteristicCallbacks {
	public:
	void onWrite(BLECharacteristic* characteristic) override {
		auto value = characteristic->getValue();

		if (value.empty()) return;

		RcFilterRequest msg{};
		if (!parseFilterRequest(value, msg)) {
			Serial.println("Invalid RaceChrono filter payload");
			return;
		}

		if (xQueueSend(g_rcPidFilterRequestQueue, &msg, 0) != pdTRUE) {
			Serial.println("RaceChrono filter queue full");
		}
	}
};

/**
 * @brief Initialise the RaceChrono BLE service and start advertising.
 *
 * This function sets up the ESP32 BLE stack, creates the RaceChrono GATT
 * service (UUID 0x1FF8), and registers the required characteristics:
 *
 * - 0x0001 : CAN main characteristic (READ + NOTIFY)
 * - 0x0002 : CAN filter characteristic (WRITE)
 * - 0x0003 : GPS main characteristic (READ + NOTIFY)
 * - 0x0004 : GPS time characteristic (READ + NOTIFY)
 *
 * After the service is started the device begins advertising so the
 * RaceChrono mobile app can discover and connect to it.
 *
 * @return true when BLE initialisation completes successfully.
 */
bool initRaceChronoBle() {
	BLEDevice::init(kDeviceName);

	g_rcBleServer = BLEDevice::createServer();
	g_rcBleServer->setCallbacks(new ServerCallbacks());

	// Default to deny all mode
	g_rcPidFilterState.mutex = xSemaphoreCreateMutex();
	if (g_rcPidFilterState.mutex == nullptr) {
		Serial.println("Failed to create RaceChrono filter mutex");
		return false;
	}
	g_rcPidFilterState.allowAll = false;
	g_rcPidFilterState.allowAllIntervalMs = 0;
	g_rcPidFilterState.requestedPidCount = 0;

	g_rcPidFilterRequestQueue = xQueueCreate(kFilterRequestQueueSize, sizeof(RcFilterRequest));

	auto* service = createRaceChronoService();
	createCanMainCharacteristic(service);
	createCanFilterCharacteristic(service);
	createGpsMainCharacteristic(service);
	createGpsTimeCharacteristic(service);

	service->start();
	startRaceChronoAdvertising();

	Serial.println("RaceChrono BLE ready");
	return true;
}

/**
 * @brief Check if a BLE notification can be sent immediately.
 *
 * This function verifies two conditions before allowing a notify operation:
 *
 * 1. A valid BLE connection is active.
 * 2. The BLE stack has available transmit credits (ACL packets).
 *
 * The ESP32 BLE controller limits how many packets can be in-flight at any
 * given time. `esp_ble_get_cur_sendable_packets_num()` exposes the remaining
 * transmit credits for the current connection. When this reaches zero,
 * additional notifications will fail or return errors.
 *
 * This function is intended to be used as a guard in high-frequency notify
 * tasks (e.g. CAN or GPS streaming) to prevent:
 * - BLE stack errors (e.g. "Unknown connection ID", L2CAP issues)
 * - Overrunning the controller transmit buffer
 *
 * @return true if a notification can be sent right now, false otherwise.
 */
bool bleCanSendNotification() {
    if (!g_rcBleConnected || g_rcBleConnId == kInvalidBleConnId) return false;
    const uint16_t sendablePackets = esp_ble_get_cur_sendable_packets_num(g_rcBleConnId);
    return sendablePackets > 0;
}

/**
 * @brief Create the primary RaceChrono BLE service.
 *
 * This helper allocates the GATT service that hosts the RaceChrono
 * characteristics used by the mobile app.
 *
 * @return Pointer to the created BLE service.
 */
static BLEService* createRaceChronoService() {
	return g_rcBleServer->createService(BLEUUID(kServiceUuid));
}

/**
 * @brief Create the RaceChrono CAN main characteristic.
 *
 * This characteristic is used for RaceChrono data notifications. It is
 * configured with READ and NOTIFY properties and initialised with a small
 * zeroed value so the attribute exists with known contents before the first
 * real payload is sent.
 *
 * @param service The RaceChrono BLE service that will own the characteristic.
 */
static void createCanMainCharacteristic(BLEService* service) {
	g_rcBleMainChar = service->createCharacteristic(
		BLEUUID(kCanMainCharUuid),
		BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
	g_rcBleMainChar->addDescriptor(new BLE2902());

	uint8_t initValue[4] = {0, 0, 0, 0};
	g_rcBleMainChar->setValue(initValue, sizeof(initValue));
}

/**
 * @brief Create the RaceChrono CAN filter characteristic.
 *
 * RaceChrono writes filter commands to this characteristic to describe which
 * CAN identifiers it wants to receive and at what interval.
 *
 * @param service The RaceChrono BLE service that will own the characteristic.
 */
static void createCanFilterCharacteristic(BLEService* service) {
	auto* filterChar = service->createCharacteristic(
		BLEUUID(kCanFilterCharUuid),
		BLECharacteristic::PROPERTY_WRITE);
	filterChar->setCallbacks(new FilterCallbacks());
}

/**
 * @brief Create the RaceChrono GPS main characteristic.
 *
 * Per the RaceChrono BLE DIY API, UUID 0x0003 exposes GPS position and motion
 * data. This characteristic must support READ and NOTIFY so the app can both
 * poll and receive pushed updates for the latest GPS sample.
 *
 * The payload is 20 bytes and contains sync bits and time-from-hour, fix
 * quality and satellite count, latitude, longitude, altitude, speed, bearing,
 * HDOP, and VDOP.
 *
 * @param service The RaceChrono BLE service that will own the characteristic.
 */
static void createGpsMainCharacteristic(BLEService* service) {
	g_rcBleGpsMainChar = service->createCharacteristic(
		BLEUUID(kGpsMainCharUuid),
		BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
	g_rcBleGpsMainChar->addDescriptor(new BLE2902());

	uint8_t initValue[20] = {
		0x00, 0x00, 0x00,
		0x3F,
		0x7F, 0xFF, 0xFF, 0xFF,
		0x7F, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF,
		0xFF, 0xFF,
		0xFF, 0xFF,
		0xFF,
		0xFF
	};
	g_rcBleGpsMainChar->setValue(initValue, sizeof(initValue));
}

/**
 * @brief Create the RaceChrono GPS time characteristic.
 *
 * Per the RaceChrono BLE DIY API, UUID 0x0004 exposes the GPS date and hour
 * packed together with the same 3-bit sync counter used by the GPS main
 * characteristic. This characteristic must support READ and NOTIFY.
 *
 * The payload is 3 bytes wide and is initialized to zero so the characteristic
 * exists with deterministic contents before valid GPS time is encoded.
 *
 * @param service The RaceChrono BLE service that will own the characteristic.
 */
static void createGpsTimeCharacteristic(BLEService* service) {
	g_rcBleGpsTimeChar = service->createCharacteristic(
		BLEUUID(kGpsTimeCharUuid),
		BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
	g_rcBleGpsTimeChar->addDescriptor(new BLE2902());

	uint8_t initValue[3] = {0x00, 0x00, 0x00};
	g_rcBleGpsTimeChar->setValue(initValue, sizeof(initValue));
}

/**
 * @brief Start BLE advertising for the RaceChrono service.
 *
 * This makes the device discoverable to the RaceChrono mobile app using the
 * service UUID configured for the DIY BLE device profile.
 */
static void startRaceChronoAdvertising() {
	auto* advertising = BLEDevice::getAdvertising();
	advertising->addServiceUUID(BLEUUID(kServiceUuid));
	BLEDevice::startAdvertising();
}

/**
 * @brief Select the next active requested PID using round-robin traversal.
 *
 * Iterates over the requested PID list starting from the current cursor
 * position and returns the first PID marked as active. The search wraps
 * around the list to ensure fair distribution across all requested PIDs.
 *
 * This function does not perform any timing or interval checks. It simply
 * selects the next active PID in a best-effort manner.
 *
 * The internal mutex is held during traversal to ensure a consistent view
 * of the requested PID list when accessed from multiple tasks.
 *
 * @param requestedPidCursor   In/out cursor for round-robin traversal. Will
 *                             be advanced to the next position after a PID
 *                             is selected.
 * @param outRequestedPid      Output copy of the selected requested PID.
 * @param outRequestedPidIndex Output index of the selected PID within the list.
 *
 * @return true if an active PID was found, false if none are active or the
 *         list is empty.
 */
bool PidFilterState::nextPid(
	size_t& requestedPidCursor,
	RequestedPid& outRequestedPid,
	size_t& outRequestedPidIndex) const {
	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		return false;
	}

	bool found = false;

	if (requestedPidCount > 0) {
		if (requestedPidCursor >= requestedPidCount) {
			requestedPidCursor = 0;
		}

		for (size_t checkedPid = 0; checkedPid < requestedPidCount; ++checkedPid) {
			const size_t pidIndex = (requestedPidCursor + checkedPid) % requestedPidCount;
			const RequestedPid& requested = requestedPids[pidIndex];

			if (!requested.active) {
				continue;
			}

			outRequestedPid = requested;
			outRequestedPidIndex = pidIndex;
			requestedPidCursor = (pidIndex + 1) % requestedPidCount;
			found = true;
			break;
		}
	}

	xSemaphoreGive(mutex);
	return found;
}

/**
 * @brief Take a thread-safe snapshot of the current filter configuration.
 *
 * This copies the current filter state (allow-all mode, interval, and
 * requested PID list) into caller-provided buffers. The copy is performed
 * under a mutex to ensure a consistent view of the state without requiring
 * the caller to hold the lock.
 *
 * This is typically used by the notify task to obtain a stable view of
 * filter configuration for a single iteration, avoiding prolonged lock
 * holding while processing or transmitting data.
 *
 * @param outAllowAll            Output flag indicating allow-all mode.
 * @param outAllowAllIntervalMs  Output interval for allow-all mode.
 * @param outRequestedPids       Output array to receive requested PID entries.
 * @param outRequestedPidCount   Output number of valid requested PIDs.
 */
void PidFilterState::snapshot(
	bool& outAllowAll,
	uint16_t& outAllowAllIntervalMs,
	RequestedPid (&outRequestedPids)[kMaxRequestedPids],
	size_t& outRequestedPidCount) const {
	outAllowAll = false;
	outAllowAllIntervalMs = 0;
	outRequestedPidCount = 0;

	if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
		return;
	}

	outAllowAll = allowAll;
	outAllowAllIntervalMs = allowAllIntervalMs;
	outRequestedPidCount = requestedPidCount;
	if (outRequestedPidCount > kMaxRequestedPids) {
		outRequestedPidCount = kMaxRequestedPids;
	}

	for (size_t i = 0; i < outRequestedPidCount; ++i) {
		outRequestedPids[i] = requestedPids[i];
	}

	xSemaphoreGive(mutex);
}
