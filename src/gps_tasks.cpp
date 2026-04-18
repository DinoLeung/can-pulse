#include "gps_tasks.h"

#include <cstdlib>

#include "gps.h"
#include "rc_ble.h"
#include "gps_snapshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "gps_task";

constexpr uint32_t GPS_TASK_STACK_SIZE = 4096;

static void readGpsTask(void*);

void startGpsTasks(BaseType_t xCoreID) {
	initGpsSnapshot();
	xTaskCreatePinnedToCore(readGpsTask, "GPS_Read", GPS_TASK_STACK_SIZE, NULL, PRIO_GPS_READ, NULL, xCoreID);
	ESP_LOGI(TAG, "Start reading GPS");
}

/**
 * @brief FreeRTOS task that continuously reads and parses NMEA data from the GPS bolt-on.
 */
void readGpsTask(void* pvParameters) {
	(void)pvParameters;
	while (true) {
		bool sentenceCompleted = false;
		
		while (g_gpsSerial.available() > 0) {
			if (g_gps.encode(static_cast<char>(g_gpsSerial.read()))) {
				sentenceCompleted = true;
			}
		}
		if (sentenceCompleted) {
			g_gpsSnapshotStore.update();
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
