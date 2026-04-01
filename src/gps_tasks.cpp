#include "gps_tasks.h"

#include <cstdlib>

#include "gps.h"
#include "rc_ble.h"
#include "gps_snapshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

constexpr uint32_t GPS_TASK_STACK_SIZE = 4096;
constexpr TickType_t GPS_TASK_DELAY = pdMS_TO_TICKS(20);

static void readGpsTask(void*);

void startGpsTasks() {
	xTaskCreate(readGpsTask, "GPS_Read", GPS_TASK_STACK_SIZE, NULL, PRIO_GPS_READ, NULL);
}

/**
 * @brief FreeRTOS task that continuously reads and parses NMEA data from the GPS bolt-on.
 */
void readGpsTask(void* pvParameters) {
	(void)pvParameters;

	while (true) {
		bool sentenceCompleted = false;

		while (Serial1.available() > 0) {
			if (g_gps.encode(static_cast<char>(Serial1.read()))) {
				sentenceCompleted = true;
			}
		}
		if (sentenceCompleted) {
			updateGpsSnapshot();
		}

		vTaskDelay(GPS_TASK_DELAY);
	}
}
