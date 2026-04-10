#include <Arduino.h>
#include "can_bus.h"
#include "gps.h"
#include "rc_ble.h"
#include "sensor_tasks.h"
#include "can_tasks.h"
#include "gps_tasks.h"
#include "rc_ble_tasks.h"

void setup() {
	Serial.begin(115200);
	delay(1000);

	initCanBus();
	initGps();
	initRaceChronoBle();
	
	startCanTasks();
	startGpsTasks();
	// startSensorTasks();
	startRaceChronoTasks();
}

void loop() { /* DO NOTHING */ }
