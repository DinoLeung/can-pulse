#include "can_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can_bus.h"
#include "can_frame_cache.h"

// message queue for can forwarding task
static QueueHandle_t messageQueue = nullptr;

static void readCan1EnqueueTask(void*);
static void forwardCan1ToCan2Task(void*);

void startCanTasks(BaseType_t xCoreID) {
	initCanFrameCache();
	// messageQueue = xQueueCreate(16, sizeof(twai_message_t));
	xTaskCreatePinnedToCore(readCan1EnqueueTask, "CAN1_Read", 4096, NULL, PRIO_CAN_READ, NULL, xCoreID);
	// xTaskCreate(forwardCan1ToCan2Task, "CAN1_Forward", 4096, NULL, 1, NULL);
	// xTaskCreate(sensorCanWriterTask, "Sensor_CAN_Writer", 2048, NULL, 1, NULL);
}

/**
 * @brief FreeRTOS task that polls CAN1 (TWAI) for incoming frames and enqueues them.
 *
 * This task continuously reads available CAN1 messages using `twai_receive()`.
 * For each received frame, it logs the identifier, frame type (standard or extended),
 * and payload bytes to Serial, then sends the `twai_message_t` into the global
 * FreeRTOS queue `messageQueue`. After draining the buffer, it delays for one tick
 * to yield CPU time to other tasks.
 */
void readCan1EnqueueTask(void* pvParameters) {
	(void)pvParameters;
	twai_message_t message;
	while (true) {
		while (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
			// emit message into RTOS queue
			// xQueueSend(messageQueue, &message, portMAX_DELAY);

			bool status = updateCanFrameCache(
				message.identifier,
				message.extd,
				message.data_length_code,
				message.data
			);
		}
		taskYIELD();
	}
}

/**
 * @brief FreeRTOS task that forwards CAN1 messages from the queue to CAN2.
 *
 * This task blocks on the global FreeRTOS queue `messageQueue` until a
 * `twai_message_t` arrives. Upon receiving a message, it invokes
 * `writeCan2()` to transmit the frame on the MCP2515-based CAN2 interface.
 * The task runs in an infinite loop to continuously handle incoming frames.
 *
 * @param pvParameters Unused parameter for FreeRTOS compatibility.
 */
void forwardCan1ToCan2Task(void* pvParameters) {
	(void)pvParameters;
	twai_message_t message;
	while (true) {
		if (xQueueReceive(messageQueue, &message, portMAX_DELAY) == pdTRUE) {
			writeCan2(message);
		}
	}
}
