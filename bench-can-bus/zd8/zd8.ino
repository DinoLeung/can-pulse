#include <Arduino.h>
#include "driver/twai.h"

// BRZ bus speed from the original example
static constexpr uint32_t CAN_BITRATE = 500000;

// -------------------------------------------------------------------------------------------------
// Frame scheduler
// -------------------------------------------------------------------------------------------------

struct ScheduledPid {
  uint16_t pid;
  uint16_t rate_hz;
  uint32_t last_sent_us;
};

// Keep the same spirit as the original example:
// - 0x18, 0x140, 0x141, 0x142 at 100 Hz
// - several others at 50 / 20 / 10 / 16.7 Hz
ScheduledPid scheduledPids[] = {
  
  // somewhat real can data
  {0x040, 100, 0},
  {0x138,  50, 0},
  {0x139,  50, 0},
  {0x13B,  50, 0},
  {0x146,  50, 0},
  {0x241,  20, 0},
  {0x345,  10, 0},
  
  // placeholders
  {0x018, 100, 0},
  {0x390,  50, 0},
  {0x141, 100, 0},
  {0x142, 100, 0},
  {0x144,  50, 0},
  {0x152,  50, 0},
  {0x156,  50, 0},
  {0x280,  50, 0},
  {0x282,  17, 0},
  {0x284,  10, 0},

};

// -------------------------------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------------------------------

static inline uint16_t hzToPeriodUs(uint16_t hz) {
  return 1000000UL / hz;
}

bool sendFrame(uint16_t id, const uint8_t *payload, uint8_t len) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;
  memcpy(msg.data, payload, len);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(50));
  if (err != ESP_OK) {
    Serial.printf("TX failed id=0x%03X err=%d\n", id, err);
    return false;
  }
  return true;
}

// -------------------------------------------------------------------------------------------------
// Fake BRZ payload generation
// -------------------------------------------------------------------------------------------------

void generatePayload(uint16_t pid, uint8_t *payload) {
  memset(payload, 0, 8);

  switch (pid) {
    case 0x040: {
      // engine speed / accelerator pos
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0xCF;
      payload[3] = 0x82;
      payload[4] = 0x99;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x138: {
      // Steering / yaw
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0xC1;
      payload[3] = 0xFF;
      payload[4] = 0xFA;
      payload[5] = 0x3F;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x139: {
      // Speed / break pressure
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0xFC;
      payload[3] = 0xE0;
      payload[4] = 0x00;
      payload[5] = 0x01;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x13B: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x00;
      payload[5] = 0x0;
      payload[6] = 0xFE;
      payload[7] = 0xFD;
      break;
    }

    case 0x146: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x89;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x241: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x30;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x345: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x84;
      payload[4] = 0x82;
      payload[5] = 0x12;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }
    
    case 0x390: {
      // acceleration
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x90;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x018: {
      // Placeholder
      payload[0] = 0x00;
      payload[1] = 0x00;
      payload[2] = 0x00;
      payload[3] = 0x00;
      payload[4] = 0x00;
      payload[5] = 0x00;
      payload[6] = 0x00;
      payload[7] = 0x00;
      break;
    }

    case 0x0D2: {
      // Placeholder
      payload[0] = 0x11;
      payload[1] = 0x22;
      payload[2] = 0x33;
      payload[3] = 0x44;
      break;
    }

    case 0x0D3: {
      // Placeholder
      payload[0] = 0x55;
      payload[1] = 0x66;
      payload[2] = 0x77;
      payload[3] = 0x88;
      break;
    }

    case 0x141: {
      // Placeholder
      payload[0] = 0x01;
      payload[1] = 0x02;
      payload[2] = 0x03;
      payload[3] = 0x04;
      break;
    }

    case 0x142: {
      // Placeholder
      payload[0] = 0x10;
      payload[1] = 0x20;
      payload[2] = 0x30;
      payload[3] = 0x40;
      break;
    }

    case 0x144: {
      // Placeholder
      payload[0] = 0xAA;
      payload[1] = 0xBB;
      payload[2] = 0xCC;
      payload[3] = 0xDD;
      break;
    }

    case 0x152: {
      // Placeholder
      payload[0] = 0x12;
      payload[1] = 0x34;
      payload[2] = 0x56;
      payload[3] = 0x78;
      break;
    }

    case 0x156: {
      // Placeholder
      payload[0] = 0x9A;
      payload[1] = 0xBC;
      payload[2] = 0xDE;
      payload[3] = 0xF0;
      break;
    }

    case 0x280: {
      // Placeholder
      payload[0] = 0x01;
      break;
    }

    case 0x282: {
      // Placeholder
      payload[0] = 0x02;
      break;
    }

    case 0x284: {
      // Placeholder
      payload[0] = 0x03;
      break;
    }

    default:
      break;
  }
}

// -------------------------------------------------------------------------------------------------
// Setup / loop
// -------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("Initializing CAN1 (TWAI) on ESP32-CAN-X2...");

  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_NO_ACK);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\n", err);
    while (true) delay(1000);
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\n", err);
    while (true) delay(1000);
  }

  Serial.println("CAN1 started at 500 kbit/s");
}

void loop() {
  uint32_t nowUs = micros();

  for (size_t i = 0; i < sizeof(scheduledPids) / sizeof(scheduledPids[0]); i++) {
    ScheduledPid &entry = scheduledPids[i];
    const uint32_t periodUs = hzToPeriodUs(entry.rate_hz);

    if ((uint32_t)(nowUs - entry.last_sent_us) >= periodUs) {
      uint8_t payload[8];
      generatePayload(entry.pid, payload);

      if (sendFrame(entry.pid, payload, 8)) {
        entry.last_sent_us = nowUs;
      } else {
        // Don't spam this too hard on a busy bus
        Serial.printf("TX failed for 0x%03X\n", entry.pid);
      }
    }
  }

  // Keep loop tight but not stupidly busy
  delayMicroseconds(200);
}