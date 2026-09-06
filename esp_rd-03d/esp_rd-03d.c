#include "esp_rd-03d.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "ESP_RD_03D";

// Private function to update retention logic
static void radar_sensor_update_retention(radar_sensor_t* sensor,
                                          bool sample_updated);

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

esp_err_t radar_sensor_init(radar_sensor_t* sensor,
                            uart_port_t uart_port,
                            gpio_num_t rx_pin,
                            gpio_num_t tx_pin) {
  if (!sensor) {
    return ESP_ERR_INVALID_ARG;
  }

  sensor->uart_port = uart_port;
  sensor->rx_pin = rx_pin;
  sensor->tx_pin = tx_pin;
  sensor->buffer_index = 0;
  sensor->parser_state = WAIT_AA;
  sensor->frame_count = 0;
  sensor->last_frame_time = xTaskGetTickCount();

  // Initialize target structures
  sensor->target.detected = false;
  sensor->target.x = 0.0f;
  sensor->target.y = 0.0f;
  sensor->target.speed = 0.0f;
  sensor->target.distance = 0.0f;
  sensor->target.angle = 0.0f;
  strcpy(sensor->target.position_description, "No target");

  sensor->raw_target =
      sensor->target;  // Initialize raw target same as filtered

  // Initialize retention system with default values
  sensor->retention.detection_retention_ms =
      RADAR_DEFAULT_DETECTION_RETENTION_MS;
  sensor->retention.absence_retention_ms = RADAR_DEFAULT_ABSENCE_RETENTION_MS;
  sensor->retention.last_detection_time = sensor->last_frame_time;
  sensor->retention.last_absence_time = sensor->last_frame_time;
  sensor->retention.raw_detected = false;
  sensor->retention.filtered_detected = false;
  sensor->retention.retention_enabled = true;  // Enable by default

  ESP_LOGI(TAG,
           "Radar sensor initialized with retention: detection=%lu ms, "
           "absence=%lu ms",
           sensor->retention.detection_retention_ms,
           sensor->retention.absence_retention_ms);

  return ESP_OK;
}

esp_err_t radar_sensor_begin(radar_sensor_t* sensor, uint32_t baud_rate) {
  if (!sensor) {
    return ESP_ERR_INVALID_ARG;
  }

  uart_config_t uart_config = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  esp_err_t ret = uart_param_config(sensor->uart_port, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure UART parameters");
    return ret;
  }

  ret = uart_set_pin(sensor->uart_port, sensor->tx_pin, sensor->rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set UART pins");
    return ret;
  }

  ret = uart_driver_install(sensor->uart_port, 1024, 1024, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver");
    return ret;
  }

  return ESP_OK;
}

bool radar_sensor_update(radar_sensor_t* sensor) {
  if (!sensor) {
    return false;
  }

  bool data_updated = false;
  uint8_t byte_in;

  while (uart_read_bytes(sensor->uart_port, &byte_in, 1, 0) > 0) {
    switch (sensor->parser_state) {
      case WAIT_AA:
        if (byte_in == 0xAA) {
          sensor->parser_state = WAIT_FF;
        }
        break;

      case WAIT_FF:
        if (byte_in == 0xFF) {
          sensor->parser_state = WAIT_03;
        } else {
          sensor->parser_state = WAIT_AA;
        }
        break;

      case WAIT_03:
        if (byte_in == 0x03) {
          sensor->parser_state = WAIT_00;
        } else {
          sensor->parser_state = WAIT_AA;
        }
        break;

      case WAIT_00:
        if (byte_in == 0x00) {
          sensor->buffer_index = 0;
          sensor->parser_state = RECEIVE_FRAME;
        } else {
          sensor->parser_state = WAIT_AA;
        }
        break;

      case RECEIVE_FRAME:
        sensor->buffer[sensor->buffer_index++] = byte_in;
        if (sensor->buffer_index >= RADAR_FULL_FRAME_SIZE) {
          // Check tail bytes
          if (sensor->buffer[24] == 0x55 && sensor->buffer[25] == 0xCC) {
            data_updated = radar_sensor_parse_data(sensor, sensor->buffer,
                                                   RADAR_FRAME_SIZE);
          }
          sensor->parser_state = WAIT_AA;
          sensor->buffer_index = 0;
        }
        break;
    }
  }

  // Keep retention timers moving even when the UART stops producing frames.
  radar_sensor_update_retention(sensor, data_updated);

  return data_updated;
}

bool radar_sensor_parse_data(radar_sensor_t* sensor,
                             const uint8_t* buf,
                             size_t len) {
  if (!sensor || !buf || len != RADAR_FRAME_SIZE) {
    return false;
  }

  // Parse first 8 bytes for the first target
  uint16_t raw_x = buf[0] | (buf[1] << 8);
  uint16_t raw_y = buf[2] | (buf[3] << 8);
  uint16_t raw_speed = buf[4] | (buf[5] << 8);
  uint16_t raw_pixel_dist = buf[6] | (buf[7] << 8);

  // Store raw target data (unfiltered)
  sensor->raw_target.detected =
      !(raw_x == 0 && raw_y == 0 && raw_pixel_dist == 0);

  // Parse signed values
  sensor->raw_target.x = ((raw_x & 0x8000) ? 1 : -1) * (raw_x & 0x7FFF);
  sensor->raw_target.y = ((raw_y & 0x8000) ? 1 : -1) * (raw_y & 0x7FFF);
  sensor->raw_target.speed =
      ((raw_speed & 0x8000) ? 1 : -1) * (raw_speed & 0x7FFF);

  if (sensor->raw_target.detected) {
    sensor->raw_target.distance =
        sqrtf(sensor->raw_target.x * sensor->raw_target.x +
              sensor->raw_target.y * sensor->raw_target.y);

    // Positive X is left and positive Y is forward, so zero is straight ahead.
    sensor->raw_target.angle =
        atan2f(sensor->raw_target.x, sensor->raw_target.y) * (180.0f / M_PI);

    // Update position description
    radar_sensor_update_position_description(&sensor->raw_target);
  } else {
    sensor->raw_target.distance = 0.0f;
    sensor->raw_target.angle = 0.0f;
    strcpy(sensor->raw_target.position_description, "No target");
  }

  // Update retention state
  sensor->retention.raw_detected = sensor->raw_target.detected;
  sensor->frame_count++;
  sensor->last_frame_time = xTaskGetTickCount();

  return true;
}

static void radar_sensor_update_retention(radar_sensor_t* sensor,
                                          bool sample_updated) {
  if (!sensor->retention.retention_enabled) {
    // If retention is disabled, just pass through raw data
    sensor->target = sensor->raw_target;
    sensor->retention.filtered_detected = sensor->retention.raw_detected;
    return;
  }

  TickType_t current_time = xTaskGetTickCount();
  bool state_changed = false;

  // Update timing only when a complete frame was received. This prevents a
  // stalled UART from keeping a stale target alive forever.
  if (sample_updated) {
    if (sensor->retention.raw_detected) {
      sensor->retention.last_detection_time = current_time;
    } else {
      sensor->retention.last_absence_time = current_time;
    }
  }

  // State machine for filtered detection
  if (sensor->retention.filtered_detected) {
    // Currently in "detected" state
    if (sensor->retention.raw_detected) {
      // Still detecting, stay in detected state
      // Copy the latest raw target data to filtered target
      sensor->target = sensor->raw_target;
    } else {
      // No longer detecting raw target
      uint32_t time_since_detection =
          (current_time - sensor->retention.last_detection_time) *
          portTICK_PERIOD_MS;

      if (time_since_detection >= sensor->retention.detection_retention_ms) {
        // Retention time exceeded, switch to not detected
        sensor->retention.filtered_detected = false;
        sensor->target.detected = false;
        sensor->target.x = 0.0f;
        sensor->target.y = 0.0f;
        sensor->target.speed = 0.0f;
        sensor->target.distance = 0.0f;
        sensor->target.angle = 0.0f;
        strcpy(sensor->target.position_description, "No target");
        state_changed = true;

        ESP_LOGI(TAG, "Target lost after %lu ms retention",
                 (unsigned long)time_since_detection);
      }
      // else: stay in detected state (retention active)
    }
  } else {
    // Currently in "not detected" state
    if (sensor->retention.raw_detected) {
      // Raw target detected
      uint32_t time_since_absence =
          (current_time - sensor->retention.last_absence_time) *
          portTICK_PERIOD_MS;

      if (time_since_absence >= sensor->retention.absence_retention_ms) {
        // Absence retention time exceeded, confirm detection
        sensor->retention.filtered_detected = true;
        sensor->target = sensor->raw_target;
        state_changed = true;

        ESP_LOGI(TAG, "Target confirmed after %lu ms absence retention",
                 (unsigned long)time_since_absence);
      }
      // else: stay in not detected state (absence retention active)
    } else {
      // Still no raw target, stay in not detected state
      // Keep target data as zeros (already set)
    }
  }

  if (state_changed) {
    ESP_LOGI(TAG, "Retention state changed to %s",
             sensor->retention.filtered_detected ? "DETECTED" : "NOT_DETECTED");
  }
}

radar_target_t radar_sensor_get_target(radar_sensor_t* sensor) {
  if (!sensor) {
    radar_target_t empty_target = {0};
    return empty_target;
  }

  return sensor->target;  // Returns filtered target
}

radar_target_t radar_sensor_get_raw_target(radar_sensor_t* sensor) {
  if (!sensor) {
    radar_target_t empty_target = {0};
    return empty_target;
  }

  return sensor->raw_target;  // Returns unfiltered raw target
}

uint32_t radar_sensor_get_frame_age_ms(radar_sensor_t* sensor) {
  if (!sensor || sensor->frame_count == 0) {
    return UINT32_MAX;
  }

  return (xTaskGetTickCount() - sensor->last_frame_time) * portTICK_PERIOD_MS;
}

uint32_t radar_sensor_get_frame_count(radar_sensor_t* sensor) {
  return sensor ? sensor->frame_count : 0;
}

void radar_sensor_set_retention_times(radar_sensor_t* sensor,
                                      uint32_t detection_retention_ms,
                                      uint32_t absence_retention_ms) {
  if (!sensor) {
    return;
  }

  sensor->retention.detection_retention_ms = detection_retention_ms;
  sensor->retention.absence_retention_ms = absence_retention_ms;

  ESP_LOGI(TAG, "Retention times updated: detection=%lu ms, absence=%lu ms",
           detection_retention_ms, absence_retention_ms);
}

void radar_sensor_enable_retention(radar_sensor_t* sensor, bool enable) {
  if (!sensor) {
    return;
  }

  sensor->retention.retention_enabled = enable;

  if (!enable) {
    // If disabling retention, immediately sync filtered state with raw state
    sensor->target = sensor->raw_target;
    sensor->retention.filtered_detected = sensor->retention.raw_detected;
  }

  ESP_LOGI(TAG, "Target retention %s", enable ? "enabled" : "disabled");
}

void radar_sensor_reset_retention(radar_sensor_t* sensor) {
  if (!sensor) {
    return;
  }

  TickType_t current_time = xTaskGetTickCount();
  sensor->retention.last_detection_time = current_time;
  sensor->retention.last_absence_time = current_time;
  sensor->retention.filtered_detected = sensor->retention.raw_detected;

  if (sensor->retention.raw_detected) {
    sensor->target = sensor->raw_target;
  } else {
    sensor->target.detected = false;
    sensor->target.x = 0.0f;
    sensor->target.y = 0.0f;
    sensor->target.speed = 0.0f;
    sensor->target.distance = 0.0f;
    sensor->target.angle = 0.0f;
    strcpy(sensor->target.position_description, "No target");
  }

  ESP_LOGI(TAG, "Retention state reset");
}

bool radar_sensor_is_retention_active(radar_sensor_t* sensor) {
  if (!sensor || !sensor->retention.retention_enabled) {
    return false;
  }

  // Retention is active if filtered state differs from raw state
  return (sensor->retention.filtered_detected !=
          sensor->retention.raw_detected);
}

uint32_t radar_sensor_get_time_since_last_detection(radar_sensor_t* sensor) {
  if (!sensor) {
    return 0;
  }

  TickType_t current_time = xTaskGetTickCount();
  return (current_time - sensor->retention.last_detection_time) *
         portTICK_PERIOD_MS;
}

uint32_t radar_sensor_get_time_since_last_absence(radar_sensor_t* sensor) {
  if (!sensor) {
    return 0;
  }

  TickType_t current_time = xTaskGetTickCount();
  return (current_time - sensor->retention.last_absence_time) *
         portTICK_PERIOD_MS;
}

void radar_sensor_deinit(radar_sensor_t* sensor) {
  if (sensor) {
    uart_driver_delete(sensor->uart_port);
    ESP_LOGI(TAG, "Radar sensor deinitialized");
  }
}

// Position description functions
void radar_sensor_update_position_description(radar_target_t* target) {
  if (!target) {
    return;
  }
  if (!target->detected) {
    strcpy(target->position_description, "No target");
    return;
  }

  float x = target->x;
  float y = target->y;
  float distance_m = target->distance / 1000.0f;  // Convert mm to meters

  // Determine primary direction
  const char* horizontal = "";
  const char* vertical = "";
  const char* distance_desc = "";

  // Horizontal direction (X-axis)
  if (fabsf(x) < 100.0f) {
    horizontal = "Center";
  } else if (x > 0) {
    if (x > 1000)
      horizontal = "Far Left";
    else if (x > 500)
      horizontal = "Left";
    else
      horizontal = "Near Left";
  } else {
    if (x < -1000)
      horizontal = "Far Right";
    else if (x < -500)
      horizontal = "Right";
    else
      horizontal = "Near Right";
  }

  // Vertical direction (Y-axis)
  if (fabsf(y) < 100.0f) {
    vertical = "Center";
  } else if (y > 0) {
    if (y > 2000)
      vertical = "Far Forward";
    else if (y > 1000)
      vertical = "Forward";
    else
      vertical = "Near Forward";
  } else {
    if (y < -2000)
      vertical = "Far Behind";
    else if (y < -1000)
      vertical = "Behind";
    else
      vertical = "Near Behind";
  }

  // Distance description
  if (distance_m < 0.5f) {
    distance_desc = "Very Close";
  } else if (distance_m < 1.0f) {
    distance_desc = "Close";
  } else if (distance_m < 2.0f) {
    distance_desc = "Medium";
  } else if (distance_m < 4.0f) {
    distance_desc = "Far";
  } else {
    distance_desc = "Very Far";
  }

  // Create comprehensive description
  if (strcmp(horizontal, "Center") == 0 && strcmp(vertical, "Center") == 0) {
    snprintf(target->position_description, 64, "Directly at sensor (%.1fm)",
             distance_m);
  } else if (strcmp(horizontal, "Center") == 0) {
    snprintf(target->position_description, 64, "%s - %s (%.1fm)", vertical,
             distance_desc, distance_m);
  } else if (strcmp(vertical, "Center") == 0) {
    snprintf(target->position_description, 64, "%s - %s (%.1fm)", horizontal,
             distance_desc, distance_m);
  } else {
    snprintf(target->position_description, 64, "%s %s - %s (%.1fm)", horizontal,
             vertical, distance_desc, distance_m);
  }
}

const char* radar_sensor_get_quadrant_name(float x, float y) {
  if (x >= 0 && y >= 0)
    return "Front-Left (Q1)";
  if (x < 0 && y >= 0)
    return "Front-Right (Q2)";
  if (x < 0 && y < 0)
    return "Back-Right (Q3)";
  if (x >= 0 && y < 0)
    return "Back-Left (Q4)";
  return "Unknown";
}

const char* radar_sensor_get_direction_description(float x,
                                                   float y,
                                                   float distance) {
  static char desc[32];
  const char* quadrant = radar_sensor_get_quadrant_name(x, y);
  snprintf(desc, 32, "%s (%.1fm)", quadrant, distance / 1000.0f);
  return desc;
}
