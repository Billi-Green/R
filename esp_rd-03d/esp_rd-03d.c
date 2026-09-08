#include "esp_rd-03d.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "ESP_RD_03D";

static const uint8_t RADAR_CONFIG_BEGIN_COMMAND[14] = {
    0xFD, 0xFC, 0xFB, 0xFA,
    0x04, 0x00,
    0xFF, 0x00,
    0x01, 0x00,
    0x04, 0x03, 0x02, 0x01,
};

static const uint8_t RADAR_MULTI_TARGET_COMMAND[12] = {
    0xFD, 0xFC, 0xFB, 0xFA,
    0x02, 0x00,
    0x90, 0x00,
    0x04, 0x03, 0x02, 0x01,
};

static const uint8_t RADAR_CONFIG_END_COMMAND[12] = {
    0xFD, 0xFC, 0xFB, 0xFA,
    0x02, 0x00,
    0xFE, 0x00,
    0x04, 0x03, 0x02, 0x01,
};

// Private function to update retention logic
static void radar_sensor_update_retention(radar_sensor_t* sensor,
                                          bool sample_updated);

static esp_err_t radar_sensor_send_command(radar_sensor_t* sensor,
                                           const uint8_t* command,
                                           size_t command_len,
                                           const char* name) {
  static const uint8_t ack_header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  static const uint8_t ack_tail[4] = {0x04, 0x03, 0x02, 0x01};
  uint8_t response[32] = {0};
  size_t response_len = 0;
  size_t header_index = 0;
  bool collecting = false;

  uart_flush_input(sensor->uart_port);
  int written = uart_write_bytes(sensor->uart_port, command, command_len);
  if (written != (int)command_len) {
    ESP_LOGW(TAG, "%s write incomplete: %d/%u", name, written,
             (unsigned)command_len);
    return ESP_FAIL;
  }
  (void)uart_wait_tx_done(sensor->uart_port, pdMS_TO_TICKS(100));

  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
  while ((int32_t)(xTaskGetTickCount() - deadline) < 0) {
    uint8_t byte_in = 0;
    if (uart_read_bytes(sensor->uart_port, &byte_in, 1,
                        pdMS_TO_TICKS(20)) <= 0) {
      continue;
    }

    if (!collecting) {
      if (byte_in == ack_header[header_index]) {
        header_index++;
        if (header_index == sizeof(ack_header)) {
          memcpy(response, ack_header, sizeof(ack_header));
          response_len = sizeof(ack_header);
          collecting = true;
        }
      } else {
        header_index = byte_in == ack_header[0] ? 1 : 0;
      }
      continue;
    }

    if (response_len >= sizeof(response)) {
      return ESP_FAIL;
    }
    response[response_len++] = byte_in;
    if (response_len >= sizeof(ack_tail) &&
        memcmp(response + response_len - sizeof(ack_tail), ack_tail,
               sizeof(ack_tail)) == 0) {
      if (response_len >= 10 &&
          (response[8] != 0x00 || response[9] != 0x00)) {
        ESP_LOGW(TAG, "%s returned failure status %02X %02X", name,
                 response[8], response[9]);
        return ESP_FAIL;
      }
      ESP_LOGI(TAG, "%s ACK received (%u bytes)", name,
               (unsigned)response_len);
      return ESP_OK;
    }
  }

  ESP_LOGW(TAG, "%s ACK timeout", name);
  return ESP_ERR_TIMEOUT;
}

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
  sensor->rx_byte_count = 0;
  sensor->invalid_frame_count = 0;
  sensor->rx_sample_count = 0;
  sensor->last_frame_time = xTaskGetTickCount();

  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    sensor->target[i].target_id = (uint8_t)i;
    sensor->target[i].detected = false;
    sensor->target[i].x = 0.0f;
    sensor->target[i].y = 0.0f;
    sensor->target[i].speed = 0.0f;
    sensor->target[i].distance = 0.0f;
    sensor->target[i].angle = 0.0f;
    strcpy(sensor->target[i].position_description, "No target");

    sensor->raw_target[i] = sensor->target[i];

    sensor->retention[i].detection_retention_ms =
        RADAR_DEFAULT_DETECTION_RETENTION_MS;
    sensor->retention[i].absence_retention_ms =
        RADAR_DEFAULT_ABSENCE_RETENTION_MS;
    sensor->retention[i].last_detection_time = sensor->last_frame_time;
    sensor->retention[i].last_absence_time = sensor->last_frame_time;
    sensor->retention[i].raw_detected = false;
    sensor->retention[i].filtered_detected = false;
    sensor->retention[i].retention_enabled = true;
  }

  ESP_LOGI(TAG,
           "Radar sensor initialized with retention: detection=%lu ms, "
           "absence=%lu ms",
            sensor->retention[0].detection_retention_ms,
            sensor->retention[0].absence_retention_ms);

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

  vTaskDelay(pdMS_TO_TICKS(200));
  ret = radar_sensor_send_command(sensor, RADAR_CONFIG_BEGIN_COMMAND,
                                  sizeof(RADAR_CONFIG_BEGIN_COMMAND),
                                  "RD-03D config begin");
  if (ret == ESP_OK) {
    (void)radar_sensor_send_command(sensor, RADAR_MULTI_TARGET_COMMAND,
                                    sizeof(RADAR_MULTI_TARGET_COMMAND),
                                    "RD-03D multi-target mode");
    (void)radar_sensor_send_command(sensor, RADAR_CONFIG_END_COMMAND,
                                    sizeof(RADAR_CONFIG_END_COMMAND),
                                    "RD-03D config end");
  } else {
    ESP_LOGW(TAG, "RD-03D configuration handshake failed; using default mode");
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
    sensor->rx_byte_count++;
    if (sensor->rx_sample_count < RADAR_RX_SAMPLE_SIZE) {
      sensor->rx_sample[sensor->rx_sample_count++] = byte_in;
    }
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
          } else {
            sensor->invalid_frame_count++;
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

  for (size_t target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
    size_t offset = target_id * 8;
    uint16_t raw_x = buf[offset] | (buf[offset + 1] << 8);
    uint16_t raw_y = buf[offset + 2] | (buf[offset + 3] << 8);
    uint16_t raw_speed = buf[offset + 4] | (buf[offset + 5] << 8);
    uint16_t raw_pixel_dist = buf[offset + 6] | (buf[offset + 7] << 8);
    radar_target_t* target = &sensor->raw_target[target_id];

    target->target_id = (uint8_t)target_id;
    target->detected = !(raw_x == 0 && raw_y == 0 && raw_speed == 0 &&
                         raw_pixel_dist == 0);
    target->x = ((raw_x & 0x8000) ? 1 : -1) * (raw_x & 0x7FFF);
    target->y = ((raw_y & 0x8000) ? 1 : -1) * (raw_y & 0x7FFF);
    target->speed = ((raw_speed & 0x8000) ? 1 : -1) * (raw_speed & 0x7FFF);

    if (target->detected) {
      target->distance = sqrtf(target->x * target->x + target->y * target->y);
      target->angle = atan2f(target->x, target->y) * (180.0f / M_PI);
      radar_sensor_update_position_description(target);
    } else {
      target->distance = 0.0f;
      target->angle = 0.0f;
      strcpy(target->position_description, "No target");
    }

    sensor->retention[target_id].raw_detected = target->detected;
  }

  sensor->frame_count++;
  sensor->last_frame_time = xTaskGetTickCount();

  return true;
}

static void radar_sensor_update_retention(radar_sensor_t* sensor,
                                          bool sample_updated) {
  TickType_t current_time = xTaskGetTickCount();
  for (size_t target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
    radar_retention_t* retention = &sensor->retention[target_id];
    radar_target_t* target = &sensor->target[target_id];
    const radar_target_t* raw_target = &sensor->raw_target[target_id];

    if (!retention->retention_enabled) {
      *target = *raw_target;
      retention->filtered_detected = retention->raw_detected;
      continue;
    }

    if (sample_updated) {
      if (retention->raw_detected) {
        retention->last_detection_time = current_time;
      } else {
        retention->last_absence_time = current_time;
      }
    }

    if (retention->filtered_detected) {
      if (retention->raw_detected) {
        *target = *raw_target;
      } else {
        uint32_t elapsed =
            (current_time - retention->last_detection_time) *
            portTICK_PERIOD_MS;
        if (elapsed >= retention->detection_retention_ms) {
          retention->filtered_detected = false;
          target->detected = false;
          target->x = 0.0f;
          target->y = 0.0f;
          target->speed = 0.0f;
          target->distance = 0.0f;
          target->angle = 0.0f;
          strcpy(target->position_description, "No target");
          ESP_LOGI(TAG, "Target %u lost after %lu ms retention",
                   (unsigned)target_id, (unsigned long)elapsed);
        }
      }
    } else if (retention->raw_detected) {
      uint32_t elapsed =
          (current_time - retention->last_absence_time) * portTICK_PERIOD_MS;
      if (elapsed >= retention->absence_retention_ms) {
        retention->filtered_detected = true;
        *target = *raw_target;
        ESP_LOGI(TAG, "Target %u confirmed after %lu ms absence retention",
                 (unsigned)target_id, (unsigned long)elapsed);
      }
    }
  }
}

radar_target_t radar_sensor_get_target(radar_sensor_t* sensor) {
  if (!sensor) {
    radar_target_t empty_target = {0};
    return empty_target;
  }

  return sensor->target[0];  // Compatibility accessor for target slot 0.
}

radar_target_t radar_sensor_get_raw_target(radar_sensor_t* sensor) {
  if (!sensor) {
    radar_target_t empty_target = {0};
    return empty_target;
  }

  return sensor->raw_target[0];  // Compatibility accessor for target slot 0.
}

void radar_sensor_get_targets(radar_sensor_t* sensor,
                              radar_target_t* targets,
                              size_t target_count) {
  if (!sensor || !targets) return;
  if (target_count > RADAR_MAX_TARGETS) target_count = RADAR_MAX_TARGETS;
  memcpy(targets, sensor->target, target_count * sizeof(*targets));
}

void radar_sensor_get_raw_targets(radar_sensor_t* sensor,
                                  radar_target_t* targets,
                                  size_t target_count) {
  if (!sensor || !targets) return;
  if (target_count > RADAR_MAX_TARGETS) target_count = RADAR_MAX_TARGETS;
  memcpy(targets, sensor->raw_target, target_count * sizeof(*targets));
}

uint8_t radar_sensor_get_target_count(radar_sensor_t* sensor) {
  if (!sensor) return 0;
  uint8_t count = 0;
  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    if (sensor->target[i].detected) ++count;
  }
  return count;
}

uint8_t radar_sensor_get_raw_target_count(radar_sensor_t* sensor) {
  if (!sensor) return 0;
  uint8_t count = 0;
  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    if (sensor->raw_target[i].detected) ++count;
  }
  return count;
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

uint32_t radar_sensor_get_rx_byte_count(radar_sensor_t* sensor) {
  return sensor ? sensor->rx_byte_count : 0;
}

uint32_t radar_sensor_get_invalid_frame_count(radar_sensor_t* sensor) {
  return sensor ? sensor->invalid_frame_count : 0;
}

size_t radar_sensor_copy_rx_sample(radar_sensor_t* sensor,
                                   uint8_t* out,
                                   size_t out_len) {
  if (!sensor || !out || out_len == 0) return 0;
  size_t count = sensor->rx_sample_count < out_len
                     ? sensor->rx_sample_count
                     : out_len;
  memcpy(out, sensor->rx_sample, count);
  return count;
}

void radar_sensor_set_retention_times(radar_sensor_t* sensor,
                                      uint32_t detection_retention_ms,
                                      uint32_t absence_retention_ms) {
  if (!sensor) {
    return;
  }

  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    sensor->retention[i].detection_retention_ms = detection_retention_ms;
    sensor->retention[i].absence_retention_ms = absence_retention_ms;
  }

  ESP_LOGI(TAG, "Retention times updated: detection=%lu ms, absence=%lu ms",
           detection_retention_ms, absence_retention_ms);
}

void radar_sensor_enable_retention(radar_sensor_t* sensor, bool enable) {
  if (!sensor) {
    return;
  }

  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    sensor->retention[i].retention_enabled = enable;
    if (!enable) {
      sensor->target[i] = sensor->raw_target[i];
      sensor->retention[i].filtered_detected = sensor->retention[i].raw_detected;
    }
  }

  ESP_LOGI(TAG, "Target retention %s", enable ? "enabled" : "disabled");
}

void radar_sensor_reset_retention(radar_sensor_t* sensor) {
  if (!sensor) {
    return;
  }

  TickType_t current_time = xTaskGetTickCount();
  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    sensor->retention[i].last_detection_time = current_time;
    sensor->retention[i].last_absence_time = current_time;
    sensor->retention[i].filtered_detected = sensor->retention[i].raw_detected;

    if (sensor->retention[i].raw_detected) {
      sensor->target[i] = sensor->raw_target[i];
    } else {
      sensor->target[i].detected = false;
      sensor->target[i].x = 0.0f;
      sensor->target[i].y = 0.0f;
      sensor->target[i].speed = 0.0f;
      sensor->target[i].distance = 0.0f;
      sensor->target[i].angle = 0.0f;
      strcpy(sensor->target[i].position_description, "No target");
    }
  }

  ESP_LOGI(TAG, "Retention state reset");
}

bool radar_sensor_is_retention_active(radar_sensor_t* sensor) {
  if (!sensor || !sensor->retention[0].retention_enabled) {
    return false;
  }

  // Retention is active if filtered state differs from raw state
  for (size_t i = 0; i < RADAR_MAX_TARGETS; ++i) {
    if (sensor->retention[i].filtered_detected !=
        sensor->retention[i].raw_detected) {
      return true;
    }
  }
  return false;
}

uint32_t radar_sensor_get_time_since_last_detection(radar_sensor_t* sensor) {
  if (!sensor) {
    return 0;
  }

  TickType_t current_time = xTaskGetTickCount();
  return (current_time - sensor->retention[0].last_detection_time) *
         portTICK_PERIOD_MS;
}

uint32_t radar_sensor_get_time_since_last_absence(radar_sensor_t* sensor) {
  if (!sensor) {
    return 0;
  }

  TickType_t current_time = xTaskGetTickCount();
  return (current_time - sensor->retention[0].last_absence_time) *
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
