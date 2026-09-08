#include "esp_rd-03d.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RADAR_UART_PORT UART_NUM_2
#define RADAR_UART_RX_PIN GPIO_NUM_2
#define RADAR_UART_TX_PIN GPIO_NUM_1
#define RADAR_UART_BAUD 256000

#define RADAR_CHANNEL 1
#define RADAR_NAME_MAX 24
#define RADAR_TEXT_MAX 160
#define RADAR_HELLO_INTERVAL_MS 2000
#define RADAR_TELEMETRY_INTERVAL_MS 100
#define RADAR_STALE_AFTER_MS 500

#define ESPNOW_PROTOCOL_MAGIC 0x47455350u
#define ESPNOW_PROTOCOL_VERSION 1u
#define ESPNOW_TYPE_HELLO 1u
#define ESPNOW_TYPE_MESSAGE 2u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t channel;
  uint8_t name_len;
  uint8_t text_len;
  char name[RADAR_NAME_MAX];
  char text[RADAR_TEXT_MAX];
} espnow_packet_t;

typedef struct {
  uint8_t sender_mac[6];
  char text[RADAR_TEXT_MAX];
} radar_command_t;

static const char *TAG = "rd03d_node";
static const uint8_t s_broadcast_mac[6] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static radar_sensor_t s_radar;
static QueueHandle_t s_command_queue;
static char s_name[RADAR_NAME_MAX];
static uint8_t s_host_mac[6];
static bool s_host_known;
static bool s_streaming;
static uint32_t s_sequence;
static uint32_t s_telemetry_sent;
static uint32_t s_telemetry_send_failures;
static bool s_uart_sample_logged;

static uint32_t uptime_ms(void) {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static bool ensure_peer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }

  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, mac, sizeof(peer.peer_addr));
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
    ESP_LOGW(TAG, "Unable to add ESP-NOW peer: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

static bool send_packet(const uint8_t mac[6], uint8_t type, const char *text) {
  if (!ensure_peer(mac)) {
    return false;
  }

  espnow_packet_t packet = {0};
  packet.magic = ESPNOW_PROTOCOL_MAGIC;
  packet.version = ESPNOW_PROTOCOL_VERSION;
  packet.type = type;
  packet.channel = RADAR_CHANNEL;
  snprintf(packet.name, sizeof(packet.name), "%s", s_name);
  packet.name_len = (uint8_t)strnlen(packet.name, sizeof(packet.name));
  if (text) {
    snprintf(packet.text, sizeof(packet.text), "%s", text);
  }
  packet.text_len = (uint8_t)strnlen(packet.text, sizeof(packet.text));

  esp_err_t err = esp_now_send(mac, (const uint8_t *)&packet, sizeof(packet));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "ESP-NOW send failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

static void send_hello(void) {
  (void)send_packet(s_broadcast_mac, ESPNOW_TYPE_HELLO, NULL);
}

static void radar_receive_callback(const esp_now_recv_info_t *info,
                                   const uint8_t *data, int data_len) {
  if (!info || !info->src_addr || !data || data_len != sizeof(espnow_packet_t) ||
      !s_command_queue) {
    return;
  }

  const espnow_packet_t *packet = (const espnow_packet_t *)data;
  if (packet->magic != ESPNOW_PROTOCOL_MAGIC ||
      packet->version != ESPNOW_PROTOCOL_VERSION ||
      packet->type != ESPNOW_TYPE_MESSAGE ||
      packet->text_len >= sizeof(packet->text) ||
      packet->name_len >= sizeof(packet->name)) {
    return;
  }

  radar_command_t command = {0};
  memcpy(command.sender_mac, info->src_addr, sizeof(command.sender_mac));
  memcpy(command.text, packet->text, packet->text_len);
  command.text[packet->text_len] = '\0';
  (void)xQueueSend(s_command_queue, &command, 0);
}

static void handle_commands(void) {
  radar_command_t command;
  while (xQueueReceive(s_command_queue, &command, 0) == pdTRUE) {
    memcpy(s_host_mac, command.sender_mac, sizeof(s_host_mac));
    s_host_known = ensure_peer(s_host_mac);

    if (strcmp(command.text, "RADAR_START") == 0) {
      s_streaming = true;
      ESP_LOGI(TAG, "Streaming enabled for %02X:%02X:%02X:%02X:%02X:%02X",
               s_host_mac[0], s_host_mac[1], s_host_mac[2], s_host_mac[3],
               s_host_mac[4], s_host_mac[5]);
      if (s_host_known) {
        (void)send_packet(s_host_mac, ESPNOW_TYPE_MESSAGE, "RADAR_ACK,START");
      }
    } else if (strcmp(command.text, "RADAR_STOP") == 0) {
      s_streaming = false;
      ESP_LOGI(TAG, "Streaming disabled");
      if (s_host_known) {
        (void)send_packet(s_host_mac, ESPNOW_TYPE_MESSAGE, "RADAR_ACK,STOP");
      }
    }
  }
}

static void send_telemetry(void) {
  if (!s_streaming || !s_host_known) {
    return;
  }

  radar_target_t targets[RADAR_MAX_TARGETS];
  radar_sensor_get_raw_targets(&s_radar, targets, RADAR_MAX_TARGETS);
  uint32_t age_ms = radar_sensor_get_frame_age_ms(&s_radar);
  bool fresh = age_ms <= RADAR_STALE_AFTER_MS;
  uint32_t sequence = s_sequence++;

  for (uint8_t target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
    radar_target_t *target = &targets[target_id];
    bool detected = fresh && target->detected;
    long x = detected ? (long)target->x : 0;
    long y = detected ? (long)target->y : 0;
    long speed = detected ? (long)target->speed : 0;
    long distance = detected ? (long)target->distance : 0;
    long angle = detected ? (long)target->angle : 0;

    char text[RADAR_TEXT_MAX];
    snprintf(text, sizeof(text), "RADAR,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld",
             (unsigned long)sequence, (unsigned)target_id, detected ? 1u : 0u,
             x, y, speed, distance, angle);
    if (send_packet(s_host_mac, ESPNOW_TYPE_MESSAGE, text)) {
      s_telemetry_sent++;
    } else {
      s_telemetry_send_failures++;
    }
  }
}

static void initialize_nvs(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

static void initialize_wifi(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(RADAR_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

void app_main(void) {
  initialize_nvs();
  initialize_wifi();

  uint8_t mac[6] = {0};
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
  snprintf(s_name, sizeof(s_name), "RD03D-%02X%02X%02X", mac[3], mac[4], mac[5]);

  s_command_queue = xQueueCreate(4, sizeof(radar_command_t));
  if (!s_command_queue) {
    ESP_LOGE(TAG, "Unable to allocate command queue");
    return;
  }

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(radar_receive_callback));

  ESP_ERROR_CHECK(radar_sensor_init(&s_radar, RADAR_UART_PORT,
                                    RADAR_UART_RX_PIN, RADAR_UART_TX_PIN));
  ESP_ERROR_CHECK(radar_sensor_begin(&s_radar, RADAR_UART_BAUD));
  radar_sensor_enable_retention(&s_radar, false);

  ESP_LOGI(TAG, "Ready as %s on ESP-NOW channel %u UART%u RX=%d TX=%d baud=%u",
           s_name, RADAR_CHANNEL, RADAR_UART_PORT, RADAR_UART_RX_PIN,
           RADAR_UART_TX_PIN, RADAR_UART_BAUD);
  send_hello();

  uint32_t last_hello = uptime_ms();
  uint32_t last_telemetry = uptime_ms();
  uint32_t last_diagnostic = uptime_ms();
  while (true) {
    (void)radar_sensor_update(&s_radar);
    handle_commands();

    uint32_t now = uptime_ms();
    if ((uint32_t)(now - last_hello) >= RADAR_HELLO_INTERVAL_MS) {
      send_hello();
      last_hello = now;
    }
    if ((uint32_t)(now - last_telemetry) >= RADAR_TELEMETRY_INTERVAL_MS) {
      send_telemetry();
      last_telemetry = now;
    }
    if ((uint32_t)(now - last_diagnostic) >= 2000) {
       radar_target_t raw_targets[RADAR_MAX_TARGETS];
       radar_sensor_get_raw_targets(&s_radar, raw_targets, RADAR_MAX_TARGETS);
       uint8_t raw_count = radar_sensor_get_raw_target_count(&s_radar);
      int rx_level = gpio_get_level(RADAR_UART_RX_PIN);
      int tx_level = gpio_get_level(RADAR_UART_TX_PIN);
      if (!s_uart_sample_logged &&
          radar_sensor_get_rx_byte_count(&s_radar) > 0) {
        uint8_t sample[RADAR_RX_SAMPLE_SIZE];
        size_t sample_len = radar_sensor_copy_rx_sample(
            &s_radar, sample, sizeof(sample));
        char hex[RADAR_RX_SAMPLE_SIZE * 3 + 1];
        size_t hex_len = 0;
        for (size_t i = 0; i < sample_len && hex_len + 3 < sizeof(hex); ++i) {
          hex_len += (size_t)snprintf(hex + hex_len, sizeof(hex) - hex_len,
                                      "%02X%s", sample[i],
                                      i + 1 == sample_len ? "" : " ");
        }
        ESP_LOGI(TAG, "UART raw first %u bytes: %s", (unsigned)sample_len,
                 hex);
        s_uart_sample_logged = true;
      }
      ESP_LOGI(TAG,
               "Diag: uart_bytes=%lu frames=%lu bad_frames=%lu age_ms=%lu "
               "rx_level=%d tx_level=%d "
                "raw_targets=%u t0=%d x=%.0f y=%.0f speed=%.0f dist=%.0f angle=%.0f "
                "telemetry_sent=%lu tx_fail=%lu streaming=%d host=%d",
               (unsigned long)radar_sensor_get_rx_byte_count(&s_radar),
               (unsigned long)radar_sensor_get_frame_count(&s_radar),
               (unsigned long)radar_sensor_get_invalid_frame_count(&s_radar),
               (unsigned long)radar_sensor_get_frame_age_ms(&s_radar),
               rx_level, tx_level,
                (unsigned)raw_count, raw_targets[0].detected ? 1 : 0,
                raw_targets[0].x, raw_targets[0].y, raw_targets[0].speed,
                raw_targets[0].distance, raw_targets[0].angle,
                (unsigned long)s_telemetry_sent,
                (unsigned long)s_telemetry_send_failures, s_streaming ? 1 : 0,
                s_host_known ? 1 : 0);
       for (uint8_t target_id = 0; target_id < RADAR_MAX_TARGETS; ++target_id) {
         if (raw_targets[target_id].detected) {
           ESP_LOGI(TAG, "Target[%u]: x=%.0f y=%.0f speed=%.0f dist=%.0f angle=%.0f",
                    (unsigned)target_id, raw_targets[target_id].x,
                    raw_targets[target_id].y, raw_targets[target_id].speed,
                    raw_targets[target_id].distance, raw_targets[target_id].angle);
         }
       }
      last_diagnostic = now;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
