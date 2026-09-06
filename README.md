# RD-03D Radar Node

Standalone ESP-IDF firmware for an ESP32-S3 connected to an AI-Thinker RD-03D.

## Wiring

- RD-03D TX -> ESP32-S3 GPIO 1 (UART RX)
- RD-03D RX -> ESP32-S3 GPIO 2 (UART TX)
- RD-03D GND -> ESP32-S3 GND

The node uses UART1 at 256000 baud and ESP-NOW channel 1. It announces as
`RD03D-XXXXXX`, accepts `RADAR_START` and `RADAR_STOP`, and sends compact
telemetry messages compatible with the GhostESP ESP-NOW manager:

```text
RADAR,sequence,detected,x_mm,y_mm,speed,distance_mm,angle_deg
```

## Build

Use ESP-IDF 6.0.2 from an activated ESP-IDF PowerShell session:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

The local `esp_rd-03d` component is registered through `EXTRA_COMPONENT_DIRS`.
