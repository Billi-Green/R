# RD-03D Radar Node

Standalone ESP-IDF firmware for an ESP32-S3 connected to an AI-Thinker RD-03D.
The node exposes the radar over ESP-NOW for the GhostESP RD-03D Radar app.

## Hardware

Required parts:

- ESP32-S3 development board
- AI-Thinker RD-03D radar module
- 3.3V-to-5V boost/buck-boost converter for a stable 5V radar supply
- Common ground between the ESP32-S3 and radar module

Default wiring:

| RD-03D | ESP32-S3 |
| --- | --- |
| VCC | Stable 5V |
| GND | GND |
| TX | GPIO2 / UART2 RX |
| RX | GPIO1 / UART2 TX |

UART wiring is crossed: radar TX goes to the ESP32 RX pin, and radar RX goes
to the ESP32 TX pin. Do not power the radar from an overloaded 3.3V rail.

## Build And Flash

Use ESP-IDF 6.1 from an activated ESP-IDF PowerShell session:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM14 flash monitor
```

Replace `COM14` with the serial port for the ESP32-S3. The radar driver is
part of the `main` component.

## Make Your Own Node

The easiest way to build a compatible module is to copy this project and edit
the constants near the top of `main/main.c`:

```c
#define RADAR_UART_PORT UART_NUM_2
#define RADAR_UART_RX_PIN GPIO_NUM_2
#define RADAR_UART_TX_PIN GPIO_NUM_1
#define RADAR_UART_BAUD 256000
#define RADAR_CHANNEL 1
```

Keep the UART at `256000` baud, 8 data bits, no parity, and 1 stop bit unless
your replacement sensor uses a different protocol. If you use another sensor,
replace the `radar_sensor_*` calls in `main.c`, but keep the ESP-NOW discovery,
command, and telemetry formats below.

The firmware initializes the sensor like this:

```c
radar_sensor_init(&s_radar, UART_NUM_2, GPIO_NUM_2, GPIO_NUM_1);
radar_sensor_begin(&s_radar, 256000);
radar_sensor_enable_retention(&s_radar, false);
```

Use `radar_sensor_get_raw_targets()` when the application must display current
sensor data without retention filtering.

## ESP-NOW Contract

Use ESP-NOW channel 1 and station mode. The node announces itself using a name
starting with `RD03D-`, for example `RD03D-A1B2C3`. The GhostESP app discovers
nodes from these hello packets.

Packets use this packed envelope:

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;       // 0x47455350, "GESP"
    uint8_t version;      // 1
    uint8_t type;         // 1 = HELLO, 2 = MESSAGE
    uint8_t channel;      // 1
    uint8_t name_len;     // bytes used in name
    uint8_t text_len;     // bytes used in text
    char name[24];
    char text[160];
} espnow_packet_t;
```

Send a type `HELLO` packet to the broadcast MAC. For message packets, add the
receiver as an unencrypted ESP-NOW peer before sending.

Commands received from the app:

```text
RADAR_START
RADAR_STOP
```

Reply to the selected host with:

```text
RADAR_ACK,START
RADAR_ACK,STOP
```

## Telemetry Format

Send one record for each of the three target slots every update. Use the same
sequence number for all three records from one sensor frame:

```text
RADAR,sequence,target_id,detected,x_mm,y_mm,speed,distance_mm,angle_deg
```

Field requirements:

- `sequence`: increment once per sensor frame
- `target_id`: `0`, `1`, or `2`
- `detected`: `1` for a fresh target, otherwise `0`
- `x_mm`, `y_mm`, `distance_mm`: millimetres when detected; zero otherwise
- `speed`: raw speed value from the sensor
- `angle_deg`: angle in degrees

The GhostESP app displays distance and position in metres, but the wire format
remains millimetres for compatibility.

## Diagnostics

The radar driver is compiled directly into the `main` component from
`main/esp_rd-03d.c` and `main/include/esp_rd-03d.h`.

The node prints a diagnostic line every two seconds containing UART byte count,
decoded frame count, frame age, target data, and ESP-NOW send counts.

If UART bytes remain at zero, check:

1. Radar TX is connected to the ESP32 RX pin.
2. Radar RX is connected to the ESP32 TX pin.
3. The radar has a stable 5V supply and common ground.
4. The selected GPIOs are not being used by another peripheral.

If UART frames work but the app does not discover the node, check that both
devices use ESP-NOW channel 1 and that the node name starts with `RD03D-`.
