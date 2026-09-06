# ESP RD-03D Radar Sensor Component

A comprehensive ESP-IDF component for interfacing with the AI-Thinker RD-03D radar sensor module, featuring advanced target detection, position tracking, and intelligent retention filtering.

## Features

- **Real-time Target Detection**: Continuous monitoring of moving targets
- **Position Tracking**: X/Y coordinates, distance, angle, and speed measurements
- **Intelligent Retention System**: Configurable filtering to reduce false positives/negatives
- **Human-Readable Descriptions**: Automatic generation of position descriptions
- **Raw Data Access**: Access to both filtered and unfiltered sensor data
- **Diagnostic Tools**: Built-in functions for monitoring sensor state and performance

## Hardware Requirements

- ESP32 or compatible microcontroller
- AI-Thinker RD-03D radar sensor module
- UART connection (RX/TX pins)

## Installation

### ESP Component Registry

```bash
idf.py add-dependency "heronet/esp_rd-03d"
```

### Manual Installation

1. Clone this repository into your project's `components` directory
2. Include the component in your main CMakeLists.txt

## Basic Usage

### Initialization

```c
#include "esp_rd-03d.h"

radar_sensor_t radar;

void app_main() {
    // Initialize the radar sensor
    esp_err_t ret = radar_sensor_init(&radar, UART_NUM_2, GPIO_NUM_16, GPIO_NUM_17);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize radar sensor");
        return;
    }

    // Start UART communication
    ret = radar_sensor_begin(&radar, 256000);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to start radar sensor");
        return;
    }

    // Main loop
    while (1) {
        if (radar_sensor_update(&radar)) {
            radar_target_t target = radar_sensor_get_target(&radar);

            if (target.detected) {
                ESP_LOGI("MAIN", "Target detected at (%.1f, %.1f) mm, distance: %.1f mm",
                         target.x, target.y, target.distance);
                ESP_LOGI("MAIN", "Position: %s", target.position_description);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## API Reference

### Core Functions

#### `radar_sensor_init()`

```c
esp_err_t radar_sensor_init(radar_sensor_t* sensor, uart_port_t uart_port, gpio_num_t rx_pin, gpio_num_t tx_pin);
```

Initialize the radar sensor structure and configure pins.

**Parameters:**

- `sensor`: Pointer to radar sensor structure
- `uart_port`: UART port number (e.g., UART_NUM_2)
- `rx_pin`: GPIO pin for UART RX
- `tx_pin`: GPIO pin for UART TX

**Returns:** ESP_OK on success, error code otherwise

#### `radar_sensor_begin()`

```c
esp_err_t radar_sensor_begin(radar_sensor_t* sensor, uint32_t baud_rate);
```

Start UART communication with the specified baud rate.

**Parameters:**

- `sensor`: Pointer to initialized radar sensor
- `baud_rate`: Communication baud rate (typically 115200)

#### `radar_sensor_update()`

```c
bool radar_sensor_update(radar_sensor_t* sensor);
```

Process incoming UART data and update target information.

**Returns:** `true` if new data was processed, `false` otherwise

#### `radar_sensor_get_target()`

```c
radar_target_t radar_sensor_get_target(radar_sensor_t* sensor);
```

Get the current filtered target data (with retention applied).

#### `radar_sensor_get_raw_target()`

```c
radar_target_t radar_sensor_get_raw_target(radar_sensor_t* sensor);
```

Get the raw unfiltered target data directly from the sensor.

### Target Data Structure

```c
typedef struct {
    bool detected;                    // Target detection status
    float x;                         // X coordinate (mm)
    float y;                         // Y coordinate (mm)
    float speed;                     // Target speed
    float distance;                  // Distance from sensor (mm)
    float angle;                     // Angle in degrees
    char position_description[64];   // Human-readable position
} radar_target_t;
```

### Retention System

The retention system helps filter out false positives and negatives by applying configurable delays before changing detection state.

#### Configuration

```c
// Set custom retention times
radar_sensor_set_retention_times(&radar, 5000, 2000); // 5s detection, 2s absence

// Enable/disable retention
radar_sensor_enable_retention(&radar, true);

// Reset retention state
radar_sensor_reset_retention(&radar);
```

#### Default Settings

- **Detection Retention**: 3000ms (3 seconds)
- **Absence Retention**: 1000ms (1 second)

### Position Descriptions

The component automatically generates human-readable position descriptions:

- **Horizontal**: "Far Left", "Left", "Near Left", "Center", "Near Right", "Right", "Far Right"
- **Vertical**: "Far Behind", "Behind", "Near Behind", "Center", "Near Forward", "Forward", "Far Forward"
- **Distance**: "Very Close", "Close", "Medium", "Far", "Very Far"

Example outputs:

- "Left Forward - Medium (1.5m)"
- "Far Right Behind - Close (0.8m)"
- "Directly at sensor (0.3m)"

### Diagnostic Functions

```c
// Check if retention is currently active
bool is_active = radar_sensor_is_retention_active(&radar);

// Get timing information
uint32_t time_since_detection = radar_sensor_get_time_since_last_detection(&radar);
uint32_t time_since_absence = radar_sensor_get_time_since_last_absence(&radar);
```

## Advanced Usage

### Custom Retention Configuration

```c
void configure_radar_for_security() {
    radar_sensor_t radar;

    // Initialize radar
    radar_sensor_init(&radar, UART_NUM_2, GPIO_NUM_16, GPIO_NUM_17);
    radar_sensor_begin(&radar, 115200);

    // Configure for security application (longer retention)
    radar_sensor_set_retention_times(&radar, 10000, 500); // 10s detection, 0.5s absence

    while (1) {
        radar_sensor_update(&radar);
        radar_target_t target = radar_sensor_get_target(&radar);

        if (target.detected) {
            // Security alert logic
            ESP_LOGW("SECURITY", "Intruder detected: %s", target.position_description);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### Real-time vs Raw Data Comparison

```c
void compare_filtered_vs_raw() {
    radar_target_t filtered = radar_sensor_get_target(&radar);
    radar_target_t raw = radar_sensor_get_raw_target(&radar);

    ESP_LOGI("COMPARE", "Filtered: %s, Raw: %s",
             filtered.detected ? "DETECTED" : "CLEAR",
             raw.detected ? "DETECTED" : "CLEAR");

    if (radar_sensor_is_retention_active(&radar)) {
        ESP_LOGI("COMPARE", "Retention is active - states differ");
    }
}
```

## Coordinate System

The AI-Thinker RD-03D uses a coordinate system where:

- **X-axis**: Positive values = Left, Negative values = Right
- **Y-axis**: Positive values = Forward, Negative values = Behind
- **Distance**: Calculated as √(x² + y²)
- **Angle**: Measured in degrees, adjusted for sensor orientation

## Error Handling

```c
esp_err_t ret = radar_sensor_init(&radar, UART_NUM_2, GPIO_NUM_16, GPIO_NUM_17);
switch (ret) {
    case ESP_OK:
        ESP_LOGI("RADAR", "Initialization successful");
        break;
    case ESP_ERR_INVALID_ARG:
        ESP_LOGE("RADAR", "Invalid arguments provided");
        break;
    default:
        ESP_LOGE("RADAR", "Initialization failed: %s", esp_err_to_name(ret));
        break;
}
```

## Performance Considerations

- Call `radar_sensor_update()` regularly (recommended: 50-100ms intervals)
- The retention system processes data even when no new sensor data arrives
- UART buffer size is optimized for the RD-03D frame format
- Position descriptions are updated automatically when target data changes

## Troubleshooting

### No Target Detection

1. Verify UART connections (RX/TX pins)
2. Check baud rate configuration (115200 is standard)
3. Ensure adequate power supply to the radar module
4. Verify the radar module is configured for the correct output format

### False Positives/Negatives

1. Adjust retention times based on your application needs
2. Use `radar_sensor_get_raw_target()` to debug sensor behavior
3. Check the mounting and orientation of the radar module

### UART Communication Issues

1. Verify GPIO pin assignments
2. Check for conflicting UART usage
3. Monitor ESP_LOGE messages for UART driver errors

## License

This component is provided under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please submit pull requests or issues through the GitHub repository.

## References

- **AI-Thinker RD-03D Datasheet**: [Official Technical Documentation](https://docs.ai-thinker.com/_media/rd-03d_specification.pdf)
- **Arduino Implementation Guide**: [Detect and Track Humans with mmWave Radar on Arduino](https://core-electronics.com.au/guides/arduino/detect-and-track-humans-with-mmwave-radar-on-an-arduino/) - Core Electronics tutorial showing similar implementation concepts

## Support

For technical support and questions:

- GitHub Issues: [Repository Issues](https://github.com/heronet/esp_rd-03d/issues)
- ESP32 Community Forums
- Component Registry: heronet/esp_rd-03d
