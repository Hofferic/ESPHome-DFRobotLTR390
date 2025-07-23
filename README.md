# ESPHome DFRobot LTR390 Custom Component

# ! WARNING: full-on vibe coding and the code is suboptimal, this _seems_ to work correctly though

This ESPHome custom component provides support for the DFRobot Gravity LTR390 UV and ambient light sensor (SEN0540).

## Features

- Ambient light measurement (lux)
- UV index measurement
- Configurable gain (1x, 3x, 6x, 9x, 18x)
- Configurable resolution (13-20 bit)
- Configurable measurement rate (25ms to 2000ms)
- Native ESPHome integration with proper configuration schema

## Hardware

- **Sensor**: DFRobot Gravity LTR390 (SEN0540)
- **Interface**: I2C
- **Default Address**: 0x1C
- **Voltage**: 3.3V - 5V
- **Current**: ~0.65mA (typical)

## Installation

Add the external component to your ESPHome configuration:

```yaml
external_components:
  - source: github://Hofferic/ESPHome-DFRobotLTR390
    components: [ dfrobot_ltr390 ]
```

## Configuration

```yaml
# I2C bus configuration
i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true
  frequency: 400000

# DFRobot LTR390 hub
dfrobot_ltr390:
  - id: my_ltr390
    address: 0x1C  # Default address

# Sensors
sensor:
  - platform: dfrobot_ltr390
    dfrobot_ltr390_id: my_ltr390
    update_interval: 60s
    
    ambient_light:
      name: "Ambient Light"
      unit_of_measurement: "lx"
      
    uv_index:
      name: "UV Index"
      
    # Optional configuration
    gain: 3                    # Options: 1, 3, 6, 9, 18
    resolution: 18             # Options: 13, 16, 17, 18, 19, 20
    measurement_rate: 100ms    # Options: 25ms, 50ms, 100ms, 200ms, 500ms, 1000ms, 2000ms
```

## Configuration Options

### Hub Configuration (`dfrobot_ltr390`)

- **id** (*Required*): The component ID
- **address** (*Optional*, int): I2C address (default: 0x1C)

### Sensor Configuration

- **dfrobot_ltr390_id** (*Required*): The hub component ID
- **update_interval** (*Optional*, time): Update interval (default: 60s)
- **ambient_light** (*Optional*): Ambient light sensor configuration
- **uv_index** (*Optional*): UV index sensor configuration
- **gain** (*Optional*, int): Sensor gain (1, 3, 6, 9, 18) (default: 3)
- **resolution** (*Optional*, int): ADC resolution in bits (13-20) (default: 18)
- **measurement_rate** (*Optional*, time): Measurement rate (25ms to 2000ms) (default: 100ms)

## Wiring

| DFRobot LTR390 | ESP32 | ESP8266 |
|---------------|-------|---------|
| VCC | 3.3V | 3.3V |
| GND | GND | GND |
| SDA | GPIO21 | GPIO4 (D2) |
| SCL | GPIO22 | GPIO5 (D1) |

**Note**: Make sure the I2C/UART switch on the board is set to **I2C** mode.

## Troubleshooting

1. **Sensor not detected**: Check wiring and ensure the I2C/UART switch is in I2C position
2. **Component marked failed**: Verify the sensor is properly powered and the I2C address is correct
3. **Inconsistent readings**: Try adjusting the gain and measurement rate settings
4. **I2C errors**: Add pull-up resistors (4.7kΩ) to SDA and SCL lines if needed

## Technical Details

- Based on the official DFRobot library implementation at https://github.com/cdjq/DFRobot_LTR390UV
- Uses proper ESPHome component architecture
- Implements asynchronous reading to avoid blocking
- Includes comprehensive error handling and logging
- Compatible with Home Assistant auto-discovery

This component is released under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
