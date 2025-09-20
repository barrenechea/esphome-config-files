# ESPHome Configuration Files

This repository contains a collection of ESPHome configuration files for various IoT devices and components of mine. While the main focus for many users will be the OpenHaystack component, this repository also includes configurations for a variety of other devices and sensors.

## OpenHaystack Component

The OpenHaystack component allows ESP32 devices to act as AirTags-compatible trackers, making them visible in Apple's Find My network.

### Usage

To use the OpenHaystack component in your project:

1. Include it as an external component in your YAML configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/barrenechea/esphome-config-files
      ref: main
    components: [openhaystack]
```

2. Add the OpenHaystack configuration to your YAML:

```yaml
openhaystack:
  keys:
    - !secret openhaystack_key
  # rotation_interval: 10min  # Optional: enable when providing multiple advertisement keys
```

3. Make sure to add your base64 advertisement key in your `secrets.yaml` file.

For a complete example, check [openhaystack-demo.yaml](openhaystack-demo.yaml) or [openhaystack-demo-s3.yaml](openhaystack-demo-s3.yaml).

### Component Files

- [`components/openhaystack/__init__.py`](components/openhaystack/__init__.py) - Component initialization
- [`components/openhaystack/openhaystack.h`](components/openhaystack/openhaystack.h) - Header file
- [`components/openhaystack/openhaystack.cpp`](components/openhaystack/openhaystack.cpp) - Implementation

## Other Components

### Desk Component

A custom component for controlling a height-adjustable desk of mine. It is incomplete, I may give it a proper implementation in the future.

- [`components/desk/__init__.py`](components/desk/__init__.py)
- [`components/desk/desk_height_sensor.h`](components/desk/desk_height_sensor.h)
- [`components/desk/desk_keypad.h`](components/desk/desk_keypad.h)

## Device Configurations

### Boards

This repository includes configurations for multiple types of ESP boards:

- ESP32 (Wemos D1 Mini32)
- ESP32-C3
- ESP32-C6
- ESP32-S3
- ESP8266/ESP8285

### Sensors & Controllers

- [Air Conditioner Remote](air-conditioner-remote.yaml) - ESP32-based IR remote controller
- [Air Sensor](air-sensor.yaml) - CO₂, temperature, and humidity sensor
- [Atlas Sensor](atlas-sensor.yaml) - pH, EC, and temperature sensors
- [BentoBox](bentobox.yaml) - Fan and light controller
- [Bowflex Heart Rate Monitor](bowflex-hr-monitor.yaml) - Bluetooth heart rate sensor
- [Peristaltic Pumps](peristaltic-main.yaml) - Precise fluid control pumps

### Smart Plugs & Relays

- Sonoff POWR2 - Wi-Fi smart plug with power monitoring
- Sonoff S26 - Wi-Fi smart plug
- Shelly Plus 1PM Mini - Wi-Fi relay with power monitoring
- Shelly Plus PM Mini - Wi-Fi power monitor

### Other Devices

- [Universal IR](universal-ir.yaml) - Infrared remote transmitter
- [Water Valve](water-valve.yaml) - Water valve controller - Just a relay to trigger a motorized self-closing valve

## Common Configuration

The repository uses a modular approach with common configurations stored in the `common/` directory:

- `common/board/` - Board-specific configurations
- `common/core/` - Core functionality (Wi-Fi, API, OTA, logging)
- `common/devices/` - Device-specific configurations
- `common/time/` - Time synchronization
