# CAN Pulse

## Overview

A simple ESP32-based device that listens to vehicle CAN bus traffic and forwards selected telemetry data to external apps over Bluetooth Low Energy (BLE). It can also integrate an external GPS receiver for position, speed, heading, and timing data.
The primary target is [RaceChrono](https://racechrono.com), using its DIY BLE CAN device protocol, but the platform can be extended for other telemetry and logging uses.

Designed as a lightweight embedded bridge, the device focuses on low-latency signal delivery, configurable data forwarding, and optional high-rate GPS integration. With supported modules, GPS update rates can significantly exceed those commonly seen on smartphones, improving track mapping and speed accuracy.


## Motivation

This project was originally developed for the 2022+ Toyota GR86 / Subaru BRZ (ZN8/ZD8) platform.

The device taps into the ASC (Active Sound Control, “GazooKazoo”) connector, which passively listens to the vehicle CAN network to generate synthetic engine sound. Since this module is non-essential and commonly disconnected, it provides a convenient and non-invasive access point to the CAN bus.

RaceChrono supports [DIY BLE CAN devices](https://github.com/aollin/racechrono-ble-diy-device), which made it possible to build a custom telemetry pipeline tailored to the vehicle and signals of interest.

It is also a practical exercise in understanding automotive CAN networks and building something functional, transparent, and extensible rather than relying on closed black-box hardware.


## Feature

### Real-time CAN ingestion
Continuously reads CAN frames from the vehicle bus via the ASC connector.

### Adaptive PID filtering
Dynamically responds to RaceChrono filter requests, serving only the required signals.

### Efficient BLE transport
Streams CAN data over BLE using the RaceChrono DIY device protocol.

### High-throughput notifications

Achieves an aggregate BLE notify rate of ~300 Hz, balancing bandwidth across active signals.

### Near-native signal rates
Individual PIDs are delivered at or close to their original CAN bus update frequencies.

### Latest-value delivery model
Prioritises fresh data — each frame is sent once, avoiding replay of stale values.

### Integrated high-rate GPS
Provides GPS data at up to 50 Hz, significantly outperforming typical phone-based GPS (~1 Hz).

### Notes on RaceChrono BLE API
This firmware intentionally does not fully follow the RaceChrono DIY BLE CAN device API. In particular, it does not honour the requested notify interval in the [CAN-Bus filter allow one PID](https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file#allow-one-pid) message.

RaceChrono consistently requests a 50 ms interval (20 Hz), which is much lower than the native update rate of many vehicle signals. In practice, the app appears to accept updates faster than the requested interval without issue.

To preserve signal fidelity, this firmware sends CAN frames at or near their native rate instead of throttling them to the requested 20 Hz.


## Hardware

### [ASL ESP-CAN-X2](https://wiki.autosportlabs.com/ESP32-CAN-X2)
### [ASL Gps-bolt-on](https://wiki.autosportlabs.com/Gps-bolt-on)

## Sounds pretty neat, how do I make my own?

### Assembly

1. Assemble the hardware following the Autosport Labs guides for the ESP32-CAN-X2 and GPS Bolt-on modules.
1. Use an extension cable between the GPS module and bolt-on board so the GPS receiver can be mounted on the dashboard or another location with clear sky visibility.
1. Connect the `X1` JST-PH 4-pin header to vehicle CAN High, CAN Low, 12V power, and ground.
1. Powering from an ignition-switched source is recommended so the device turns on only when the vehicle is in accessory or running mode.

### Flashing the firmware

1. Install [PlatformIO CLI](https://platformio.org/install/cli) or [PlatformIO IDE](https://platformio.org/install/ide).
1. Clone this repository.
1. From the project directory, run:
   ```bash
   platformio run --target upload --environment esp32-can-x2
   ```

### Connecting to RaceChrono
1. Power on the device.
1. In RaceChrono, open `Settings` -> `Other devices` -> `Add other device`
1. Select `RaceChrono DIY` from the list, then `Bluetooth LE`.
1. Make sure the device is turned on, tap `Search for devices in range`.
1. Select `CAN Pulse BLE`, check the options `GPS` and `CAN-Bus`, tap `ok` to connect.
1. If `CAN Pulse BLE` does not appear, unplug the device from its power source and start again.

### Vehicle integration notes

- CAN bus pinout, connector location, and available power sources vary by vehicle.
- Always verify wiring before connecting the device.
- If unsupported CAN transceivers or constant battery power are used incorrectly, battery drain or communication faults may occur.

### BOM

| Part# | Quantity |
| -------- | -------- |
| [ASL ESP-CAN-X2](https://wiki.autosportlabs.com/ESP32-CAN-X2) | 1 |
| [ASL Gps-bolt-on](https://wiki.autosportlabs.com/Gps-bolt-on) | 1 |
| PHR-4 | 1 |
| XHR-7 | 1 |
| SPH-001T-P0.5L | 4 |
| SXH-002TP0.6| 7 |
| CDM810-04A-MW-F810-050-67 | 1 |
| CDM810-08A-MW-F810-050-67 | 1 |
| M8 A-coded 4pin female to open end cable | 1 |
| M8 A-coded 8pin female to open end cable | 1 |
| Accessing CAN bus via GR86/BRZ ASC port |
| 1376106-1 | 1 |
| 1376109-1 | 4 |

## References

This project is heavily based on reverse engineering work documented by the community, in particular:
- https://github.com/timurrrr/ft86/blob/main/can_bus/gen2.md

That document provides a detailed breakdown of the Gen2 GR86/BRZ CAN bus signals and effectively serves as the foundation for this project. It guided the identification of useful frames, signal meanings, and overall approach to decoding and forwarding data.

Without that work, building a usable and accurate telemetry pipeline would be significantly more time-consuming.

## Oil Pressure Oil Temperature Bolt-on

### Status

Abandon for now or at least deprioritised, due to lack of PCB design skills.

<details>

<summary>details</summary>

### Motivation

The factory setup leaves much to be desired:
- The gauge cluster on BRZ lacks a proper, high-resolution oil temperature gauge.
- Oil pressure data is absent.

This project aims to fill that gap by:
- Forwarding CAN data from the ASC (Active Sound Control) unit to a new CAN output.
- Reading engine oil pressure using a Bosch analog fluid pressure sensor (0 to 10 bar).
- Reading engine oil temperature using a Bosch analog fluid temperature sensor (-40 to 150 °C).
- Injecting that data into the new CAN bus stream so that digital dashboards and logging tools (e.g. RaceChrono) can display it.

Accurate oil pressure monitoring is critical in performance and track environments — especially given the FA24 platform’s known vulnerability to oil starvation during high-G right-hand turns.

### Features

- Dual-CAN forwarding: Reads from CAN1 (TWAI), writes to CAN2 (MCP2515).
- Sensor integration: Adds real-time oil pressure and temperature data (40Hz sampling rate, 20Hz update rate).
- Plug-and-play install: Designed to interface cleanly with the ASC connector and ESP32.

### Hardware

#### [Custom ASL ESP32-CAN-X2 Bolt-on](./bolt-on-bosch-pst-f-1/)
#### [Bosch Pressure Sensor Combined PST-F 1](https://www.bosch-motorsport.com/content/downloads/Raceparts/en-GB/54249355.html)

</details>

## License

- **Firmware** (ESP32 code): [GPLv3](./LICENSE)
- **Hardware design** (schematics, PCB layout): [CERN-OHL-S v2](./bolt-on-bosch-pst-f-1/LICENSE)
