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


## Hardware

### [ASL ESP-CAN-X2](https://wiki.autosportlabs.com/ESP32-CAN-X2)
### [ASL Gps-bolt-on](https://wiki.autosportlabs.com/Gps-bolt-on)

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
