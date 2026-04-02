# CAN Pulse

## Overview

A simple ESP32-based device that listens to and forwards CAN bus data from the 2022+ Toyota GR86 / Subaru BRZ (ZN8/ZD8).
The goal is to make CAN bus data available in [RaceChrono](https://racechrono.com).

The device taps into the ASC (“GazooKazoo”) connector, which passively listens to the vehicle CAN network to generate synthetic engine noise. Since this module is non-essential and commonly disconnected, it provides a convenient and non-invasive access point to the CAN bus.

## Motivation

RaceChrono supports [DIY BLE CAN devices](https://github.com/aollin/racechrono-ble-diy-device), which opens the door to building a fully custom telemetry pipeline. Instead of relying on off-the-shelf hardware, this project aims to create a lightweight, purpose-built device that integrates cleanly with the car and exposes exactly the data needed.

It’s also a practical exercise in understanding the vehicle’s CAN network and building something that is both functional and extensible, rather than a black box.

## Roadmap

### Current Status

The basic firmware is complete and functional:
- CAN frames are read from the vehicle bus.
- Frames are cached and served according to filter requests issued by RaceChrono.
- Filter requests are driven by formulas configured in RaceChrono, allowing selective forwarding of relevant PIDs.
- CAN frames are transmitted over BLE using the RaceChrono DIY device protocol.
- BLE notifications are sent on a best effort, latest value basis, updated frames are sent once, without replaying stale data.
- The system achieves an overall BLE notify rate of ~300 Hz, with individual PIDs delivered at close to their native bus update rates where possible.

This means the device is already usable for real-time telemetry in RaceChrono.

### Next Step: High-Rate GPS Integration

The next major milestone is adding high-frequency GPS data support using the [ASL GPS bolt-on](https://wiki.autosportlabs.com/Gps-bolt-on)

Planned capabilities:
- Integrate GPS data alongside CAN telemetry.
- 10-25 Hz GPS update rate(vs typical phone GPS at 1 Hz)
- Improve lap timing accuracy and track position fidelity.

Longer term, this will allow the device to act as a complete standalone telemetry unit, rather than relying on the phone for positioning data.

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
