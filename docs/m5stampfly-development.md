# M5StampFly Development Branch

This document describes the M5StampFly development work being integrated by
mura with Hal, an AI coding assistant.

The branch used for this work is `stampfly-dev`.

## Purpose

M5StampFly is a very small and light indoor drone based on ESP32-S3. It has
propeller guards, push-fit propellers, and onboard sensors that make it a good
platform for learning and experimenting with ArduPilot.

In Japan, there are few readily available small aircraft for learning ArduPilot
flight. This work explores M5StampFly as a practical, educational, and
experimental ArduPilot aircraft.

## Branch Policy

- Keep upstream pull requests independent where possible.
- Use `stampfly-dev` as the integration branch for M5StampFly development.
- Prefer board-specific configuration in `hwdef.dat` when the change is only
  needed by M5StampFly.
- Send generally useful fixes upstream as separate pull requests.
- Keep the firmware small enough for ESP32-S3 while retaining useful ArduPilot
  features.

## Hardware Focus

The current work focuses on the onboard M5StampFly hardware and useful Grove
extensions. Detailed schematics, pinouts, and sensor register maps are documented in
[stampfly-hardware.md](stampfly-hardware.md).

- BMI270 IMU
- BMP280 barometer
- BMM150 compass
- VL53L3CX distance sensors (Dual: Downward 0x30, Forward 0x29)
- INA3221 battery monitor
- WS2812C RGB LEDs
- GPIO40 ToneAlarm buzzer
- PMW3901 / CXOF optical flow
- Optional Grove-connected u-blox GPS such as SAM-M8Q

## Current Build Direction

The M5StampFly build is being slimmed by disabling libraries and source files
that are not used on this airframe.

Current choices include:

- Keep MAVLink control and telemetry over Wi-Fi.
- Keep FlashFS and Lua scripting for experimentation.
- Keep EKF3.
- Keep 6DoF support for now.
- Remove helicopter-specific build sources.
- Remove unused device backends where practical.
- Keep only the optical flow backends needed for CXOF and MAVLink flow.
- Keep only the GPS backends needed for MAVLink GPS and optional u-blox GPS.

## Confirmed So Far

- Copter builds successfully for `esp32s3m5stampfly`.
- WS2812C LED output works.
- ToneAlarm buzzer output works.
- MAVLink telemetry over Wi-Fi works.
- VL53L3CX distance sensor data is available.
- Optical flow data can be received.
- BMM150 compass is detected.
- BMI270 IMU is detected.
- BMP280 barometer is detected.
- INA3221 battery monitor is detected.

## Build Command

```bash
./waf configure --board esp32s3m5stampfly
./waf copter -j1
```
