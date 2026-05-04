# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OpenDTU-OnBattery is an ESP32-based firmware (fork of OpenDTU) that monitors Hoymiles solar inverters, manages battery systems via multiple BMS protocols, and implements dynamic power limiting for zero-export policies. It embeds a Vue.js SPA into ESP32 flash memory.

## Build Commands

**The webapp MUST be built before firmware. Never skip this step.**

```bash
# Build webapp first
cd webapp
yarn install --frozen-lockfile
yarn build

# Build firmware (default environment)
cd ..
pio run -e generic_esp32s3_usb

# Flash to device
pio run -e generic_esp32s3_usb -t upload
```

Firmware build takes 10–45 minutes on first run (downloads ESP32 platform + libraries). Requires unrestricted internet access. `HTTPClientError` during `pio run` means network restrictions are blocking PlatformIO downloads.

### Available Build Environments

```
generic_esp32_4mb_no_ota   # 4MB ESP32, no OTA
generic_esp32_8mb          # 8MB ESP32 with OTA
generic_esp32s3            # ESP32-S3
generic_esp32s3_usb        # ESP32-S3 with USB (default)
```

See `platformio.ini` for the full list including Olimex and Fusion board variants.

### Webapp Development

```bash
cd webapp
yarn dev      # Dev server
yarn preview  # Production preview on port 4173
```

## Linting and Validation

```bash
# Webapp
cd webapp
yarn lint                   # oxlint + eslint (auto-fixes)
yarn prettier --check src/  # Check formatting

# C++ (from repo root)
cpplint --repository=. --recursive \
  --filter=-build/c++11,-runtime/references,-readability/braces,-whitespace,-legal,-build/include \
  ./src ./include ./lib/Hoymiles ./lib/MqttSubscribeParser ./lib/TimeoutHelper ./lib/ResetReason
```

## Tests

There are no unit tests. The `test/` directory is empty except for documentation.

## Architecture

### Firmware (`src/` + `include/`)

C++17, Arduino framework on ESP32. Entry point: `src/main.cpp`.

Major subsystems:
- **`PowerLimiter.cpp`** — Core dynamic power limiting logic (largest file, ~38KB)
- **`Configuration.cpp`** — All persistent config management (~64KB)
- **`battery/`** — BMS drivers: JK BMS, Pylontech, Victron SmartShunt, Pytes, Zendure, SBS
- **`solarcharger/`** — Solar charger drivers: Victron MPPT, MQTT-based
- **`gridcharger/`** — Grid charger drivers: Huawei, Trucki
- **`powermeter/`** — Power meter drivers: SMA HM, SDM, SML, Modbus, JSON HTTP

### External Libraries (`lib/`)

- `Hoymiles/` — Hoymiles RF inverter protocol implementation
- `VeDirectFrameHandler/` — Victron VE.Direct serial protocol
- `SdmEnergyMeter/` — SDM power meter
- `SMLParser/` — Smart Meter Language parser
- `CMT2300a/` — CMT2300A radio module driver

### Web Application (`webapp/`)

Vue 3.5 + TypeScript + Vite. Entry point: `webapp/src/main.ts`. Built output goes to `webapp_dist/` and is gzipped into ESP32 flash via `platformio.ini` board resources.

Structure:
- `src/views/` — Page-level components (~28 views)
- `src/components/` — Reusable UI components
- `src/types/` — TypeScript type definitions (mirrors firmware config structs)
- `src/locales/` — i18n translation files

### Build Automation (`pio-scripts/`)

- `compile_webapp.py` — PlatformIO pre-script; rebuilds webapp automatically if stale
- `auto_firmware_version.py` — Injects git version into firmware
- `create_factory_bin.py` — Produces combined factory flash binary

## Key Constraints

- **4MB ESP32 boards cannot do OTA** — firmware footprint exceeds half of 4MB flash
- **Avoid inverter firmware v2.0.4** — breaks Power Distribution Logic
- C++ standard: C++17 with strict warnings
- Node.js 24 required for webapp builds (`corepack enable` for yarn)
