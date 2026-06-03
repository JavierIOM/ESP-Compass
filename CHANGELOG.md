# Changelog

All notable changes to ESP-Compass are documented here.

## [2.1.0] — 2026-06-03

### Added
- Manual heading trim: +/-1° and +/-5° buttons in the calibration panel to correct small residual north alignment errors without recalibrating
- Trim offset persisted in EEPROM independently of calibration data — survives reboots
- Trim resets to zero when calibration is cleared
- `trim_offset` field added to WebSocket JSON so the UI stays in sync on reconnect

## [2.0.0] — 2026-04-27

### Added
- One-button spin calibration: point north, press Start, spin 360° — auto-completes at ~89% circle coverage
- Hard-iron offsets, soft-iron ellipse correction, and north heading offset all computed from a single spin
- Tilt-compensated north reference: accelerometer captured at button press, accurate even when not perfectly level
- Blinking red UNCALIBRATED warning in UI when no valid calibration is loaded
- `calibrated` field in WebSocket JSON

### Changed
- Roll calculation switched from `asin(ay/cos_pitch)` to `atan2(ay, az)` — stable across full ±180°, no gimbal lock
- EMA filter (alpha=0.2) applied to raw accelerometer and magnetometer inputs before tilt compensation
- Heading smoothing increased from 5 to 10 circular-mean samples
- Needle uses cumulative rotation with shortest-path delta — no 360° wrap-around spin at 0°/360° boundary
- EEPROM magic bumped to 0xCAF4 (backward-compatible load of 0xCAF2 and 0xCAFE)
- JSON buffer increased to 768 bytes

### Fixed
- Sector tracking used absolute `atan2(rawY, rawX)` — large hard-iron offsets collapsed all readings into a narrow arc. Fixed to use centre-relative angle
- Noise gate now requires both X and Y axis ranges to exceed 10 µT before sector counting starts — prevents instant false completion at spin start
- clearCal now writes 0xFFFF magic directly instead of calling saveCalibration, so reboot correctly shows UNCALIBRATED
- Old EEPROM format branch no longer silently loads corrupt data when `loadBase()` fails

## [1.0.1] — 2025-01-01

### Added
- GPS panel with live OpenStreetMap embed
- Toggleable GPS detail panel (DOP values, UTC time, fix quality)
- GPS speed converted from knots to mph

### Changed
- GPS switched from UART to I2C (Adafruit PA1010D via STEMMA QT)
- JSON buffer increased to 512 bytes for GPS fields
- OLED detection now does I2C bus scan before calling `display.begin()` to prevent false positives

### Fixed
- OLED display overlap — now shows only IP address centred at the bottom

## [1.0.0] — 2025-01-01

### Added
- Radar/CRT terminal theme (green-on-black, scanline overlay, glow effects)
- SVG compass with rotating needle using CSS transforms
- Tilt-compensated heading via LSM303AGR accelerometer + magnetometer
- 4-point cardinal calibration (N/E/S/W)
- BME280 environment panel (temperature, humidity, pressure)
- OLED display support (SSD1306)
- WiFi AP mode: SSID `ESP32-Compass`, password `compass123`
- WebSocket at `/ws` for 10Hz real-time updates
- SPIFFS-hosted web UI
