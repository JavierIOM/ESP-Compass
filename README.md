# ESP32 Digital Compass

**Version 2.0.0**

A remote-accessible digital compass built with ESP32 and the Adafruit LSM303AGR accelerometer/magnetometer. Optional support for BME280 (temperature/humidity/pressure), OLED display, and GPS. The ESP32 creates its own WiFi access point — perfect for field use. Access your compass from any device with a real-time web interface.

## Features

- Real-time tilt-compensated compass heading
- 16-point cardinal direction display (N, NNE, NE, etc.)
- One-button spin calibration with hard-iron offset, soft-iron scale, and north alignment
- EMA-filtered sensor inputs for stable readings
- 10-sample circular mean heading smoothing
- Blinking UNCALIBRATED warning when no valid calibration is stored
- Live magnetometer readings (X, Y, Z axes)
- **Optional BME280** for temperature, humidity, and pressure
- **Optional 0.96" OLED display** showing heading, direction, grid square, temperature, WiFi info
- **Optional GPS module** for location and Maidenhead grid square calculation
- WebSocket-based real-time updates (10Hz)
- CRT/radar terminal aesthetic — green on black
- Dark/Light mode toggle (dark mode default)
- Works on any device with a web browser
- Access Point mode — no existing WiFi network required

## Hardware Requirements

### Bill of Materials (UK)

All parts available from **The Pi Hut** (UK):

| Component | Product | Price | Link |
|-----------|---------|-------|------|
| ESP32 Board | FireBeetle ESP32 IoT Microcontroller | ~£8-10 | [The Pi Hut](https://thepihut.com/products/firebeetle-esp32-iot-microcontroller-supports-wi-fi-bluetooth) |
| Sensor | Adafruit LSM303AGR Accelerometer/Magnetometer (ID: 4413) | ~£8-10 | [The Pi Hut](https://thepihut.com/products/adafruit-lsm303agr-accelerometer-magnetometer-stemma-qt-qwiic) |
| Cable | STEMMA QT to Male Header Cable (150mm) | ~£1 | [The Pi Hut](https://thepihut.com/products/stemma-qt-qwiic-jst-sh-4-pin-to-premium-male-headers-cable) |
| USB Cable | USB Cable (Micro-USB or USB-C) | Included | Usually comes with board |
| Environmental Sensor (Optional) | Adafruit BME280 with STEMMA QT (ID: 2652) | ~£15-18 | [The Pi Hut](https://thepihut.com/products/adafruit-bme280-i2c-or-spi-temperature-humidity-pressure-sensor) |
| STEMMA QT Cable (Optional) | STEMMA QT to STEMMA QT Cable (100mm) | ~£1 | [The Pi Hut](https://thepihut.com/products/stemma-qt-qwiic-jst-sh-4-pin-cable-100mm-long) |
| OLED Display (Optional) | 0.96" OLED Display Module (128x64, I2C) | ~£4 | [The Pi Hut](https://thepihut.com/products/0-96-oled-display-module-128x64) |
| GPS Module (Optional) | Adafruit Mini GPS PA1010D (STEMMA QT) | ~£30 | [The Pi Hut](https://thepihut.com/products/adafruit-mini-gps-pa1010d-uart-and-i2c-stemma-qt) |

**Total Cost: ~£17-21** (base) or with optional extras:
- With BME280: ~£33-40
- With OLED: ~£21-25
- With GPS: ~£32-46
- Fully loaded: ~£51-69

### Why FireBeetle ESP32?

- **Low Power:** Ultra-low power consumption design, perfect for battery operation
- **Built-in LiPo Support:** Battery connector with onboard charging circuit
- **Compact Design:** Smaller than standard ESP32 DevKit boards
- **I2C Ready:** GPIO21 (SDA) and GPIO22 (SCL) match our code perfectly

### About the Cable

**IMPORTANT:** The LSM303AGR uses **STEMMA QT** connectors (JST-SH, 1mm pitch).
- Use: STEMMA QT / Qwiic / JST-SH cables
- Don't use: STEMMA (JST-PH, 2mm pitch) — won't fit

## Wiring

All components connect to the FireBeetle ESP32 via I2C. STEMMA QT / Qwiic cables make wiring simple — no soldering required.

```
     ┌─────────────────┐      ┌─────────────────┐
     │  GPS Module     │      │   OLED Display  │
     │  (PA1010D)      │      │   (SSD1306)     │
     └────┬────────────┘      └────┬────────────┘
          │ STEMMA QT (I2C)        │ I2C
          │                        │
          │                        │    ┌─────────────────┐
          │                        │    │     BME280      │
          │                        │    │ (Environmental) │
          │                        │    └────┬────────────┘
          │                        │         │ STEMMA QT (I2C)
          │                        │         │
          │                        │    ┌────▼────────────┐
          │                        │    │   LSM303AGR     │
          │                        │    │   (Compass)     │
          │                        │    └────┬────────────┘
          │                        │         │ STEMMA QT to Headers
          │                        │         │
          └────────────────────────┼─────────┤
                                   │         │
     ┌─────────────────────────────▼─────────▼───┐
     │           FireBeetle ESP32                │
     │  I2C Bus: GPIO21 (SDA), GPIO22 (SCL)      │
     │  Power: 3.3V and GND to all devices       │
     └───────────────────────────────────────────┘
```

### LSM303AGR Compass (required)

| Cable Wire | FireBeetle Pin |
|------------|----------------|
| Black | GND |
| Red | 3.3V |
| Blue | GPIO 21 (SDA) |
| Yellow | GPIO 22 (SCL) |

### BME280, OLED, GPS (optional)

All daisy-chain on the same I2C bus via STEMMA QT cables. No extra wiring to the ESP32.

## Software Setup

### PlatformIO (Recommended)

1. Install [VS Code](https://code.visualstudio.com/) and the PlatformIO IDE extension
2. Open this project folder — PlatformIO auto-detects `platformio.ini`
3. Wait for libraries to download (first time only)

#### Build and Upload

```
# Firmware
pio run -t upload

# Web files (SPIFFS)
pio run -t uploadfs
```

Both must be uploaded. Either order is fine.

#### USB Drivers

Windows 10/11 usually installs drivers automatically. If not:
- CH340: https://sparks.gogo.co.nz/ch340.html
- CP2102: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

## Project Structure

```
ESP-Compass/
├── platformio.ini          # PlatformIO configuration
├── src/
│   └── main.cpp            # Firmware
├── data/
│   └── index.html          # Web interface (uploaded to SPIFFS)
├── case/
│   └── compass_case.scad   # 3D printable case (OpenSCAD)
└── README.md
```

## Usage

### First Use

1. Power on the ESP32 (USB or battery)
2. Connect your device to the **ESP32-Compass** WiFi (password: `compass123`)
3. Open **http://192.168.4.1** in a browser
4. If no calibration has been saved, a blinking red **UNCALIBRATED** warning will show — calibrate before trusting the heading

### Calibration

The compass uses a one-button spin calibration that computes hard-iron offset, soft-iron scale, and north alignment in a single pass.

**Procedure:**

1. Take the device outside away from metal objects
2. Hold it level
3. Point it at magnetic north (use your phone compass if unsure)
4. Press **Start Calibration** in the web UI
5. Slowly rotate the device through a full 360° horizontal spin
6. The progress bar advances as you cover new sectors — it auto-completes at ~89% coverage
7. Calibration is saved to EEPROM and persists across reboots

**Tips:**
- Keep it level throughout — tilt during the spin doesn't matter much, but try to hold level when pressing Start (that's when north is captured)
- Slow is better than fast
- Avoid phones, speakers, and power supplies nearby (30cm+ clearance)

**Clear Calibration:** Press **Clear Calibration** to wipe EEPROM and force recalibration on next boot.

### Tips for Best Results

- Keep the sensor 30cm+ away from speakers, motors, phones, laptops, and power supplies
- Recalibrate if you change environment (e.g., move from indoors to outdoors)
- The OLED display shows the current heading even without a browser connection

### Battery Operation

- LiPo connector: JST 2.0mm on board
- Onboard LiPo charging via USB
- Typical battery life: 8-12 hours with 1000mAh

## 3D Printable Case

A custom case is in the `case/` folder.

- Mounting standoffs for FireBeetle ESP32 (M3) and LSM303AGR (M2.5)
- Flat Li-Po battery compartment (65x36x10mm, 3000mAh)
- USB port cutout
- 5mm perspex/acrylic lid with DXF cutting template

**Print settings:** 0.2mm layer height, 20% infill, no supports, PLA or PETG.

## WebSocket Data Format

JSON sent at 10Hz over `/ws`:

```json
{
  "heading": 245.3,
  "direction": "WSW",
  "mag_x": 23.45,
  "mag_y": -12.34,
  "mag_z": 45.67,
  "cal_state": 0,
  "cal_progress": 0,
  "calibrated": 1,
  "temperature": 22.5,
  "humidity": 45.2,
  "pressure": 1013.25,
  "gps_has_fix": true,
  "gps_lat": 54.1234,
  "gps_lon": -4.5678,
  "gps_sats": 8,
  "gps_alt": 125.5,
  "gps_speed": 0.0,
  "grid_square": "IO74re"
}
```

- `cal_state`: 0 = idle, 1 = spinning
- `cal_progress`: 0–100% coverage of the 360° spin
- `calibrated`: 1 = valid calibration loaded, 0 = uncalibrated
- Environmental and GPS fields only present when those sensors are connected

## Troubleshooting

**"Could not find LSM303AGR"** — check all 4 wire connections; verify cable is STEMMA QT not STEMMA

**Heading is wrong** — run calibration; avoid nearby magnets/metal

**UNCALIBRATED warning showing** — press Start Calibration and do a slow 360° spin

**Calibration stuck at low %** — you may have magnetic interference nearby; move outside and retry

**Can't see ESP32-Compass WiFi** — wait 10 seconds after power-on; press RST if needed

**Page won't load** — confirm you're on the ESP32-Compass network; verify SPIFFS was uploaded

**Upload fails** — check it's a data cable not a charge-only cable; try BOOT button during upload

## Technical Details

**FireBeetle ESP32:** ESP-WROOM-32, dual-core Xtensa LX6, 240MHz, 802.11 b/g/n WiFi, LiPo charging

**LSM303AGR:** ±2/4/8/16g accelerometer, ±50 gauss magnetometer, 16-bit, I2C 0x19/0x1E

**Heading pipeline:**
1. Raw accel + mag read at 10Hz
2. EMA filter (α=0.2) on both axes
3. Pitch = `asin(-ax)`, Roll = `atan2(ay, az)`
4. Tilt-compensated mag components
5. `atan2` for heading
6. Hard-iron offset and soft-iron scale applied
7. 10-sample circular mean smoothing
8. North heading offset applied

## Dependencies

Managed automatically by PlatformIO:

- Adafruit Unified Sensor
- Adafruit LSM303 Accel
- Adafruit LIS2MDL
- Adafruit BME280 Library
- Adafruit SSD1306 + GFX Library
- Adafruit GPS Library
- AsyncTCP
- ESPAsyncWebServer

## Version History

- **v2.0.0** (April 2026) — **Accuracy Overhaul + Spin Calibration**
  - Replaced 4-point N/E/S/W calibration with one-button 360° spin calibration
  - Spin calibration now computes hard-iron offset, soft-iron scale, AND north heading alignment
  - Fixed roll calculation: `asin(ay/cos_pitch)` → `atan2(ay, az)` — stable across ±180°, no gimbal lock
  - Added EMA filter (α=0.2) on raw accel + mag before tilt compensation
  - Increased heading smoothing from 5 to 10 samples
  - Added blinking UNCALIBRATED warning in UI
  - Fixed sector tracking using centre-relative angle (large hard-iron offsets no longer prevent calibration completing)
  - Fixed minimum range gate — noise no longer instantly fills all sectors at spin start
  - Fixed north offset captured with tilt-compensated formula (not flat-plane)
  - Fixed clearCal not persisting — now writes invalid EEPROM magic so reboot correctly shows UNCALIBRATED
  - Fixed needle 360° wrap-around spin using cumulative rotation with shortest-path delta
  - EEPROM magic bumped to `0xCAF4` (backward-compatible with `0xCAF2`, `0xCAFE`)

- **v1.0.1** (January 2025) — **Hardware Tested with GPS**
  - Switched GPS from Serial UART to I2C (Adafruit Mini GPS PA1010D via STEMMA QT)
  - Fixed false OLED detection when display not connected
  - Improved JavaScript robustness

- **v1.0.0** (January 2025) — **First Stable Release**
  - Core features fully tested: compass heading, calibration, BME280 environmental data

- **v0.0.6** (January 2025)
  - Optional OLED display and GPS module support
  - Maidenhead grid square calculation

- **v0.0.5** (January 2025)
  - Optional BME280 environmental sensor
  - Redesigned 3D printed case

- **v0.0.4** (January 2025)
  - Fixed compass needle rotating wrong direction
  - 5-sample circular mean filter

- **v0.0.3** (January 2025)
  - Magnetometer calibration via web interface
  - EEPROM storage

- **v0.0.2** (January 2025)
  - Fixed tilt compensation division-by-zero
  - Pre-allocated JSON buffer

- **v0.0.1** (January 2025)
  - Initial PlatformIO setup, WebSocket, dark/light mode

## License

Open source. Free to modify, distribute, and use for any purpose.

## Credits

Hardware: Espressif (ESP32), STMicroelectronics (LSM303AGR), DFRobot (FireBeetle), Adafruit (breakout boards)

Software: Adafruit sensor libraries, ESPAsyncWebServer/AsyncTCP

---

Built with ESP32 and Adafruit LSM303AGR | April 2026
