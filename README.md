# ESP32 Digital Compass

**Version 0.0.5** (Hardware Tested)

A remote-accessible digital compass built with ESP32 and the Adafruit LSM303AGR accelerometer/magnetometer sensor. Optional BME280 sensor support for temperature, humidity, and pressure readings. The ESP32 creates its own WiFi access point, perfect for field use. Access your compass from any device with a beautiful, real-time web interface.

## Features

- Real-time compass heading with tilt compensation
- 16-point cardinal direction display (N, NNE, NE, etc.)
- Live magnetometer readings (X, Y, Z axes)
- **Optional BME280 sensor** for temperature, humidity, and pressure
- WebSocket-based real-time updates (10Hz)
- Responsive web interface with animated compass needle
- Dark/Light mode toggle (dark mode default)
- Works on any device with a web browser
- Ultra-low power consumption with battery support
- Access Point mode - no existing WiFi network required (perfect for field use)

## Hardware Requirements

### Bill of Materials (UK)

All parts available from **The Pi Hut** (UK):

| Component | Product | Price | Link |
|-----------|---------|-------|------|
| ESP32 Board | FireBeetle ESP32 IoT Microcontroller | ~£8-10 | [The Pi Hut](https://thepihut.com/products/firebeetle-esp32-iot-microcontroller-supports-wi-fi-bluetooth) |
| Sensor | Adafruit LSM303AGR Accelerometer/Magnetometer (ID: 4413) | ~£8-10 | [The Pi Hut](https://thepihut.com/products/adafruit-lsm303agr-accelerometer-magnetometer-stemma-qt-qwiic) |
| Cable | STEMMA QT to Male Header Cable (150mm) | ~£1 | [The Pi Hut](https://thepihut.com/products/stemma-qt-qwiic-jst-sh-4-pin-to-premium-male-headers-cable) |
| USB Cable | USB Cable (Micro-USB or USB-C) | Included | Usually comes with board |
| Environmental Sensor (Optional) | BME280 Temperature/Humidity/Pressure | ~£5-8 | [The Pi Hut](https://thepihut.com/products/bme280-breakout-temperature-pressure-humidity-sensor) |

**Total Cost: ~£17-21** (or ~£22-29 with optional BME280)

### Why FireBeetle ESP32?

- **Low Power:** Ultra-low power consumption design, perfect for battery operation
- **Built-in LiPo Support:** Battery connector with onboard charging circuit
- **Compact Design:** Smaller than standard ESP32 DevKit boards
- **Quality Components:** DFRobot quality, more reliable than generic clones
- **I2C Ready:** GPIO21 (SDA) and GPIO22 (SCL) match our code perfectly

### About the Cable

**IMPORTANT:** The LSM303AGR uses **STEMMA QT** connectors (JST-SH, 1mm pitch).
- ✅ Use: STEMMA QT / Qwiic / JST-SH cables
- ❌ Don't use: STEMMA (JST-PH, 2mm pitch) - won't fit!

The recommended cable has:
- JST-SH female connector (plugs into sensor)
- Male jumper wires (plug into FireBeetle headers)
- 150mm length (perfect for this project)

## Wiring

### No Soldering Required!

The LSM303AGR connects to the FireBeetle ESP32 using the STEMMA QT cable:

```
┌─────────────────┐
│   LSM303AGR     │
│  (Sensor Board) │
└────┬────────────┘
     │ STEMMA QT Port (JST-SH connector)
     │
     └─── STEMMA QT to Male Header Cable
              │
              ├─ Black  → GND
              ├─ Red    → 3.3V (or V pin)
              ├─ Blue   → GPIO21 (SDA)
              └─ Yellow → GPIO22 (SCL)
                    │
          ┌─────────▼──────────┐
          │  FireBeetle ESP32  │
          │   Pin Headers      │
          └────────────────────┘
```

### Pin Connections

| LSM303AGR (Cable) | FireBeetle ESP32 Pin | Notes |
|-------------------|----------------------|-------|
| Black (GND) | GND | Ground |
| Red (VIN) | 3.3V or V | Power supply |
| Blue (SDA) | GPIO 21 (SDA) | I2C Data |
| Yellow (SCL) | GPIO 22 (SCL) | I2C Clock |

**Note:** The LSM303AGR board has voltage regulation and works with both 3.3V and 5V input. The FireBeetle ESP32 GPIO pins are 3.3V logic level.

### Optional: BME280 Environmental Sensor

The BME280 provides temperature, humidity, and pressure readings. It connects to the same I2C bus:

| BME280 Pin | FireBeetle ESP32 Pin | Notes |
|------------|----------------------|-------|
| VIN | 3.3V | Power supply |
| GND | GND | Ground |
| SDA | GPIO 21 (SDA) | I2C Data (shared with LSM303) |
| SCL | GPIO 22 (SCL) | I2C Clock (shared with LSM303) |

**Note:** The BME280 is optional. If not connected, the compass works normally without environmental data.

## Software Requirements

### PlatformIO Setup (Recommended)

This project uses PlatformIO for building and uploading.

#### 1. Install VS Code and PlatformIO

1. Download and install [VS Code](https://code.visualstudio.com/)
2. Open VS Code
3. Go to Extensions (Ctrl+Shift+X)
4. Search for "PlatformIO IDE"
5. Install the PlatformIO IDE extension
6. Restart VS Code when prompted

#### 2. Open the Project

1. Open VS Code
2. Click "Open Folder" and select this project folder
3. PlatformIO will automatically detect `platformio.ini` and configure the project
4. Wait for PlatformIO to download required libraries (first time only)

#### 3. USB Drivers (Usually Automatic)

Windows 10/11 usually installs drivers automatically when you plug in the FireBeetle.

If the board isn't recognized:
- **CP2102 Driver:** https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- **CH340 Driver:** https://sparks.gogo.co.nz/ch340.html

## Project Structure

```
ESP-Compass/
├── platformio.ini          # PlatformIO configuration
├── src/
│   └── main.cpp            # Main application code
├── data/
│   └── index.html          # Web interface (uploaded to SPIFFS)
├── case/
│   └── compass_case.scad   # 3D printable case (OpenSCAD)
└── README.md
```

## 3D Printable Case

A custom 3D printable case is included in the `case/` folder.

**Features:**
- Mounting standoffs for FireBeetle ESP32 (M3 screws)
- Mounting standoffs for LSM303AGR sensor (M2.5 screws)
- Flat Li-Po battery compartment (65x36x10mm, 3000mAh)
- Separate platform piece for easy assembly
- USB port cutout
- 5mm perspex/acrylic lid with DXF cutting template

**Print Settings:**
- Layer height: 0.2mm
- Infill: 20%
- Supports: Not required
- Material: PLA or PETG

**Files:**
- `compass_case.scad` - OpenSCAD source (parametric, customizable)
- `lid_template.dxf` - DXF file for laser cutting the perspex lid
- STL files available in releases

**Hardware needed for case:**
- 4x M3 screws (6-8mm) for FireBeetle
- 4x M2.5 screws (6-8mm) for LSM303AGR
- 1x 3000mAh flat Li-Po battery (65x36x10mm)
- 1x 5mm clear perspex/acrylic sheet (78x62mm with 4mm corner radius)

## Installation

### 1. Configure Access Point (Optional)

The ESP32 creates its own WiFi network. You can customize the name and password by editing `src/main.cpp` lines 12-13:

```cpp
const char* ap_ssid = "ESP32-Compass";     // The WiFi network name
const char* ap_password = "compass123";     // Password (min 8 chars, or "" for open network)
```

**Default settings work great for most users!**

### 2. Build the Project

In VS Code with PlatformIO:

1. Click the PlatformIO icon in the sidebar (alien head icon)
2. Under "Project Tasks" → "firebeetle32":
   - Click **Build** to compile the project

Or use the keyboard shortcut: `Ctrl+Alt+B`

### 3. Upload the Web Interface (SPIFFS)

The web files must be uploaded separately to the ESP32's filesystem:

1. Connect the FireBeetle ESP32 via USB
2. In PlatformIO sidebar, click **Upload Filesystem Image**
   - This uploads everything in the `data/` folder to SPIFFS

**Important:** Do this BEFORE or AFTER uploading the main firmware - both are required!

### 4. Upload the Firmware

1. In PlatformIO sidebar, click **Upload**
2. Wait for compilation and upload to complete
3. The serial monitor will show upload progress

Or use the keyboard shortcut: `Ctrl+Alt+U`

### 5. Connect to the Compass

After uploading, the ESP32 creates its own WiFi network:

1. Open Serial Monitor (PlatformIO sidebar → Monitor, or `Ctrl+Alt+S`)
2. Press the **RST** button on the ESP32
3. You'll see:
   ```
   ESP32 Digital Compass
   =====================
   LSM303AGR sensors initialized successfully!
   Starting Access Point...
   Access Point Started!
   AP SSID: ESP32-Compass
   AP Password: compass123
   IP Address: 192.168.4.1
   Web server started!
   Connect to WiFi network: ESP32-Compass
   Then visit: http://192.168.4.1
   ```

4. On your phone/tablet/laptop:
   - Open WiFi settings
   - Connect to network: **ESP32-Compass**
   - Enter password: **compass123**
   - Wait for connection

5. Open any web browser and navigate to: **http://192.168.4.1**

The compass interface will load immediately!

## Usage

### First Use

1. Power on the ESP32 (via USB or battery)
2. Wait ~5 seconds for the Access Point to start
3. Connect your device to the **ESP32-Compass** WiFi network
4. Open your browser to **http://192.168.4.1**
5. The compass needle should start moving immediately
6. Rotate the sensor to test - the needle follows magnetic north

**Perfect for field use:** The ESP32 works anywhere, no internet or existing WiFi needed!

### Calibration

The compass includes an automatic calibration feature for accurate readings.

**Calibration Procedure:**

1. Open the compass web interface at **http://192.168.4.1**
2. Click the **"Calibrate"** button
3. During the 15-second countdown:
   - Slowly rotate the sensor in all directions
   - Complete 360° horizontal rotation
   - Tilt up and down
   - Roll side to side
   - Figure-8 pattern works well
4. Calibration completes automatically
5. Offsets are saved to EEPROM and persist across reboots

**Clear Calibration:**
- Click the **"Clear Calibration"** button to reset offsets to zero

### Tips for Best Results

- **Level mounting:** Keep sensor relatively level (within ±30°)
- **Avoid interference:** Keep 30cm+ away from:
  - Speakers/magnets
  - Motors
  - Large metal objects
  - Phones/laptops
  - Power supplies
- **Stable mounting:** Secure the sensor to avoid vibration

### Battery Operation

The FireBeetle ESP32 has excellent battery support:

- **LiPo Connector:** JST 2.0mm connector on board
- **Charging:** Onboard LiPo charging via USB
- **Battery Life:** 8-12 hours typical with 1000mAh battery

## Web Interface Features

### Compass Display
- **Animated compass rose** with 60 degree marks
- **Cardinal directions** (N, E, S, W) highlighted in red
- **Red needle** points to magnetic north
- **Smooth rotation** with CSS transitions
- **Responsive design** - works on phone, tablet, desktop

### Data Panel
- **Large heading display** - current cardinal direction
- **Numeric heading** - degrees (0-360°)
- **Magnetometer data** - X, Y, Z axes in µT (microtesla)
- **Real-time updates** - 10 times per second

### Theme Toggle
- **Dark mode** (default) - easy on the eyes
- **Light mode** - for bright environments

### Connection Status
- **Green pulsing dot** - Connected to ESP32
- **Red pulsing dot** - Disconnected (auto-reconnects)

## Troubleshooting

### Hardware Issues

**"Could not find LSM303AGR" Error**
- Check all 4 wire connections (GND, VIN, SDA, SCL)
- Verify sensor power - look for a small LED on the sensor board
- Ensure cable is fully inserted into STEMMA QT port
- Try the other STEMMA QT port on the sensor

**Compass Points Wrong Direction**
- Perform calibration procedure (see Usage section)
- Check for magnetic interference nearby

### Software Issues

**Can't See ESP32-Compass WiFi Network**
- Wait 10 seconds after powering on
- Check Serial Monitor - does it say "Access Point Started!"?
- Press the RST button on the ESP32

**Web Page Won't Load**
- Ensure you're connected to the **ESP32-Compass** WiFi network
- Try the default IP: **http://192.168.4.1**
- Verify SPIFFS upload was successful

**Page Loads But Shows "Disconnected"**
- Check Serial Monitor - is the ESP32 still running?
- Try refreshing the page

### Upload Issues

**"Failed to connect to ESP32"**
- Check USB cable (some are power-only, need data cable)
- Try a different USB port
- Press and hold BOOT button while uploading

## Technical Details

### Hardware Specifications

**FireBeetle ESP32:**
- Microcontroller: ESP-WROOM-32 (Dual-core Xtensa LX6)
- Clock Speed: Up to 240MHz
- WiFi: 802.11 b/g/n (2.4GHz)
- Power: Ultra-low consumption design, LiPo charging circuit

**Adafruit LSM303AGR:**
- Accelerometer Range: ±2g / ±4g / ±8g / ±16g (selectable)
- Magnetometer Range: ±50 gauss
- Resolution: 16-bit
- I2C Addresses: 0x19 (accelerometer), 0x1E (magnetometer)

### Communication Protocol

- WebSocket connection on port 80
- JSON data format:
  ```json
  {
    "heading": 245.3,
    "direction": "WSW",
    "mag_x": 23.45,
    "mag_y": -12.34,
    "mag_z": 45.67,
    "temperature": 22.5,
    "humidity": 45.2,
    "pressure": 1013.25
  }
  ```
- Environmental data (temperature, humidity, pressure) only included when BME280 is connected
- Update frequency: 10Hz (100ms intervals)

## Customization

### Change Update Rate

In `src/main.cpp`, line 25:
```cpp
const unsigned long updateInterval = 100; // milliseconds

// Examples:
// 50ms = 20Hz (very smooth, more power)
// 100ms = 10Hz (current setting, balanced)
// 200ms = 5Hz (slower, saves power)
```

### Modify Compass Appearance

Edit `data/index.html` CSS variables to customize colors and styling.

## Version History

- **v0.0.5** (January 2025) - **Hardware Tested**
  - **New Features:**
    - Optional BME280 sensor support for temperature, humidity, and pressure
    - Sensor auto-detection - works with or without BME280 connected
    - Environmental data panel in web UI (only shown when sensor present)
  - **Case Design:**
    - Redesigned case for flat 65x36x10mm Li-Po battery (3000mAh)
    - Separate platform piece for easier assembly
    - 5mm perspex/acrylic lid option with DXF cutting template

- **v0.0.4** (January 2025) - **Hardware Tested**
  - **Bug Fixes:**
    - Fixed compass needle rotating in wrong direction (inverted heading)
  - **Improvements:**
    - Added 5-sample moving average filter for smoother heading display
    - Uses circular mean algorithm to handle 0°/360° wraparound correctly

- **v0.0.3** (January 2025) - **Hardware Tested**
  - **New Features:**
    - Added automatic magnetometer calibration via web interface
    - Calibration button with 15-second countdown timer
    - Clear calibration button to reset offsets
    - Calibration data saved to EEPROM (persists across reboots)
    - Real-time calibration status display in web UI
  - **Technical Details:**
    - Hard-iron calibration using min/max method
    - EEPROM storage with magic number validation
    - WebSocket commands for calibration control
  - **Status:** Successfully tested on hardware

- **v0.0.2** (January 2025)
  - **Bug Fixes:**
    - Fixed critical division-by-zero in tilt compensation when device is tilted near vertical
    - Fixed gimbal lock handling when pitch approaches ±90 degrees
    - Fixed potential NaN values from asin() with out-of-range inputs
    - Fixed potential negative array index in cardinal direction lookup
    - Fixed SPIFFS mount failure now properly halts execution with error message
  - **Improvements:**
    - Replaced String concatenation with pre-allocated buffer to prevent heap fragmentation
    - Added JSON parse error handling in web interface
    - Removed redundant AsyncWebSocket include
    - Added bounds checking throughout compass calculations

- **v0.0.1** (January 2025)
  - Initial PlatformIO setup
  - Migrated from Arduino IDE to PlatformIO
  - Tilt-compensated heading calculation
  - WebSocket real-time updates
  - Dark/Light mode interface
  - FireBeetle ESP32 support
  - STEMMA QT connector support

## Dependencies

Managed automatically by PlatformIO (see `platformio.ini`):

- Adafruit Unified Sensor
- Adafruit LSM303 Accel
- Adafruit LIS2MDL
- Adafruit BME280 Library
- AsyncTCP (me-no-dev)
- ESPAsyncWebServer (me-no-dev)

## License

This project is open source. Feel free to modify, distribute, and use for any purpose.

## Credits

**Hardware:**
- ESP32 by Espressif Systems
- LSM303AGR sensor by STMicroelectronics
- FireBeetle board by DFRobot
- Breakout board by Adafruit Industries

**Software Libraries:**
- Adafruit Sensor Libraries
- ESPAsyncWebServer by me-no-dev
- AsyncTCP by me-no-dev

---

Built with ESP32 and Adafruit LSM303AGR sensor | January 2025
