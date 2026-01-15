# ESP32 Digital Compass

A remote-accessible digital compass built with ESP32 and the Adafruit LSM303AGR accelerometer/magnetometer sensor. The ESP32 creates its own WiFi access point, perfect for field use. Access your compass from any device with a beautiful, real-time web interface.

## Features

- Real-time compass heading with tilt compensation
- 16-point cardinal direction display (N, NNE, NE, etc.)
- Live magnetometer readings (X, Y, Z axes)
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

**Total Cost: ~£17-21**

### Why FireBeetle ESP32?

- **Low Power:** Ultra-low power consumption design, perfect for battery operation
- **Built-in LiPo Support:** Battery connector with onboard charging circuit
- **Compact Design:** Smaller than standard ESP32 DevKit boards
- **Quality Components:** DFRobot quality, more reliable than generic clones
- **I2C Ready:** GPIO21 (SDA) and GPIO22 (SCL) match our code perfectly
- **Proven:** I2C and GPIO functions fully working in Arduino IDE

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

## Software Requirements

### Arduino IDE Setup (Windows)

#### 1. Install Arduino IDE
- Download Arduino IDE 2.x from: https://www.arduino.cc/en/software
- Choose "Windows Win 10 and newer, 64 bits"
- Run the installer
- Installation time: ~5 minutes

#### 2. Add ESP32 Board Support
- Open Arduino IDE
- Go to **File → Preferences**
- In "Additional Board Manager URLs", add:
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
- Click **OK**
- Go to **Tools → Board → Boards Manager**
- Search for **"esp32"**
- Install **"esp32 by Espressif Systems"** (latest version)
- Wait for download/install (~5-10 minutes)

#### 3. Install Required Libraries

Open **Tools → Manage Libraries** and install each of these:

**From Library Manager:**
- `Adafruit LSM303 Accel` by Adafruit
- `Adafruit LSM303AGR Mag` by Adafruit
- `Adafruit Unified Sensor` by Adafruit (auto-installed as dependency)

**Manual Installation Required:**

ESPAsyncWebServer and AsyncTCP need manual installation:

1. Download these ZIP files:
   - AsyncTCP: https://github.com/me-no-dev/AsyncTCP/archive/master.zip
   - ESPAsyncWebServer: https://github.com/me-no-dev/ESPAsyncWebServer/archive/master.zip

2. In Arduino IDE:
   - Go to **Sketch → Include Library → Add .ZIP Library**
   - Select the downloaded AsyncTCP ZIP file
   - Repeat for ESPAsyncWebServer ZIP file

#### 4. Install ESP32 Filesystem Uploader

This tool uploads the web interface to the ESP32's SPIFFS filesystem:

1. Download from: https://github.com/me-no-dev/arduino-esp32fs-plugin/releases
2. Extract the contents
3. Copy to: `C:\Users\[YourUsername]\Documents\Arduino\tools\`
   - Create the `tools` folder if it doesn't exist
4. Restart Arduino IDE
5. Verify: **Tools** menu should now have "ESP32 Sketch Data Upload"

#### 5. USB Drivers (Usually Automatic)

Windows 10/11 usually installs drivers automatically when you plug in the FireBeetle.

If the board isn't recognized:
- **CP2102 Driver:** https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- **CH340 Driver:** https://sparks.gogo.co.nz/ch340.html

**Total setup time: 15-20 minutes**

## Installation

### 1. Configure Access Point (Optional)

The ESP32 creates its own WiFi network. You can customize the name and password by editing `ESP32-Compass.ino` lines 11-12:

```cpp
const char* ap_ssid = "ESP32-Compass";     // The WiFi network name
const char* ap_password = "compass123";     // Password (min 8 chars, or "" for open network)
```

**Default settings work great for most users!** The ESP32 will broadcast "ESP32-Compass" as its network name.

### 2. Select Board and Port

- Connect FireBeetle ESP32 via USB
- Go to **Tools → Board → esp32**
- Select **"FireBeetle-ESP32"** (or "ESP32 Dev Module" if not listed)
- Go to **Tools → Port**
- Select the COM port that appeared when you plugged in the board
  - Usually COM3, COM4, COM5, etc.
  - Windows should show "USB-SERIAL CH340" or "Silicon Labs CP210x"

### 3. Upload the Web Interface

This step uploads the HTML/CSS/JS files to the ESP32's filesystem:

- In Arduino IDE, go to **Tools → ESP32 Sketch Data Upload**
- Wait for upload to complete (~30 seconds)
- You should see "SPIFFS Image Uploaded" in the console

**Important:** Do this BEFORE uploading the sketch!

### 4. Upload the Arduino Sketch

- Click the **Upload** button (→ arrow icon)
- Wait for compilation and upload (~1-2 minutes)
- Console will show "Leaving... Hard resetting via RTS pin..."

### 5. Connect to the Compass

After uploading, the ESP32 creates its own WiFi network:

1. Open **Tools → Serial Monitor** and set baud rate to **115200**
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

**Bookmark the page and you'll only need to connect to the WiFi network in future.**

## Usage

### First Use

1. Power on the ESP32 (via USB or battery)
2. Wait ~5 seconds for the Access Point to start
3. Connect your device to the **ESP32-Compass** WiFi network
4. Open your browser to **http://192.168.4.1** (or use your bookmark)
5. The compass needle should start moving immediately
6. Rotate the sensor to test - the needle follows magnetic north

**Perfect for field use:** The ESP32 works anywhere, no internet or existing WiFi needed!

### Calibration

The compass may need calibration for accurate readings in your location.

**Calibration Procedure:**

1. Open the compass web interface
2. Watch the magnetometer X, Y, Z values at the bottom
3. Slowly rotate the sensor in all directions:
   - Complete 360° horizontal rotation
   - Tilt up and down
   - Roll side to side
   - Figure-8 pattern works well
4. Note the min/max values for each axis:
   ```
   X: min = -45.2, max = 52.3
   Y: min = -38.7, max = 41.2
   Z: min = -62.1, max = 58.9
   ```
5. Calculate offsets:
   ```
   magOffsetX = (-45.2 + 52.3) / 2 = 3.55
   magOffsetY = (-38.7 + 41.2) / 2 = 1.25
   magOffsetZ = (-62.1 + 58.9) / 2 = -1.6
   ```
6. Update the sketch (lines 29-31):
   ```cpp
   float magOffsetX = 3.55;
   float magOffsetY = 1.25;
   float magOffsetZ = -1.6;
   ```
7. Re-upload the sketch

### Tips for Best Results

- **Level mounting:** Keep sensor relatively level (within ±30°)
- **Avoid interference:** Keep 30cm+ away from:
  - Speakers/magnets
  - Motors
  - Large metal objects
  - Phones/laptops
  - Power supplies
- **Stable mounting:** Secure the sensor to avoid vibration
- **Update rate:** 10Hz refresh (100ms) - smooth and responsive

### Battery Operation

The FireBeetle ESP32 has excellent battery support:

- **LiPo Connector:** JST 2.0mm connector on board
- **Charging:** Onboard LiPo charging via USB
- **Battery Life:** 8-12 hours typical with 1000mAh battery
- **Low Power:** FireBeetle's optimized for battery operation

## Web Interface Features

### Compass Display
- **Animated compass rose** with 60 degree marks
- **Cardinal directions** (N, E, S, W) highlighted in red
- **Red needle** points to magnetic north
- **Gray needle** shows south (opposite direction)
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
- **Saved preference** - browser remembers your choice
- **Smooth transition** - animated color changes

### Connection Status
- **Green pulsing dot** - Connected to ESP32
- **Red pulsing dot** - Disconnected (auto-reconnects every 3 seconds)
- **Status text** - Shows connection state

## Troubleshooting

### Hardware Issues

**"Could not find LSM303AGR" Error**
- Check all 4 wire connections (GND, VIN, SDA, SCL)
- Verify sensor power - look for a small LED on the sensor board
- Ensure cable is fully inserted into STEMMA QT port
- Try the other STEMMA QT port on the sensor (both are identical)
- Check continuity with a multimeter if available

**Sensor Found But Readings Are All Zeros**
- The sensor is powered but not communicating
- Try swapping SDA and SCL wires
- Check for loose connections
- Press the RST button on the ESP32

**Compass Points Wrong Direction**
- Perform calibration procedure (see Usage section)
- Check for magnetic interference nearby
- Ensure sensor orientation is consistent
- Remember: compass shows magnetic north, not true north

### Software Issues

**Can't See ESP32-Compass WiFi Network**
- Wait 10 seconds after powering on - AP takes a few seconds to start
- Check Serial Monitor - does it say "Access Point Started!"?
- Ensure ESP32 is powered (check for LED on board)
- Try pressing the RST button on the ESP32
- Move closer to the ESP32 (AP range is ~10-20m)
- Check your device's WiFi settings are enabled
- Some devices hide networks with weak signal - move closer

**Web Page Won't Load**
- Ensure you're connected to the **ESP32-Compass** WiFi network first!
- Try the default IP: **http://192.168.4.1**
- Check Serial Monitor for the correct IP address
- Verify SPIFFS upload was successful (step 3 of installation)
- Try a different browser
- Clear browser cache
- Some devices need "Stay connected" confirmation when joining network without internet

**Page Loads But Shows "Disconnected"**
- The HTML loaded but WebSocket connection failed
- Check Serial Monitor - is the ESP32 still running?
- Try refreshing the page (F5)
- Check browser console (F12) for JavaScript errors
- Ensure you're still connected to the ESP32-Compass network

**WebSocket Connection Keeps Dropping**
- Move your device closer to the ESP32
- Check WiFi signal strength on your device
- Reduce update frequency in code (saves power and bandwidth):
  ```cpp
  const unsigned long updateInterval = 200; // Change from 100 to 200ms
  ```
- Check for interference (other WiFi networks, microwaves, metal objects)
- Try rebooting the ESP32

**Compass Needle Jerky/Stuttering**
- Normal near metal objects or magnetic sources
- Try calibration
- Increase CSS transition time in `data/index.html`:
  ```css
  transition: transform 0.5s ease-out; /* Change from 0.3s */
  ```

### Upload Issues

**"Failed to connect to ESP32"**
- Check USB cable (some are power-only, need data cable)
- Try a different USB port
- Press and hold BOOT button while uploading
- Install/update USB drivers (see Software Requirements section)
- Check Device Manager - is the COM port visible?

**"A fatal error occurred: Timed out waiting for packet header"**
- Press RST button, then immediately try upload again
- Try holding BOOT button during upload
- Reduce upload speed: Tools → Upload Speed → 115200

## Technical Details

### Hardware Specifications

**FireBeetle ESP32:**
- Microcontroller: ESP-WROOM-32 (Dual-core Xtensa LX6)
- Clock Speed: Up to 240MHz
- Flash: 16MB
- SRAM: 520KB
- WiFi: 802.11 b/g/n (2.4GHz)
- Bluetooth: Bluetooth 4.2 + BLE
- GPIO: 10 digital pins, 5 analog pins (ADC)
- Interfaces: I2C, SPI, UART
- Power: Ultra-low consumption design, LiPo charging circuit

**Adafruit LSM303AGR:**
- Accelerometer Range: ±2g / ±4g / ±8g / ±16g (selectable)
- Magnetometer Range: ±50 gauss
- Resolution: 16-bit
- I2C Addresses: 0x19 (accelerometer), 0x1E (magnetometer)
- Update Rate: Up to 400Hz (we use 10Hz)
- Power: ~0.5mA active

### Software Architecture

**Compass Calculation Algorithm:**
1. Read accelerometer (X, Y, Z) for tilt/orientation
2. Read magnetometer (X, Y, Z) for magnetic field
3. Normalize accelerometer vector
4. Calculate pitch: `pitch = asin(-ax)`
5. Calculate roll: `roll = asin(ay / cos(pitch))`
6. Apply tilt compensation to magnetometer:
   ```cpp
   mag_x_comp = mx * cos(pitch) + mz * sin(pitch)
   mag_y_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch)
   ```
7. Calculate heading: `heading = atan2(mag_y_comp, mag_x_comp)`
8. Convert to degrees: `heading_degrees = heading * 180 / PI`
9. Normalize to 0-360°

**Communication Protocol:**
- I2C bus @ 100kHz (default speed)
- WebSocket connection on port 80
- JSON data format:
  ```json
  {
    "heading": 245.3,
    "direction": "WSW",
    "mag_x": 23.45,
    "mag_y": -12.34,
    "mag_z": 45.67
  }
  ```
- Update frequency: 10Hz (100ms intervals)

**Web Server:**
- ESPAsyncWebServer (non-blocking)
- AsyncWebSocket for real-time data
- SPIFFS filesystem for HTML/CSS/JS
- Serves index.html as default page
- Auto-reconnection logic on client side

### Power Consumption

**Typical Current Draw:**
- ESP32 WiFi active: ~80mA
- LSM303AGR sensor: ~0.5mA
- **Total: ~80-100mA**

**Battery Life Estimates (with LiPo):**
- 500mAh battery: ~5-6 hours
- 1000mAh battery: ~10-12 hours
- 2000mAh battery: ~20-24 hours

**Power Optimization Options:**
- Reduce WiFi transmit power in code
- Increase update interval (less frequent reads)
- Use ESP32 light sleep between updates
- FireBeetle's power management chip already optimized

## Customization

### Change Update Rate

In `ESP32-Compass.ino`, line 24:
```cpp
const unsigned long updateInterval = 100; // milliseconds

// Examples:
// 50ms = 20Hz (very smooth, more power)
// 100ms = 10Hz (current setting, balanced)
// 200ms = 5Hz (slower, saves power)
// 500ms = 2Hz (very slow, maximum battery life)
```

### Modify Compass Appearance

Edit `data/index.html` CSS variables (lines 15-25):

```css
:root {
    --bg-primary: #0f172a;      /* Main background color */
    --bg-secondary: #1e293b;    /* Panel background */
    --text-primary: #f1f5f9;    /* Main text color */
    --accent: #ef4444;          /* Needle and highlight color */
    --success: #10b981;         /* Connection indicator */
    /* Change these hex colors to customize! */
}
```

**Color Suggestions:**
- Blue theme: `--accent: #3b82f6;`
- Green theme: `--accent: #10b981;`
- Purple theme: `--accent: #a855f7;`
- Orange theme: `--accent: #f97316;`

### Change Compass Size

In `data/index.html`, line 114:
```css
.compass-container {
    width: min(400px, 90vw);  /* Change 400px to larger/smaller */
    height: min(400px, 90vw);
}
```

### Add Additional Data Fields

In `ESP32-Compass.ino`, around line 70, add to the JSON:
```cpp
String json = "{";
json += "\"heading\":" + String(heading, 1) + ",";
json += "\"direction\":\"" + direction + "\",";
json += "\"mag_x\":" + String(mag_x, 2) + ",";
json += "\"mag_y\":" + String(mag_y, 2) + ",";
json += "\"mag_z\":" + String(mag_z, 2) + ",";
json += "\"temperature\":" + String(temperature, 1); // Add custom field
json += "}";
```

Then update `data/index.html` to display it.

## Demo Mode

Want to see the interface without hardware?

Open `demo.html` in your browser - it simulates a working compass with animated rotation and realistic magnetometer readings.

Perfect for:
- Testing the interface design
- Showing off the project
- Development/customization
- Screenshots

## Version History

- **v1.0** (January 2025)
  - Initial release
  - Tilt-compensated heading calculation
  - WebSocket real-time updates
  - Dark/Light mode interface
  - FireBeetle ESP32 support
  - STEMMA QT connector support
  - Demo mode

## Future Enhancements

Possible additions:
- GPS integration for true north calculation
- Data logging to SD card
- Historical heading graph
- Calibration wizard in web interface
- Mobile app (PWA support)
- MQTT publishing for home automation
- Multiple sensor support
- Pitch/roll display

## Support & Resources

**For Hardware Issues:**
- Adafruit LSM303AGR: https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout
- DFRobot FireBeetle: https://wiki.dfrobot.com/FireBeetle_ESP32_IOT_Microcontroller

**For ESP32 Questions:**
- ESP32 Arduino Core: https://github.com/espressif/arduino-esp32
- Espressif Documentation: https://docs.espressif.com/projects/esp-idf/

**For This Project:**
- Check the troubleshooting section above
- Review the wiring diagram
- Verify all software libraries are installed
- Check Serial Monitor output for error messages

## License

This project is open source. Feel free to modify, distribute, and use for any purpose.

## Credits

**Hardware:**
- ESP32 by Espressif Systems
- LSM303AGR sensor by STMicroelectronics
- FireBeetle board by DFRobot
- Breakout board by Adafruit Industries

**Software Libraries:**
- Adafruit Sensor Library
- Adafruit LSM303 Libraries
- ESPAsyncWebServer by me-no-dev
- AsyncTCP by me-no-dev

**Built with:**
- Arduino IDE
- HTML5, CSS3, JavaScript
- WebSocket API
- ESP32 Arduino Core

---

**Project by Javier**

Built with ESP32 and Adafruit LSM303AGR sensor | January 2025

**GitHub Repository:** https://github.com/JavierIOM/ESP-Compass
