#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <SPIFFS.h>
#include <EEPROM.h>

// EEPROM configuration
#define EEPROM_SIZE 64
#define CALIBRATION_MAGIC 0xCAFE  // Magic number to verify valid calibration data
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_OFFSET_X_ADDR 2
#define EEPROM_OFFSET_Y_ADDR 6
#define EEPROM_OFFSET_Z_ADDR 10

// Access Point credentials - CHANGE THESE IF DESIRED
const char* ap_ssid = "ESP32-Compass";     // The WiFi network name
const char* ap_password = "compass123";     // Password (min 8 chars, or "" for open network)

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDRESS 0x3C

// GPS configuration (using Serial2)
#define GPS_RX_PIN 16  // Connect GPS TX to this pin
#define GPS_TX_PIN 17  // Connect GPS RX to this pin
#define GPS_BAUD 9600

// Create sensor objects
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;

// Optional sensor status
bool bmeAvailable = false;
bool oledAvailable = false;
bool gpsAvailable = false;

// GPS data
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
float gpsSpeed = 0.0;
int gpsSatellites = 0;
bool gpsHasFix = false;
char gridSquare[7] = "------";  // 6-character Maidenhead locator + null

// Display update timing (slower than sensor updates)
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 250; // Update display 4 times per second

// Web server on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Timing variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // Update every 100ms

// Calibration offsets (loaded from EEPROM or set via calibration)
float magOffsetX = 0.0;
float magOffsetY = 0.0;
float magOffsetZ = 0.0;

// Calibration state
bool calibrating = false;
unsigned long calibrationStartTime = 0;
const unsigned long calibrationDuration = 15000; // 15 seconds

// Min/Max tracking during calibration
float magMinX, magMaxX;
float magMinY, magMaxY;
float magMinZ, magMaxZ;

// Pre-allocated buffer for JSON to reduce heap fragmentation
char jsonBuffer[384];

// Smoothing filter for heading
#define SMOOTHING_SAMPLES 5
float headingHistory[SMOOTHING_SAMPLES];
int headingIndex = 0;
bool headingBufferFilled = false;

// Forward declarations
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len);
float calculateTiltCompensatedHeading(float ax, float ay, float az, float mx, float my, float mz);
String getCardinalDirection(float heading);
void loadCalibration();
void saveCalibration();
void startCalibration();
void updateCalibration(float mx, float my, float mz);
void finishCalibration();
void updateDisplay(float heading, const String& direction, float temperature);
void calculateGridSquare(float lat, float lon, char* grid);
void processGPS();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\nESP32 Digital Compass");
  Serial.println("=====================");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Initialize I2C
  Wire.begin();

  // Initialize sensors
  if (!accel.begin()) {
    Serial.println("ERROR: Could not find LSM303AGR accelerometer!");
    while (1) delay(10);
  }

  if (!mag.begin()) {
    Serial.println("ERROR: Could not find LSM303AGR magnetometer!");
    while (1) delay(10);
  }

  Serial.println("LSM303AGR sensors initialized successfully!");

  // Try to initialize BME280 (optional sensor)
  if (bme.begin(0x76) || bme.begin(0x77)) {
    bmeAvailable = true;
    Serial.println("BME280 sensor found!");
  } else {
    bmeAvailable = false;
    Serial.println("BME280 not found - continuing without environmental data");
  }

  // Try to initialize OLED display (optional)
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    oledAvailable = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("ESP32 Compass");
    display.println("Initializing...");
    display.display();
    Serial.println("OLED display found!");
  } else {
    oledAvailable = false;
    Serial.println("OLED display not found - continuing without display");
  }

  // Try to initialize GPS (optional)
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  // We'll check for GPS data in the loop - if we get valid data, gpsAvailable becomes true
  Serial.println("GPS serial initialized (will detect if module connected)");

  // Load calibration from EEPROM
  loadCalibration();

  // Initialize SPIFFS for serving web files
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    Serial.println("Web interface will not be available.");
    while (1) delay(10);  // Halt execution - web server won't work without SPIFFS
  }

  // Start Access Point
  Serial.println("Starting Access Point...");
  WiFi.softAP(ap_ssid, ap_password);

  IPAddress IP = WiFi.softAPIP();
  Serial.println("Access Point Started!");
  Serial.print("AP SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP Password: ");
  Serial.println(ap_password);
  Serial.print("IP Address: ");
  Serial.println(IP);

  // Setup WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve static files from SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  // Start server
  server.begin();
  Serial.println("Web server started!");
  Serial.println("Connect to WiFi network: " + String(ap_ssid));
  Serial.println("Then visit: http://" + IP.toString());
}

void loop() {
  // Clean up WebSocket clients
  ws.cleanupClients();

  // Process GPS data if available
  processGPS();

  // Check if calibration is complete
  if (calibrating && (millis() - calibrationStartTime >= calibrationDuration)) {
    finishCalibration();
  }

  // Read and send compass data at regular intervals
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    sensors_event_t accel_event;
    sensors_event_t mag_event;

    // Get sensor readings
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    // Get raw magnetometer values
    float raw_mag_x = mag_event.magnetic.x;
    float raw_mag_y = mag_event.magnetic.y;
    float raw_mag_z = mag_event.magnetic.z;

    // Debug output - print raw sensor data every second
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint >= 1000) {
      lastDebugPrint = millis();
      Serial.println("--- RAW SENSOR DATA ---");
      Serial.printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n",
        accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z);
      Serial.printf("Mag Raw: X=%.2f Y=%.2f Z=%.2f µT\n", raw_mag_x, raw_mag_y, raw_mag_z);
      Serial.printf("Mag Cal: X=%.2f Y=%.2f Z=%.2f µT (offsets: %.2f, %.2f, %.2f)\n",
        raw_mag_x - magOffsetX, raw_mag_y - magOffsetY, raw_mag_z - magOffsetZ,
        magOffsetX, magOffsetY, magOffsetZ);
    }

    // Update calibration if in progress (use raw values)
    if (calibrating) {
      updateCalibration(raw_mag_x, raw_mag_y, raw_mag_z);
    }

    // Apply calibration offsets
    float mag_x = raw_mag_x - magOffsetX;
    float mag_y = raw_mag_y - magOffsetY;
    float mag_z = raw_mag_z - magOffsetZ;

    // Calculate heading with tilt compensation
    float heading = calculateTiltCompensatedHeading(
      accel_event.acceleration.x,
      accel_event.acceleration.y,
      accel_event.acceleration.z,
      mag_x, mag_y, mag_z
    );

    // Convert to degrees (0-360)
    heading = heading * 180.0 / PI;
    if (heading < 0) {
      heading += 360.0;
    }

    // Apply smoothing filter (circular average to handle 0/360 wraparound)
    headingHistory[headingIndex] = heading;
    headingIndex = (headingIndex + 1) % SMOOTHING_SAMPLES;
    if (headingIndex == 0) headingBufferFilled = true;

    int samplesToUse = headingBufferFilled ? SMOOTHING_SAMPLES : headingIndex;
    if (samplesToUse > 0) {
      // Use circular mean to handle wraparound at 0/360
      float sinSum = 0, cosSum = 0;
      for (int i = 0; i < samplesToUse; i++) {
        float rad = headingHistory[i] * PI / 180.0;
        sinSum += sin(rad);
        cosSum += cos(rad);
      }
      heading = atan2(sinSum, cosSum) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
    }

    // Get cardinal direction
    String direction = getCardinalDirection(heading);

    // Calculate remaining calibration time
    int calRemaining = 0;
    if (calibrating) {
      calRemaining = (int)((calibrationDuration - (millis() - calibrationStartTime)) / 1000) + 1;
      if (calRemaining < 0) calRemaining = 0;
    }

    // Read environmental data if available
    float temperature = 0.0;
    float humidity = 0.0;
    float pressure = 0.0;
    if (bmeAvailable) {
      temperature = bme.readTemperature();
      humidity = bme.readHumidity();
      pressure = bme.readPressure() / 100.0F; // Convert to hPa
    }

    // Build JSON with all available data
    int len = snprintf(jsonBuffer, sizeof(jsonBuffer),
      "{\"heading\":%.1f,\"direction\":\"%s\",\"mag_x\":%.2f,\"mag_y\":%.2f,\"mag_z\":%.2f,\"calibrating\":%s,\"calRemaining\":%d",
      heading, direction.c_str(), mag_x, mag_y, mag_z,
      calibrating ? "true" : "false", calRemaining);

    // Add environmental data if available
    if (bmeAvailable) {
      len += snprintf(jsonBuffer + len, sizeof(jsonBuffer) - len,
        ",\"temperature\":%.1f,\"humidity\":%.1f,\"pressure\":%.1f",
        temperature, humidity, pressure);
    }

    // Add GPS data if available
    if (gpsAvailable && gpsHasFix) {
      len += snprintf(jsonBuffer + len, sizeof(jsonBuffer) - len,
        ",\"gps_lat\":%.6f,\"gps_lon\":%.6f,\"gps_alt\":%.1f,\"gps_speed\":%.1f,\"gps_sats\":%d,\"grid_square\":\"%s\"",
        gpsLatitude, gpsLongitude, gpsAltitude, gpsSpeed, gpsSatellites, gridSquare);
    }

    // Close JSON object
    snprintf(jsonBuffer + len, sizeof(jsonBuffer) - len, "}");

    ws.textAll(jsonBuffer);

    // Update OLED display at slower rate
    if (oledAvailable && (millis() - lastDisplayUpdate >= displayUpdateInterval)) {
      lastDisplayUpdate = millis();
      updateDisplay(heading, direction, temperature);
    }
  }
}

float calculateTiltCompensatedHeading(float ax, float ay, float az, float mx, float my, float mz) {
  // Normalize accelerometer values with division-by-zero protection
  float accel_norm = sqrt(ax * ax + ay * ay + az * az);

  // Protect against division by zero (sensor fault or initialization)
  if (accel_norm < 0.001) {
    accel_norm = 0.001;
  }

  ax /= accel_norm;
  ay /= accel_norm;
  az /= accel_norm;

  // Clamp ax to valid range for asin (-1 to 1) to prevent NaN
  if (ax > 1.0) ax = 1.0;
  if (ax < -1.0) ax = -1.0;

  // Calculate pitch
  float pitch = asin(-ax);

  // Calculate roll with gimbal lock protection
  float cos_pitch = cos(pitch);
  float roll;

  // Protect against division by zero near gimbal lock (pitch near +/-90 degrees)
  if (fabs(cos_pitch) < 0.01) {
    // Near gimbal lock - use simplified calculation
    roll = 0.0;
  } else {
    // Clamp ay/cos(pitch) to valid range for asin
    float roll_arg = ay / cos_pitch;
    if (roll_arg > 1.0) roll_arg = 1.0;
    if (roll_arg < -1.0) roll_arg = -1.0;
    roll = asin(roll_arg);
  }

  // Tilt compensated magnetic field components
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float sin_pitch = sin(pitch);

  float mag_x_comp = mx * cos_pitch + mz * sin_pitch;
  float mag_y_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

  // Calculate heading (negated to correct for sensor orientation)
  float heading = -atan2(mag_y_comp, mag_x_comp);

  return heading;
}

String getCardinalDirection(float heading) {
  // Convert heading to 16-point compass rose
  static const char* directions[] = {
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
  };

  // Ensure heading is in valid range (0-360)
  while (heading < 0) heading += 360.0;
  while (heading >= 360.0) heading -= 360.0;

  // Calculate index safely (always positive due to range normalization above)
  int index = (int)((heading + 11.25) / 22.5) % 16;

  // Extra safety check for index bounds
  if (index < 0) index = 0;
  if (index > 15) index = 15;

  return directions[index];
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    // Handle incoming data (calibration commands)
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      // Copy data to a properly sized buffer for null termination
      char msgBuffer[64];
      size_t copyLen = len < sizeof(msgBuffer) - 1 ? len : sizeof(msgBuffer) - 1;
      memcpy(msgBuffer, data, copyLen);
      msgBuffer[copyLen] = '\0';
      String msg = msgBuffer;

      if (msg.indexOf("startCal") >= 0) {
        startCalibration();
        Serial.println("Calibration started via WebSocket");
      } else if (msg.indexOf("clearCal") >= 0) {
        // Clear calibration
        magOffsetX = 0.0;
        magOffsetY = 0.0;
        magOffsetZ = 0.0;
        saveCalibration();
        Serial.println("Calibration cleared via WebSocket");
      }
    }
  }
}

// Load calibration offsets from EEPROM
void loadCalibration() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic == CALIBRATION_MAGIC) {
    EEPROM.get(EEPROM_OFFSET_X_ADDR, magOffsetX);
    EEPROM.get(EEPROM_OFFSET_Y_ADDR, magOffsetY);
    EEPROM.get(EEPROM_OFFSET_Z_ADDR, magOffsetZ);

    // Sanity check: typical magnetometer offsets should be within +/- 200 µT
    // If values are NaN, Inf, or out of range, reset to zero
    bool valid = true;
    if (isnan(magOffsetX) || isinf(magOffsetX) || fabs(magOffsetX) > 200.0) valid = false;
    if (isnan(magOffsetY) || isinf(magOffsetY) || fabs(magOffsetY) > 200.0) valid = false;
    if (isnan(magOffsetZ) || isinf(magOffsetZ) || fabs(magOffsetZ) > 200.0) valid = false;

    if (valid) {
      Serial.println("Calibration loaded from EEPROM:");
      Serial.printf("  Offset X: %.2f\n", magOffsetX);
      Serial.printf("  Offset Y: %.2f\n", magOffsetY);
      Serial.printf("  Offset Z: %.2f\n", magOffsetZ);
    } else {
      Serial.println("EEPROM calibration data invalid - resetting to zero");
      magOffsetX = 0.0;
      magOffsetY = 0.0;
      magOffsetZ = 0.0;
    }
  } else {
    Serial.println("No calibration data found in EEPROM");
    magOffsetX = 0.0;
    magOffsetY = 0.0;
    magOffsetZ = 0.0;
  }
}

// Save calibration offsets to EEPROM
void saveCalibration() {
  uint16_t magic = CALIBRATION_MAGIC;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
  EEPROM.put(EEPROM_OFFSET_X_ADDR, magOffsetX);
  EEPROM.put(EEPROM_OFFSET_Y_ADDR, magOffsetY);
  EEPROM.put(EEPROM_OFFSET_Z_ADDR, magOffsetZ);
  EEPROM.commit();
  Serial.println("Calibration saved to EEPROM");
}

// Start calibration process
void startCalibration() {
  calibrating = true;
  calibrationStartTime = millis();

  // Initialize min/max with extreme values
  magMinX = 99999.0;
  magMaxX = -99999.0;
  magMinY = 99999.0;
  magMaxY = -99999.0;
  magMinZ = 99999.0;
  magMaxZ = -99999.0;

  Serial.println("Calibration started - rotate the compass in all directions");
}

// Update calibration with new magnetometer reading
void updateCalibration(float mx, float my, float mz) {
  if (mx < magMinX) magMinX = mx;
  if (mx > magMaxX) magMaxX = mx;
  if (my < magMinY) magMinY = my;
  if (my > magMaxY) magMaxY = my;
  if (mz < magMinZ) magMinZ = mz;
  if (mz > magMaxZ) magMaxZ = mz;
}

// Finish calibration and calculate offsets
void finishCalibration() {
  calibrating = false;

  // Calculate offsets as midpoint of min/max (hard-iron correction)
  magOffsetX = (magMinX + magMaxX) / 2.0;
  magOffsetY = (magMinY + magMaxY) / 2.0;
  magOffsetZ = (magMinZ + magMaxZ) / 2.0;

  // Save to EEPROM
  saveCalibration();

  Serial.println("Calibration complete!");
  Serial.printf("  Min X: %.2f, Max X: %.2f, Offset X: %.2f\n", magMinX, magMaxX, magOffsetX);
  Serial.printf("  Min Y: %.2f, Max Y: %.2f, Offset Y: %.2f\n", magMinY, magMaxY, magOffsetY);
  Serial.printf("  Min Z: %.2f, Max Z: %.2f, Offset Z: %.2f\n", magMinZ, magMaxZ, magOffsetZ);
}

// Process incoming GPS data
void processGPS() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (gps.encode(c)) {
      // We got valid GPS data - module is connected
      if (!gpsAvailable) {
        gpsAvailable = true;
        Serial.println("GPS module detected!");
      }

      // Update GPS fix status based on current validity
      if (gps.location.isValid()) {
        gpsHasFix = true;
        gpsLatitude = gps.location.lat();
        gpsLongitude = gps.location.lng();
        calculateGridSquare(gpsLatitude, gpsLongitude, gridSquare);
      } else {
        // Lost GPS fix
        gpsHasFix = false;
      }

      if (gps.altitude.isValid()) {
        gpsAltitude = gps.altitude.meters();
      }

      if (gps.speed.isValid()) {
        gpsSpeed = gps.speed.kmph();
      }

      if (gps.satellites.isValid()) {
        gpsSatellites = gps.satellites.value();
      }
    }
  }
}

// Calculate 6-character Maidenhead grid square from lat/lon
void calculateGridSquare(float lat, float lon, char* grid) {
  // Maidenhead Locator System
  // Field: 18 zones of 20 degrees longitude, 18 zones of 10 degrees latitude (A-R)
  // Square: 10 zones each (0-9)
  // Subsquare: 24 zones each (a-x)

  // Normalize longitude to 0-360 and latitude to 0-180
  lon += 180.0;
  lat += 90.0;

  // Field (first two characters)
  grid[0] = 'A' + (int)(lon / 20.0);
  grid[1] = 'A' + (int)(lat / 10.0);

  // Square (next two characters)
  lon = fmod(lon, 20.0);
  lat = fmod(lat, 10.0);
  grid[2] = '0' + (int)(lon / 2.0);
  grid[3] = '0' + (int)(lat / 1.0);

  // Subsquare (last two characters) - lowercase a-x per Maidenhead standard
  lon = fmod(lon, 2.0);
  lat = fmod(lat, 1.0);
  int subLon = (int)(lon * 12.0);  // 24 divisions over 2 degrees
  int subLat = (int)(lat * 24.0);  // 24 divisions over 1 degree
  // Clamp to valid range 0-23 (handles boundary case at exactly 2.0 or 1.0)
  if (subLon > 23) subLon = 23;
  if (subLat > 23) subLat = 23;
  grid[4] = 'a' + subLon;
  grid[5] = 'a' + subLat;

  grid[6] = '\0';
}

// Update OLED display with current data
void updateDisplay(float heading, const String& direction, float temperature) {
  display.clearDisplay();

  // Large heading at top (centered)
  display.setTextSize(3);
  char headingStr[8];
  snprintf(headingStr, sizeof(headingStr), "%.1f", heading);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(headingStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.print(headingStr);
  display.setTextSize(1);
  display.print(" ");  // Small space before degree symbol

  // Cardinal direction below heading
  display.setTextSize(2);
  display.getTextBounds(direction.c_str(), 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 26);
  display.print(direction);

  // Grid square and temperature on same line
  display.setTextSize(1);
  display.setCursor(0, 46);
  if (gpsAvailable && gpsHasFix) {
    display.print(gridSquare);
  } else {
    display.print("No GPS");
  }

  // Temperature on right side
  if (bmeAvailable) {
    char tempStr[10];
    snprintf(tempStr, sizeof(tempStr), "%.1fC", temperature);
    display.getTextBounds(tempStr, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, 46);
    display.print(tempStr);
  }

  // Network info at bottom
  display.setCursor(0, 56);
  display.print(ap_ssid);

  // IP on right
  IPAddress ip = WiFi.softAPIP();
  String ipStr = ip.toString();
  display.getTextBounds(ipStr.c_str(), 0, 0, &x1, &y1, &w, &h);
  display.setCursor(SCREEN_WIDTH - w, 56);
  display.print(ipStr);

  display.display();
}
