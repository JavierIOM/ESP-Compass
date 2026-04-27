#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include <SPIFFS.h>
#include <EEPROM.h>

// EEPROM configuration
#define EEPROM_SIZE 64
#define CALIBRATION_MAGIC 0xCAF4      // Spin cal: offsets + scale + heading offset
#define CALIBRATION_MAGIC_V3 0xCAF2   // Old: offsets + scale (no heading offset)
#define CALIBRATION_MAGIC_OLD 0xCAFE  // Oldest: offsets only
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_OFFSET_X_ADDR 2
#define EEPROM_OFFSET_Y_ADDR 6
#define EEPROM_OFFSET_Z_ADDR 10
#define EEPROM_SCALE_X_ADDR 14
#define EEPROM_SCALE_Y_ADDR 18
#define EEPROM_HEADING_OFFSET_ADDR 22

// Access Point credentials - CHANGE THESE IF DESIRED
const char* ap_ssid = "ESP32-Compass";     // The WiFi network name
const char* ap_password = "compass123";     // Password (min 8 chars, or "" for open network)

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDRESS 0x3C

// GPS configuration (I2C via STEMMA QT)
#define GPS_I2C_ADDRESS 0x10

// Create sensor objects
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_GPS GPS(&Wire);

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
float gpsHDOP = 0.0;
float gpsVDOP = 0.0;
float gpsPDOP = 0.0;
uint8_t gpsFixQuality = 0;
char gpsTime[9] = "--:--:--";   // HH:MM:SS
char gpsDate[11] = "----/--/--"; // YYYY/MM/DD

// Display update timing (slower than sensor updates)
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 250; // Update display 4 times per second

// Web server on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Timing variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // Update every 100ms

// Calibration offsets, scale, and heading offset (loaded from EEPROM)
float magOffsetX = 0.0;
float magOffsetY = 0.0;
float magOffsetZ = 0.0;
float magScaleX = 1.0;
float magScaleY = 1.0;
float headingOffset = 0.0; // Corrects heading so north = 0°

// Spin calibration state
enum CalState { CAL_IDLE, CAL_SPINNING };
CalState calState = CAL_IDLE;
float calMinX, calMaxX, calMinY, calMaxY;
float calNorthRawX, calNorthRawY, calNorthRawZ; // Raw mag captured at north reference
float calNorthAccX, calNorthAccY, calNorthAccZ; // Accel captured at north reference (for tilt compensation)
bool calSectors[36];                // 36 sectors × 10° = full circle
int calSectorsVisited = 0;

// Latest raw magnetometer reading (updated every loop tick)
float latestRawMagX = 0.0;
float latestRawMagY = 0.0;
float latestRawMagZ = 0.0;

// Pre-allocated buffer for JSON to reduce heap fragmentation
char jsonBuffer[512];

// Smoothing filter for heading
#define SMOOTHING_SAMPLES 10
float headingHistory[SMOOTHING_SAMPLES];
int headingIndex = 0;
bool headingBufferFilled = false;

// EMA filter for sensor inputs — reduces noise before it hits the tilt compensation trig
#define EMA_ALPHA 0.2f
float emaAccX = 0.0f, emaAccY = 0.0f, emaAccZ = 0.0f;
float emaMagX = 0.0f, emaMagY = 0.0f, emaMagZ = 0.0f;
bool emaInit = false;

// Whether valid calibration data is loaded
bool calibrationValid = false;

// Forward declarations
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len);
float calculateTiltCompensatedHeading(float ax, float ay, float az, float mx, float my, float mz);
String getCardinalDirection(float heading);
void loadCalibration();
void saveCalibration();
void startSpinCalibration();
void updateSpinCalibration();
void computeSpinCalibration();
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
  // First do an I2C scan to verify a device is actually present at 0x3C
  // (display.begin() can return true even with no device connected)
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  bool oledOnBus = (Wire.endTransmission() == 0);
  if (oledOnBus && display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
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

  // Try to initialize GPS via I2C (optional - Adafruit Mini GPS PA1010D)
  if (GPS.begin(GPS_I2C_ADDRESS)) {
    gpsAvailable = true;
    GPS.sendCommand("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");  // RMC + GGA + GSA sentences
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);       // 1Hz update rate
    Serial.println("GPS module found on I2C!");
  } else {
    gpsAvailable = false;
    Serial.println("GPS module not found - continuing without GPS");
  }

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

  // Read and send compass data at regular intervals
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    sensors_event_t accel_event;
    sensors_event_t mag_event;

    // Get sensor readings
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    // Get raw magnetometer values and store for calibration capture
    float raw_mag_x = mag_event.magnetic.x;
    float raw_mag_y = mag_event.magnetic.y;
    float raw_mag_z = mag_event.magnetic.z;
    latestRawMagX = raw_mag_x;
    latestRawMagY = raw_mag_y;
    latestRawMagZ = raw_mag_z;

    // Update spin calibration if active
    updateSpinCalibration();

    // Apply calibration offsets and scale
    float mag_x = (raw_mag_x - magOffsetX) * magScaleX;
    float mag_y = (raw_mag_y - magOffsetY) * magScaleY;
    float mag_z = raw_mag_z - magOffsetZ;

    // EMA filter — smooth sensor inputs before tilt compensation to reduce noise
    if (!emaInit) {
      emaAccX = accel_event.acceleration.x;
      emaAccY = accel_event.acceleration.y;
      emaAccZ = accel_event.acceleration.z;
      emaMagX = mag_x; emaMagY = mag_y; emaMagZ = mag_z;
      emaInit = true;
    }
    emaAccX += EMA_ALPHA * (accel_event.acceleration.x - emaAccX);
    emaAccY += EMA_ALPHA * (accel_event.acceleration.y - emaAccY);
    emaAccZ += EMA_ALPHA * (accel_event.acceleration.z - emaAccZ);
    emaMagX += EMA_ALPHA * (mag_x - emaMagX);
    emaMagY += EMA_ALPHA * (mag_y - emaMagY);
    emaMagZ += EMA_ALPHA * (mag_z - emaMagZ);

    // Calculate heading with tilt compensation using filtered sensor values
    float heading = calculateTiltCompensatedHeading(
      emaAccX, emaAccY, emaAccZ,
      emaMagX, emaMagY, emaMagZ
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

    // Apply north offset so calibrated north = 0°
    heading += headingOffset;
    if (heading < 0) heading += 360.0;
    if (heading >= 360.0) heading -= 360.0;

    // Get cardinal direction
    String direction = getCardinalDirection(heading);

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
      "{\"heading\":%.1f,\"direction\":\"%s\",\"mag_x\":%.2f,\"mag_y\":%.2f,\"mag_z\":%.2f,\"cal_state\":%d,\"cal_progress\":%d,\"calibrated\":%d",
      heading, direction.c_str(), mag_x, mag_y, mag_z,
      (int)calState, (calSectorsVisited * 100 / 36), (int)calibrationValid);

    // Add environmental data if available
    if (bmeAvailable) {
      len += snprintf(jsonBuffer + len, sizeof(jsonBuffer) - len,
        ",\"temperature\":%.1f,\"humidity\":%.1f,\"pressure\":%.1f",
        temperature, humidity, pressure);
    }

    // Add GPS data if module is available
    if (gpsAvailable) {
      len += snprintf(jsonBuffer + len, sizeof(jsonBuffer) - len,
        ",\"gps_has_fix\":%s,\"gps_sats\":%d",
        gpsHasFix ? "true" : "false", gpsSatellites);
      if (gpsHasFix) {
        len += snprintf(jsonBuffer + len, sizeof(jsonBuffer) - len,
          ",\"gps_lat\":%.6f,\"gps_lon\":%.6f,\"gps_alt\":%.1f,\"gps_speed\":%.1f,\"grid_square\":\"%s\""
          ",\"gps_hdop\":%.1f,\"gps_vdop\":%.1f,\"gps_pdop\":%.1f,\"gps_fix_quality\":%d,\"gps_time\":\"%s\",\"gps_date\":\"%s\"",
          gpsLatitude, gpsLongitude, gpsAltitude, gpsSpeed, gridSquare,
          gpsHDOP, gpsVDOP, gpsPDOP, gpsFixQuality, gpsTime, gpsDate);
      }
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

  // Calculate pitch and roll
  // atan2(ay, az) for roll is stable across full ±180° range with no gimbal lock
  float pitch = asin(-ax);
  float roll = atan2(ay, az);

  // Tilt compensated magnetic field components
  float cos_pitch = cos(pitch);
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
      Serial.printf("WS received: %s\n", msgBuffer);

      if (msg.indexOf("calStart") >= 0) {
        startSpinCalibration();
      } else if (msg.indexOf("clearCal") >= 0) {
        calState = CAL_IDLE;
        magOffsetX = 0.0; magOffsetY = 0.0; magOffsetZ = 0.0;
        magScaleX = 1.0; magScaleY = 1.0;
        headingOffset = 0.0;
        calibrationValid = false;
        // Write an invalid magic so EEPROM doesn't reload as "calibrated" on next boot
        uint16_t inv = 0xFFFF;
        EEPROM.put(EEPROM_MAGIC_ADDR, inv);
        EEPROM.commit();
        Serial.println("Calibration cleared");
      }
    }
  }
}

// Load calibration from EEPROM
void loadCalibration() {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  auto loadBase = [&]() {
    EEPROM.get(EEPROM_OFFSET_X_ADDR, magOffsetX);
    EEPROM.get(EEPROM_OFFSET_Y_ADDR, magOffsetY);
    EEPROM.get(EEPROM_OFFSET_Z_ADDR, magOffsetZ);
    EEPROM.get(EEPROM_SCALE_X_ADDR, magScaleX);
    EEPROM.get(EEPROM_SCALE_Y_ADDR, magScaleY);
    bool valid = true;
    if (isnan(magOffsetX) || isinf(magOffsetX) || fabs(magOffsetX) > 500.0) valid = false;
    if (isnan(magOffsetY) || isinf(magOffsetY) || fabs(magOffsetY) > 500.0) valid = false;
    if (isnan(magOffsetZ) || isinf(magOffsetZ) || fabs(magOffsetZ) > 500.0) valid = false;
    if (isnan(magScaleX)  || isinf(magScaleX)  || magScaleX < 0.1 || magScaleX > 10.0) valid = false;
    if (isnan(magScaleY)  || isinf(magScaleY)  || magScaleY < 0.1 || magScaleY > 10.0) valid = false;
    return valid;
  };

  if (magic == CALIBRATION_MAGIC) {
    if (loadBase()) {
      EEPROM.get(EEPROM_HEADING_OFFSET_ADDR, headingOffset);
      if (isnan(headingOffset) || isinf(headingOffset)) headingOffset = 0.0;
      calibrationValid = true;
      Serial.printf("Calibration loaded: offset=%.2f,%.2f scale=%.3f,%.3f north=%.1f°\n",
        magOffsetX, magOffsetY, magScaleX, magScaleY, headingOffset);
    } else {
      Serial.println("EEPROM calibration invalid - resetting");
      magOffsetX = 0; magOffsetY = 0; magOffsetZ = 0;
      magScaleX = 1; magScaleY = 1; headingOffset = 0;
    }
  } else if (magic == CALIBRATION_MAGIC_V3 || magic == CALIBRATION_MAGIC_OLD) {
    bool valid = false;
    if (magic == CALIBRATION_MAGIC_V3) {
      valid = loadBase();
    } else {
      EEPROM.get(EEPROM_OFFSET_X_ADDR, magOffsetX);
      EEPROM.get(EEPROM_OFFSET_Y_ADDR, magOffsetY);
      EEPROM.get(EEPROM_OFFSET_Z_ADDR, magOffsetZ);
      magScaleX = 1.0; magScaleY = 1.0;
      valid = !(isnan(magOffsetX) || isnan(magOffsetY) || isnan(magOffsetZ));
    }
    if (valid) {
      headingOffset = 0.0;
      calibrationValid = true;
      Serial.println("Old calibration format loaded (no north offset — recalibrate for best accuracy)");
    } else {
      Serial.println("Old EEPROM calibration invalid - resetting");
      magOffsetX = 0; magOffsetY = 0; magOffsetZ = 0;
      magScaleX = 1; magScaleY = 1; headingOffset = 0;
    }
  } else {
    Serial.println("No calibration data in EEPROM");
    magOffsetX = 0; magOffsetY = 0; magOffsetZ = 0;
    magScaleX = 1; magScaleY = 1; headingOffset = 0;
  }
}

// Save calibration to EEPROM
void saveCalibration() {
  uint16_t magic = CALIBRATION_MAGIC;
  EEPROM.put(EEPROM_MAGIC_ADDR, magic);
  EEPROM.put(EEPROM_OFFSET_X_ADDR, magOffsetX);
  EEPROM.put(EEPROM_OFFSET_Y_ADDR, magOffsetY);
  EEPROM.put(EEPROM_OFFSET_Z_ADDR, magOffsetZ);
  EEPROM.put(EEPROM_SCALE_X_ADDR, magScaleX);
  EEPROM.put(EEPROM_SCALE_Y_ADDR, magScaleY);
  EEPROM.put(EEPROM_HEADING_OFFSET_ADDR, headingOffset);
  EEPROM.commit();
  calibrationValid = true;
  Serial.println("Calibration saved to EEPROM");
}

// Begin spin calibration — call when user presses the button pointing north (device must be level)
void startSpinCalibration() {
  calState = CAL_SPINNING;
  calNorthRawX = latestRawMagX;
  calNorthRawY = latestRawMagY;
  calNorthRawZ = latestRawMagZ;
  calNorthAccX = emaAccX;
  calNorthAccY = emaAccY;
  calNorthAccZ = emaAccZ;
  calMinX = calMaxX = latestRawMagX;
  calMinY = calMaxY = latestRawMagY;
  memset(calSectors, 0, sizeof(calSectors));
  calSectorsVisited = 0;
  Serial.println("Spin calibration started — spin 360° slowly");
}

// Called every loop tick to track the spin and detect completion
void updateSpinCalibration() {
  if (calState != CAL_SPINNING) return;

  // Expand min/max envelope
  if (latestRawMagX < calMinX) calMinX = latestRawMagX;
  if (latestRawMagX > calMaxX) calMaxX = latestRawMagX;
  if (latestRawMagY < calMinY) calMinY = latestRawMagY;
  if (latestRawMagY > calMaxY) calMaxY = latestRawMagY;

  // Don't mark sectors until there's enough spread to trust the centre estimate.
  // Without this gate, atan2(~0, ~0) from sensor noise covers all sectors instantly.
  float rangeX = calMaxX - calMinX;
  float rangeY = calMaxY - calMinY;
  if (rangeX < 5.0f && rangeY < 5.0f) return;

  // Use center-relative angle — without this, a large hard-iron offset collapses
  // all raw readings into a narrow arc so the sector counter never advances past ~5
  float centerX = (calMaxX + calMinX) / 2.0f;
  float centerY = (calMaxY + calMinY) / 2.0f;
  float angle = atan2(latestRawMagY - centerY, latestRawMagX - centerX) * 180.0f / PI;
  if (angle < 0) angle += 360.0f;
  int sector = (int)(angle / 10.0f) % 36;
  if (!calSectors[sector]) {
    calSectors[sector] = true;
    calSectorsVisited++;
  }

  // Auto-complete when 32/36 sectors covered (~89% of circle)
  if (calSectorsVisited >= 32) {
    computeSpinCalibration();
  }
}

// Compute and save calibration once the full spin is done
void computeSpinCalibration() {
  // Hard-iron offsets = centre of the ellipse
  magOffsetX = (calMaxX + calMinX) / 2.0;
  magOffsetY = (calMaxY + calMinY) / 2.0;
  magOffsetZ = 0.0;

  // Soft-iron scale = normalise ellipse to circle
  float rangeX = calMaxX - calMinX;
  float rangeY = calMaxY - calMinY;
  float avgRange = (rangeX + rangeY) / 2.0;
  magScaleX = (rangeX > 0.1) ? avgRange / rangeX : 1.0;
  magScaleY = (rangeY > 0.1) ? avgRange / rangeY : 1.0;

  // North offset: compute tilt-compensated heading for the saved north reference,
  // then store the negative so it reads 0° when pointing north.
  // Uses the accel captured at button press so tilt at that moment is accounted for.
  float corrNx = (calNorthRawX - magOffsetX) * magScaleX;
  float corrNy = (calNorthRawY - magOffsetY) * magScaleY;
  float corrNz = calNorthRawZ - magOffsetZ;
  float northHeading = calculateTiltCompensatedHeading(
    calNorthAccX, calNorthAccY, calNorthAccZ,
    corrNx, corrNy, corrNz
  ) * 180.0 / PI;
  if (northHeading < 0) northHeading += 360.0;
  headingOffset = -northHeading;

  saveCalibration();
  calState = CAL_IDLE;

  Serial.println("Spin calibration complete!");
  Serial.printf("  Offset: %.2f, %.2f  Scale: %.3f, %.3f  North: %.1f°\n",
    magOffsetX, magOffsetY, magScaleX, magScaleY, headingOffset);
}

// Process incoming GPS data (I2C)
void processGPS() {
  if (!gpsAvailable) return;

  // Read available data from GPS over I2C
  GPS.read();

  // Check if a new NMEA sentence has been received and parse it
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      // Update fix status
      if (GPS.fix) {
        gpsHasFix = true;
        gpsLatitude = GPS.latitudeDegrees;
        gpsLongitude = GPS.longitudeDegrees;
        gpsAltitude = GPS.altitude;
        gpsSpeed = GPS.speed * 1.15078; // Convert knots to mph
        gpsSatellites = (int)GPS.satellites;
        gpsHDOP = GPS.HDOP;
        gpsVDOP = GPS.VDOP;
        gpsPDOP = GPS.PDOP;
        gpsFixQuality = GPS.fixquality;
        snprintf(gpsTime, sizeof(gpsTime), "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);
        snprintf(gpsDate, sizeof(gpsDate), "20%02d/%02d/%02d", GPS.year, GPS.month, GPS.day);
        calculateGridSquare(gpsLatitude, gpsLongitude, gridSquare);
      } else {
        gpsHasFix = false;
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

  // Network info at bottom - show IP centered
  IPAddress ip = WiFi.softAPIP();
  String ipStr = ip.toString();
  display.getTextBounds(ipStr.c_str(), 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 56);
  display.print(ipStr);

  display.display();
}
