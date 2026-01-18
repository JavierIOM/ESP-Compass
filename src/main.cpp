#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
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

// Create sensor objects
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

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
char jsonBuffer[256];

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

    // Send data to all connected WebSocket clients using pre-allocated buffer
    snprintf(jsonBuffer, sizeof(jsonBuffer),
      "{\"heading\":%.1f,\"direction\":\"%s\",\"mag_x\":%.2f,\"mag_y\":%.2f,\"mag_z\":%.2f,\"calibrating\":%s,\"calRemaining\":%d}",
      heading, direction.c_str(), mag_x, mag_y, mag_z,
      calibrating ? "true" : "false", calRemaining);

    ws.textAll(jsonBuffer);
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
      data[len] = 0; // Null terminate
      String msg = (char*)data;

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
    Serial.println("Calibration loaded from EEPROM:");
    Serial.printf("  Offset X: %.2f\n", magOffsetX);
    Serial.printf("  Offset Y: %.2f\n", magOffsetY);
    Serial.printf("  Offset Z: %.2f\n", magOffsetZ);
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
