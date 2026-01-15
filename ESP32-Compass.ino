#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LSM303AGR_Mag.h>
#include <SPIFFS.h>

// Access Point credentials - CHANGE THESE IF DESIRED
const char* ap_ssid = "ESP32-Compass";     // The WiFi network name
const char* ap_password = "compass123";     // Password (min 8 chars, or "" for open network)

// Create sensor objects
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303AGR_Mag_Unified mag = Adafruit_LSM303AGR_Mag_Unified(12345);

// Web server on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Timing variables
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // Update every 100ms

// Calibration offsets (you may need to calibrate these)
float magOffsetX = 0.0;
float magOffsetY = 0.0;
float magOffsetZ = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\nESP32 Digital Compass");
  Serial.println("=====================");

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

  // Initialize SPIFFS for serving web files
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    return;
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

  // Read and send compass data at regular intervals
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    sensors_event_t accel_event;
    sensors_event_t mag_event;

    // Get sensor readings
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    // Apply calibration offsets
    float mag_x = mag_event.magnetic.x - magOffsetX;
    float mag_y = mag_event.magnetic.y - magOffsetY;
    float mag_z = mag_event.magnetic.z - magOffsetZ;

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

    // Get cardinal direction
    String direction = getCardinalDirection(heading);

    // Send data to all connected WebSocket clients
    String json = "{";
    json += "\"heading\":" + String(heading, 1) + ",";
    json += "\"direction\":\"" + direction + "\",";
    json += "\"mag_x\":" + String(mag_x, 2) + ",";
    json += "\"mag_y\":" + String(mag_y, 2) + ",";
    json += "\"mag_z\":" + String(mag_z, 2);
    json += "}";

    ws.textAll(json);
  }
}

float calculateTiltCompensatedHeading(float ax, float ay, float az, float mx, float my, float mz) {
  // Normalize accelerometer values
  float accel_norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= accel_norm;
  ay /= accel_norm;
  az /= accel_norm;

  // Calculate pitch and roll
  float pitch = asin(-ax);
  float roll = asin(ay / cos(pitch));

  // Tilt compensated magnetic field components
  float mag_x_comp = mx * cos(pitch) + mz * sin(pitch);
  float mag_y_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

  // Calculate heading
  float heading = atan2(mag_y_comp, mag_x_comp);

  return heading;
}

String getCardinalDirection(float heading) {
  // Convert heading to 16-point compass rose
  const char* directions[] = {
    "N", "NNE", "NE", "ENE",
    "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW",
    "W", "WNW", "NW", "NNW"
  };

  int index = (int)((heading + 11.25) / 22.5) % 16;
  return directions[index];
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}
