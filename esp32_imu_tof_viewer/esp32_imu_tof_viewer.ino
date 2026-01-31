#include <Adafruit_BNO055.h>

/*
 * ESP32 IMU + ToF AP Mode WebSocket Stream
 * Hardware: ELEGOO ESP32 (CP2102), Adafruit BNO055, VL53L1X (XSHUT GPIO19)
 * I2C: SDA=21, SCL=22
 * AP: SSID IMU_TOF_AP, pass imu12345, IP 192.168.4.1, WebSocket port 81, 25 Hz JSON
 */

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L1X.h>

// ============== CONFIG (edit here) ==============
#define WIFI_AP_SSID     "IMU_TOF_AP"
#define WIFI_AP_PASS     "imu12345"
#define WS_PORT          81
#define STREAM_HZ        25
#define SERIAL_BAUD      115200

#define I2C_SDA          21
#define I2C_SCL          22
#define VL53L1X_XSHUT    19

// ============== GLOBALS ==============
WebSocketsServer webSocket(WS_PORT);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
VL53L1X tof;

bool bno_ok = false;
bool tof_ok = false;
unsigned long lastSend = 0;
const unsigned long sendIntervalMs = 1000 / STREAM_HZ;

// Axis convention (for tilt / vertical_mm):
// - BNO055 reports gravity in sensor (body) frame: (gx, gy, gz) in m/s^2.
// - We assume ToF measures along body -Z (pointing "down" when board is level).
// - Body +Z = up when level, so "down" unit vector = (0, 0, -1).
// - cos(tilt) = dot(gravity_unit, down_unit) = -gz/|g|; tilt = angle between
//   sensor's down axis and true gravity. vertical_mm = raw_mm * cos(tilt).
static float cosTiltFromGravity(float gx, float gy, float gz) {
  float g = sqrtf(gx*gx + gy*gy + gz*gz);
  if (g < 1e-6f) return 1.0f;
  float cosTilt = -gz / g;  // down = (0,0,-1)
  if (cosTilt < 0.0f) cosTilt = 0.0f;
  if (cosTilt > 1.0f) cosTilt = 1.0f;
  return cosTilt;
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  (void)num;
  (void)payload;
  (void)len;
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] client connected\n");
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] client disconnected\n");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n--- IMU + ToF AP WebSocket ---");

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // I2C scan
  Serial.println("I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X\n", addr);
    }
  }

  // VL53L1X: XSHUT high to enable, then init
  pinMode(VL53L1X_XSHUT, OUTPUT);
  digitalWrite(VL53L1X_XSHUT, LOW);
  delay(10);
  digitalWrite(VL53L1X_XSHUT, HIGH);
  delay(10);
  if (tof.init()) {
    tof.setDistanceMode(VL53L1X::Long);
    tof.setMeasurementTimingBudget(33000);
    tof.startContinuous(50);
    tof_ok = true;
    Serial.println("VL53L1X OK");
  } else {
    Serial.println("VL53L1X init FAIL");
  }

  // BNO055 at 0x28 (wire ADR/AD0 to GND). Needs time after power-up.
  delay(650);
  Serial.println("I2C before BNO055 init:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X\n", addr);
    }
  }
  if (bno.begin()) {
    bno.setExtCrystalUse(false);
    delay(100);
    bno_ok = true;
    Serial.println("BNO055 OK");
  } else {
    Serial.println("BNO055 init FAIL - check: SDA=21, SCL=22, 3.3V, GND, BNO055 ADR->GND for 0x28");
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  IPAddress ap = WiFi.softAPIP();
  Serial.printf("AP IP: %s\n", ap.toString().c_str());

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.printf("WebSocket server on port %d, %d Hz\n", WS_PORT, STREAM_HZ);
}

void buildJson(String& out) {
  unsigned long t = millis();
  out = "{\"time_ms\":";
  out += t;

  // BNO055
  if (bno_ok) {
    imu::Quaternion q = bno.getQuat();
    out += ",\"quat_w\":";
    out += q.w();
    out += ",\"quat_x\":";
    out += q.x();
    out += ",\"quat_y\":";
    out += q.y();
    out += ",\"quat_z\":";
    out += q.z();

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    out += ",\"yaw\":";
    out += euler.x();
    out += ",\"pitch\":";
    out += euler.y();
    out += ",\"roll\":";
    out += euler.z();

    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float gx = grav.x(), gy = grav.y(), gz = grav.z();
    out += ",\"grav_x\":";
    out += gx;
    out += ",\"grav_y\":";
    out += gy;
    out += ",\"grav_z\":";
    out += gz;

    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    out += ",\"linacc_x\":";
    out += lin.x();
    out += ",\"linacc_y\":";
    out += lin.y();
    out += ",\"linacc_z\":";
    out += lin.z();

    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    out += ",\"cal_sys\":";
    out += (int)sys;
    out += ",\"cal_g\":";
    out += (int)gyro;
    out += ",\"cal_a\":";
    out += (int)accel;
    out += ",\"cal_m\":";
    out += (int)mag;

    int8_t temp = bno.getTemp();
    out += ",\"temp_c\":";
    out += (int)temp;

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    out += ",\"acc_x\":";
    out += acc.x();
    out += ",\"acc_y\":";
    out += acc.y();
    out += ",\"acc_z\":";
    out += acc.z();
    imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    out += ",\"gyro_x\":";
    out += gyr.x();
    out += ",\"gyro_y\":";
    out += gyr.y();
    out += ",\"gyro_z\":";
    out += gyr.z();
    imu::Vector<3> magVec = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    out += ",\"mag_x\":";
    out += magVec.x();
    out += ",\"mag_y\":";
    out += magVec.y();
    out += ",\"mag_z\":";
    out += magVec.z();
  } else {
    out += ",\"bno_error\":1";
    out += ",\"quat_w\":1,\"quat_x\":0,\"quat_y\":0,\"quat_z\":0";
    out += ",\"yaw\":0,\"pitch\":0,\"roll\":0";
    out += ",\"grav_x\":0,\"grav_y\":0,\"grav_z\":-9.8";
    out += ",\"cal_sys\":0,\"cal_g\":0,\"cal_a\":0,\"cal_m\":0";
  }

  // VL53L1X
  static uint16_t raw_mm = 0;
  static uint8_t range_status = 255;
  if (tof_ok && tof.dataReady()) {
    raw_mm = tof.read();
    range_status = (uint8_t)tof.ranging_data.range_status;
  }
  out += ",\"raw_mm\":";
  out += raw_mm;
  out += ",\"range_status\":";
  out += (int)range_status;

  float vertical_mm = (float)raw_mm;
  if (bno_ok) {
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float gx = grav.x(), gy = grav.y(), gz = grav.z();
    float ct = cosTiltFromGravity(gx, gy, gz);
    vertical_mm = (float)raw_mm * ct;
  }
  out += ",\"vertical_mm\":";
  out += vertical_mm;

  out += "}";
}

void loop() {
  webSocket.loop();

  if (millis() - lastSend >= sendIntervalMs) {
    lastSend = millis();
    String json;
    buildJson(json);
    webSocket.broadcastTXT(json);
    Serial.println(json);
  }
}
