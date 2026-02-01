/*
  Assistive Feeding Robot - Unified ESP32 Firmware

  What this sketch does:
  - Receives line-based commands over USB Serial from the Raspberry Pi
      (PING / ARM / TOOL SPOON / POSE FOOD_LEFT|FOOD_CENTER|FOOD_RIGHT|PRESENT / ACTION SCOOP)
    and replies with one-line OK/ERR responses (so your Pi send_and_wait() works).

  - Reads:
      * Adafruit BNO055 IMU (I2C)
      * VL53L1X Time-of-Flight distance sensor (I2C, XSHUT pin)

  - Runs an ESP32 Wi-Fi Access Point + HTTP dashboard
      * Connect laptop/phone to the AP
      * Open http://192.168.4.1/ to see live sensor + state

  - Streams JSON at STREAM_HZ over WebSocket (port WS_PORT)

  Notes:
  - Default is DRY_RUN=1 (NO SERVO MOTION). Flip to 0 once you have pins/poses calibrated.
  - Uses ESP32 Arduino core 3.x LEDC API (ledcAttach / ledcWrite by pin).

  Libraries you need in Arduino IDE:
    - "WebSockets" by Markus Sattler (Links2004)
    - "Adafruit BNO055" + "Adafruit Unified Sensor"
    - "VL53L1X" by Pololu
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L1X.h>

// ------------------------
// Build-time config
// ------------------------
#define DRY_RUN 1               // 1 = do not move servos (safe demo), 0 = enable servo output
#define SERIAL_BAUD 115200

#define WIFI_AP_SSID "ASSIST_AP"
#define WIFI_AP_PASS "assist123"

#define HTTP_PORT 80
#define WS_PORT 81
#define STREAM_HZ 25

#define I2C_SDA 21
#define I2C_SCL 22

#define VL53L1X_XSHUT 19

// ---- Servo output (edit these pins to match your wiring) ----
// Set a pin to -1 to disable that servo output.
static const int PIN_SERVO_BASE = 16;   // rotates between LEFT/CENTER/RIGHT
static const int PIN_SERVO_LIFT = 17;   // moves spoon up/down toward bowl (used w/ ToF stop)
static const int PIN_SERVO_TOOL = 18;   // rotates spoon to "scoop"

static const int SERVO_PWM_FREQ = 50;   // Hz
static const int SERVO_PWM_RES  = 16;   // bits

// Pulse widths (microseconds) - you MUST tune these for your hardware.
static const int BASE_LEFT_US   = 1100;
static const int BASE_CENTER_US = 1500;
static const int BASE_RIGHT_US  = 1900;

static const int LIFT_UP_US     = 1300;
static const int LIFT_DOWN_US   = 1900;

static const int TOOL_NEUTRAL_US = 1500;
static const int TOOL_SCOOP_US   = 1900;

// ToF stop threshold for "approach until close" behavior
static uint16_t TOF_STOP_MM_DEFAULT = 60;

// ------------------------
// Globals
// ------------------------
WebServer http(HTTP_PORT);
WebSocketsServer ws(WS_PORT);

Adafruit_BNO055 bno(55, 0x28);
VL53L1X tof;

static bool bno_ok = false;
static bool tof_ok = false;

static uint32_t last_stream_ms = 0;
static const uint32_t stream_interval_ms = 1000 / STREAM_HZ;

static String serial_line;

// WebSocket command handling is queued (processed in loop) to avoid
// doing long/blocking actions inside the ws callback.
static volatile bool ws_cmd_pending = false;
static uint8_t ws_cmd_client = 0;
static String ws_cmd_line;

// Forward decls (because we call streamTick() from pump())
static void streamTick();

// ------------------------
// Robot state (for dashboard)
// ------------------------
struct RobotState {
  bool armed = false;
  bool estop = false;
  String tool = "NONE";
  String pose = "IDLE";
  String last_cmd = "";
  String last_resp = "";

  // sensors
  uint16_t raw_mm = 0;
  float vertical_mm = 0;
  uint8_t range_status = 255;

  float qw = 1, qx = 0, qy = 0, qz = 0;
  float yaw = 0, pitch = 0, roll = 0;
  float gx = 0, gy = 0, gz = -9.8;
  int cal_sys = 0, cal_g = 0, cal_a = 0, cal_m = 0;

  uint16_t tof_stop_mm = 0;
};

static RobotState st;

// ------------------------
// Utility: servo pulse to LEDC duty
// ------------------------
static uint32_t usToDuty(int us) {
  const int period_us = 20000; // 50Hz
  const uint32_t maxDuty = (1u << SERVO_PWM_RES) - 1;
  us = constrain(us, 500, 2500);
  return (uint32_t)((uint64_t)us * maxDuty / period_us);
}

static bool attachServoPin(int pin) {
  if (pin < 0) return false;
#if DRY_RUN
  (void)pin;
  return true;
#else
  bool ok = ledcAttach(pin, SERVO_PWM_FREQ, SERVO_PWM_RES);
  if (!ok) {
    Serial.printf("[servo] ledcAttach failed on pin %d\n", pin);
    return false;
  }
  return true;
#endif
}

static void writeServoUs(int pin, int us) {
  if (pin < 0) return;
#if DRY_RUN
  (void)pin; (void)us;
#else
  ledcWrite(pin, usToDuty(us));
#endif
}

// ------------------------
// Utility: keep web stack alive during blocking motions
// ------------------------
static void pump(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    http.handleClient();
    ws.loop();

    // keep streaming during long actions
    streamTick();

    delay(1);
  }
}

// ------------------------
// IMU/ToF helpers
// ------------------------
static float cosTiltFromGravity(float gx, float gy, float gz) {
  float g = sqrtf(gx * gx + gy * gy + gz * gz);
  if (g < 1e-6f) return 1.0f;
  float cosTilt = -gz / g; // down = (0,0,-1)
  if (cosTilt < 0.0f) cosTilt = 0.0f;
  if (cosTilt > 1.0f) cosTilt = 1.0f;
  return cosTilt;
}

static void updateSensorsOnce() {
  // IMU
  if (bno_ok) {
    imu::Quaternion q = bno.getQuat();
    st.qw = q.w();
    st.qx = q.x();
    st.qy = q.y();
    st.qz = q.z();

    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    st.yaw = e.x();
    st.pitch = e.y();
    st.roll = e.z();

    imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    st.gx = g.x();
    st.gy = g.y();
    st.gz = g.z();

    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    st.cal_sys = (int)sys;
    st.cal_g = (int)gyro;
    st.cal_a = (int)accel;
    st.cal_m = (int)mag;
  } else {
    // keep sane defaults
    st.qw = 1; st.qx = 0; st.qy = 0; st.qz = 0;
    st.yaw = 0; st.pitch = 0; st.roll = 0;
    st.gx = 0; st.gy = 0; st.gz = -9.8;
    st.cal_sys = st.cal_g = st.cal_a = st.cal_m = 0;
  }

  // ToF
  if (tof_ok && tof.dataReady()) {
    st.raw_mm = tof.read();
    st.range_status = (uint8_t)tof.ranging_data.range_status;
  }

  // tilt-compensated "vertical" projection
  st.vertical_mm = (float)st.raw_mm;
  if (bno_ok) {
    float ct = cosTiltFromGravity(st.gx, st.gy, st.gz);
    st.vertical_mm = (float)st.raw_mm * ct;
  }
}

static void buildJson(String &out) {
  // Manual JSON build to avoid ArduinoJson dependency
  out = "{\"time_ms\":";
  out += millis();

  // State
  out += ",\"armed\":"; out += (st.armed ? 1 : 0);
  out += ",\"estop\":"; out += (st.estop ? 1 : 0);
  out += ",\"tool\":\""; out += st.tool; out += "\"";
  out += ",\"pose\":\""; out += st.pose; out += "\"";

  // Last command/response (truncate so JSON stays small)
  String lc = st.last_cmd; if (lc.length() > 80) lc = lc.substring(0, 80);
  String lr = st.last_resp; if (lr.length() > 80) lr = lr.substring(0, 80);
  // escape quotes minimally
  lc.replace("\"", "'");
  lr.replace("\"", "'");
  out += ",\"last_cmd\":\""; out += lc; out += "\"";
  out += ",\"last_resp\":\""; out += lr; out += "\"";

  // Sensors
  out += ",\"raw_mm\":"; out += st.raw_mm;
  out += ",\"vertical_mm\":"; out += st.vertical_mm;
  out += ",\"range_status\":"; out += (int)st.range_status;

  if (!bno_ok) out += ",\"bno_error\":1";
  if (!tof_ok) out += ",\"tof_error\":1";

  out += ",\"quat_w\":"; out += st.qw;
  out += ",\"quat_x\":"; out += st.qx;
  out += ",\"quat_y\":"; out += st.qy;
  out += ",\"quat_z\":"; out += st.qz;

  out += ",\"yaw\":"; out += st.yaw;
  out += ",\"pitch\":"; out += st.pitch;
  out += ",\"roll\":"; out += st.roll;

  out += ",\"grav_x\":"; out += st.gx;
  out += ",\"grav_y\":"; out += st.gy;
  out += ",\"grav_z\":"; out += st.gz;

  out += ",\"cal_sys\":"; out += st.cal_sys;
  out += ",\"cal_g\":"; out += st.cal_g;
  out += ",\"cal_a\":"; out += st.cal_a;
  out += ",\"cal_m\":"; out += st.cal_m;

  out += ",\"tof_stop_mm\":"; out += st.tof_stop_mm;

  out += "}";
}

static void streamTick() {
  const uint32_t now = millis();
  if (now - last_stream_ms < stream_interval_ms) return;
  last_stream_ms = now;

  updateSensorsOnce();
  String json;
  buildJson(json);
  ws.broadcastTXT(json);
}

// ------------------------
// Motion primitives (tune for your robot)
// ------------------------
static void applyPose(const String &poseName) {
  st.pose = poseName;

  if (!st.armed || st.estop) return;

  if (poseName == "FOOD_LEFT") {
    writeServoUs(PIN_SERVO_BASE, BASE_LEFT_US);
    writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
    writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);
    pump(350);
  } else if (poseName == "FOOD_CENTER") {
    writeServoUs(PIN_SERVO_BASE, BASE_CENTER_US);
    writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
    writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);
    pump(350);
  } else if (poseName == "FOOD_RIGHT") {
    writeServoUs(PIN_SERVO_BASE, BASE_RIGHT_US);
    writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
    writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);
    pump(350);
  } else if (poseName == "PRESENT") {
    // for demo: lift up + neutral tool
    writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
    writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);
    pump(350);
  } else if (poseName == "HOME") {
    writeServoUs(PIN_SERVO_BASE, BASE_CENTER_US);
    writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
    writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);
    pump(350);
  }
}

static void actionScoop() {
  if (!st.armed || st.estop) return;

  // 1) Lower toward bowl until ToF says we're close enough (or until we hit max time)
  const uint32_t max_ms = 1800;
  const uint32_t step_ms = 60;
  const uint16_t stop_mm = st.tof_stop_mm;

  const uint32_t steps = max_ms / step_ms;
  for (uint32_t i = 0; i < steps; i++) {
    // linear ramp: UP -> DOWN
    int us = (int)((float)LIFT_UP_US + (float)(LIFT_DOWN_US - LIFT_UP_US) * ((float)i / (float)(steps - 1)));
    writeServoUs(PIN_SERVO_LIFT, us);

    // keep network responsive + stream
    pump(step_ms);
    streamTick();
    updateSensorsOnce();

    if (tof_ok && st.raw_mm > 0 && st.raw_mm <= stop_mm) {
      break;
    }
  }

  // 2) Scoop motion
  writeServoUs(PIN_SERVO_TOOL, TOOL_SCOOP_US);
  pump(250);
  writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);
  pump(250);

  // 3) Raise back up
  writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
  pump(400);
}

// ------------------------
// Command parsing
// ------------------------
static String toUpperCopy(String s) {
  s.toUpperCase();
  return s;
}

static void splitFirstToken(const String &line, String &t0, String &rest) {
  int sp = line.indexOf(' ');
  if (sp < 0) {
    t0 = line;
    rest = "";
    return;
  }
  t0 = line.substring(0, sp);
  rest = line.substring(sp + 1);
  rest.trim();
}

static String handleCommand(const String &line_in) {
  String line = line_in;
  line.trim();
  if (line.length() == 0) return "";

  st.last_cmd = line;

  String cmd, rest;
  splitFirstToken(line, cmd, rest);
  String cmdU = toUpperCopy(cmd);
  String restU = toUpperCopy(rest);

  // Safety: allow only a small set of commands during ESTOP
  if (st.estop && !(cmdU == "PING" || cmdU == "STATUS" || cmdU == "ESTOP_CLEAR")) {
    st.last_resp = "ERR ESTOP";
    return st.last_resp;
  }

  if (cmdU == "PING") {
    st.last_resp = "OK PONG";
    return st.last_resp;
  }

  if (cmdU == "STATUS") {
    st.last_resp = "OK STATUS";
    return st.last_resp;
  }

  if (cmdU == "ESTOP") {
    st.estop = true;
    st.armed = false;
    st.last_resp = "OK ESTOP";
    return st.last_resp;
  }

  if (cmdU == "ESTOP_CLEAR") {
    st.estop = false;
    st.last_resp = "OK ESTOP_CLEAR";
    return st.last_resp;
  }

  if (cmdU == "ARM") {
    st.armed = true;
    st.last_resp = "OK ARM";
    return st.last_resp;
  }

  if (cmdU == "DISARM") {
    st.armed = false;
    st.last_resp = "OK DISARM";
    return st.last_resp;
  }

  if (cmdU == "TOOL") {
    // Example: TOOL SPOON
    if (restU.length() == 0) {
      st.last_resp = "ERR TOOL_MISSING";
      return st.last_resp;
    }
    st.tool = restU;
    st.last_resp = "OK TOOL " + restU;
    return st.last_resp;
  }

  if (cmdU == "POSE") {
    if (restU.length() == 0) {
      st.last_resp = "ERR POSE_MISSING";
      return st.last_resp;
    }
    applyPose(restU);
    st.last_resp = "OK POSE " + restU;
    return st.last_resp;
  }

  if (cmdU == "ACTION") {
    if (restU == "SCOOP") {
      actionScoop();
      st.last_resp = "OK ACTION SCOOP";
      return st.last_resp;
    }
    st.last_resp = "ERR ACTION_UNKNOWN";
    return st.last_resp;
  }

  if (cmdU == "SET") {
    // Example: SET TOF_STOP_MM 60
    String k, v;
    splitFirstToken(restU, k, v);
    if (k == "TOF_STOP_MM" && v.length() > 0) {
      long mmL = v.toInt();
      if (mmL < 0) mmL = 0;
      if (mmL > 2000) mmL = 2000;
      st.tof_stop_mm = (uint16_t)mmL;
      st.last_resp = "OK SET TOF_STOP_MM " + String((int)st.tof_stop_mm);
      return st.last_resp;
    }
    st.last_resp = "ERR SET_UNKNOWN";
    return st.last_resp;
  }

  st.last_resp = "ERR UNKNOWN_CMD";
  return st.last_resp;
}

// ------------------------
// Web UI (served from ESP32)
// ------------------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ASSIST Dashboard</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial; margin:16px;}
    .row{display:flex; gap:16px; flex-wrap:wrap;}
    .card{border:1px solid #ddd; border-radius:12px; padding:12px; min-width:280px;}
    .k{color:#555; font-size:12px;}
    .v{font-family:ui-monospace, SFMono-Regular, Menlo, monospace;}
    button{padding:10px 12px; margin:4px 4px 4px 0; border-radius:10px; border:1px solid #ccc; background:#fafafa; cursor:pointer;}
    button.danger{border-color:#c33;}
    #log{white-space:pre-wrap; font-family:ui-monospace, SFMono-Regular, Menlo, monospace; font-size:12px; max-height:220px; overflow:auto; border:1px solid #eee; padding:8px; border-radius:10px;}
    canvas{border:1px solid #eee; border-radius:10px; width:320px; height:320px;}
    .pill{display:inline-block; padding:4px 10px; border-radius:999px; border:1px solid #ddd; font-size:12px;}
  </style>
</head>
<body>
<h2>ASSIST ESP32 Live Dashboard</h2>
<div class="pill" id="wsStatus">WS: disconnected</div>

<div class="row">
  <div class="card">
    <div><b>Controls</b></div>
    <div>
      <button onclick="sendCmd('ARM')">ARM</button>
      <button onclick="sendCmd('DISARM')">DISARM</button>
      <button class="danger" onclick="sendCmd('ESTOP')">ESTOP</button>
      <button onclick="sendCmd('ESTOP_CLEAR')">ESTOP_CLEAR</button>
    </div>
    <div>
      <button onclick="sendCmd('POSE FOOD_LEFT')">POSE LEFT</button>
      <button onclick="sendCmd('POSE FOOD_CENTER')">POSE CENTER</button>
      <button onclick="sendCmd('POSE FOOD_RIGHT')">POSE RIGHT</button>
      <button onclick="sendCmd('POSE PRESENT')">POSE PRESENT</button>
    </div>
    <div>
      <button onclick="sendCmd('TOOL SPOON')">TOOL SPOON</button>
      <button onclick="sendCmd('ACTION SCOOP')">ACTION SCOOP</button>
    </div>

    <hr/>
    <div class="k">ToF stop threshold (mm)</div>
    <input id="tofStop" type="number" min="10" max="300" step="1" style="width:120px"/>
    <button onclick="sendCmd('SET TOF_STOP_MM ' + document.getElementById('tofStop').value)">Set</button>
  </div>

  <div class="card">
    <div><b>Robot State</b></div>
    <div class="k">armed / estop</div>
    <div class="v" id="stateFlags">-</div>

    <div class="k">tool / pose</div>
    <div class="v" id="statePose">-</div>

    <div class="k">last_cmd</div>
    <div class="v" id="lastCmd">-</div>

    <div class="k">last_resp</div>
    <div class="v" id="lastResp">-</div>
  </div>

  <div class="card">
    <div><b>Sensors</b></div>
    <div class="k">ToF raw_mm / vertical_mm / range_status</div>
    <div class="v" id="tofLine">-</div>

    <div class="k">yaw / pitch / roll</div>
    <div class="v" id="euler">-</div>

    <div class="k">quat (w,x,y,z)</div>
    <div class="v" id="quat">-</div>

    <div class="k">gravity (x,y,z)</div>
    <div class="v" id="grav">-</div>

    <div class="k">cal (sys,g,a,m)</div>
    <div class="v" id="cal">-</div>
  </div>

  <div class="card">
    <div><b>IMU Cube</b></div>
    <canvas id="c" width="320" height="320"></canvas>
  </div>
</div>

<h3>Log</h3>
<div id="log"></div>

<script>
let ws;
let lastJson = null;
const logEl = document.getElementById('log');
const statusEl = document.getElementById('wsStatus');

function logLine(s){
  const now = new Date().toLocaleTimeString();
  logEl.textContent = `[${now}] ${s}\n` + logEl.textContent;
}

function connect(){
  const url = `ws://${location.hostname}:81`;
  ws = new WebSocket(url);
  statusEl.textContent = 'WS: connecting…';

  ws.onopen = () => { statusEl.textContent = 'WS: connected'; logLine('connected'); sendCmd('STATUS'); };
  ws.onclose = () => { statusEl.textContent = 'WS: disconnected'; logLine('disconnected'); setTimeout(connect, 1000); };
  ws.onerror = (e) => { logLine('ws error'); };

  ws.onmessage = (ev) => {
    const txt = ev.data;
    if (typeof txt === 'string' && txt.length && txt[0] === '{') {
      try { lastJson = JSON.parse(txt); updateUI(lastJson); drawCube(lastJson); }
      catch(e){ /* ignore */ }
    } else {
      logLine(txt);
    }
  };
}

function sendCmd(cmd){
  if (!ws || ws.readyState !== 1) { logLine('WS not connected'); return; }
  ws.send(cmd);
  logLine('> ' + cmd);
}

function f2(x){
  if (x === undefined || x === null) return '-';
  const n = Number(x);
  return Number.isFinite(n) ? n.toFixed(2) : String(x);
}

function updateUI(j){
  document.getElementById('stateFlags').textContent = `armed=${j.armed}  estop=${j.estop}`;
  document.getElementById('statePose').textContent  = `tool=${j.tool}  pose=${j.pose}`;
  document.getElementById('lastCmd').textContent    = j.last_cmd;
  document.getElementById('lastResp').textContent   = j.last_resp;

  document.getElementById('tofLine').textContent = `raw_mm=${j.raw_mm}  vertical_mm=${f2(j.vertical_mm)}  status=${j.range_status}`;
  document.getElementById('euler').textContent   = `yaw=${f2(j.yaw)}  pitch=${f2(j.pitch)}  roll=${f2(j.roll)}`;
  document.getElementById('quat').textContent    = `w=${f2(j.quat_w)} x=${f2(j.quat_x)} y=${f2(j.quat_y)} z=${f2(j.quat_z)}`;
  document.getElementById('grav').textContent    = `x=${f2(j.grav_x)} y=${f2(j.grav_y)} z=${f2(j.grav_z)}`;
  document.getElementById('cal').textContent     = `sys=${j.cal_sys} g=${j.cal_g} a=${j.cal_a} m=${j.cal_m}`;

  const ts = document.getElementById('tofStop');
  if (!ts.value) ts.value = j.tof_stop_mm;
}

// --- Minimal cube renderer (no external libs) ---
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

const V = [
  [-1,-1,-1],[ 1,-1,-1],[ 1, 1,-1],[-1, 1,-1],
  [-1,-1, 1],[ 1,-1, 1],[ 1, 1, 1],[-1, 1, 1]
];
const E = [
  [0,1],[1,2],[2,3],[3,0],
  [4,5],[5,6],[6,7],[7,4],
  [0,4],[1,5],[2,6],[3,7]
];

function quatToMat(w,x,y,z){
  // normalize
  const n = Math.hypot(w,x,y,z) || 1;
  w/=n; x/=n; y/=n; z/=n;
  const xx=x*x, yy=y*y, zz=z*z;
  const xy=x*y, xz=x*z, yz=y*z;
  const wx=w*x, wy=w*y, wz=w*z;
  return [
    [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
    [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
    [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
  ];
}

function mulMV(m,v){
  return [
    m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
    m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
    m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]
  ];
}

function project(p){
  const fov = 2.2;
  const z = p[2] + 3.5;
  const s = fov / z;
  return [p[0]*s, p[1]*s];
}

function drawCube(j){
  if (!j) return;
  const w=j.quat_w, x=j.quat_x, y=j.quat_y, z=j.quat_z;
  const m = quatToMat(w,x,y,z);

  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.lineWidth = 2;
  ctx.strokeStyle = '#111';

  const pts = V.map(v => {
    const r = mulMV(m,v);
    const p = project(r);
    const cx = canvas.width/2 + p[0]*120;
    const cy = canvas.height/2 + p[1]*120;
    return [cx, cy];
  });

  ctx.beginPath();
  for (const [a,b] of E){
    ctx.moveTo(pts[a][0], pts[a][1]);
    ctx.lineTo(pts[b][0], pts[b][1]);
  }
  ctx.stroke();

  // draw front-top-left vertex for orientation
  ctx.fillStyle = '#d33';
  ctx.beginPath();
  ctx.arc(pts[7][0], pts[7][1], 6, 0, Math.PI*2);
  ctx.fill();
}

connect();
</script>
</body>
</html>
)rawliteral";

static void handleRoot() {
  http.send_P(200, "text/html", INDEX_HTML);
}

static void handleHealth() {
  http.send(200, "text/plain", "ok");
}

static void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] client %u connected\n", num);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] client %u disconnected\n", num);
    return;
  }
  if (type == WStype_TEXT) {
    // queue the command for the main loop
    if (!ws_cmd_pending) {
      ws_cmd_client = num;
      ws_cmd_line = "";
      ws_cmd_line.reserve(len + 1);
      for (size_t i = 0; i < len; i++) ws_cmd_line += (char)payload[i];
      ws_cmd_pending = true;
    } else {
      // Busy: drop, but tell the client.
      ws.sendTXT(num, "ERR BUSY");
    }
  }
}

// ------------------------
// Setup + main loop
// ------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  st.tof_stop_mm = TOF_STOP_MM_DEFAULT;

  Serial.println("\n--- ASSIST unified firmware ---");
  Serial.printf("DRY_RUN=%d\n", (int)DRY_RUN);

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100);

  // ToF init (with XSHUT)
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
    Serial.println("[ToF] VL53L1X OK");
  } else {
    Serial.println("[ToF] VL53L1X init FAIL (still running)");
  }

  // IMU init
  delay(650);
  if (bno.begin()) {
    bno.setExtCrystalUse(false);
    delay(100);
    bno_ok = true;
    Serial.println("[IMU] BNO055 OK");
  } else {
    Serial.println("[IMU] BNO055 init FAIL (still running)");
  }

  // Servo attach + safe defaults
  attachServoPin(PIN_SERVO_BASE);
  attachServoPin(PIN_SERVO_LIFT);
  attachServoPin(PIN_SERVO_TOOL);

  writeServoUs(PIN_SERVO_BASE, BASE_CENTER_US);
  writeServoUs(PIN_SERVO_LIFT, LIFT_UP_US);
  writeServoUs(PIN_SERVO_TOOL, TOOL_NEUTRAL_US);

  // Wi-Fi AP + HTTP + WS
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  IPAddress ap = WiFi.softAPIP();
  Serial.printf("[WiFi] AP SSID=%s PASS=%s\n", WIFI_AP_SSID, WIFI_AP_PASS);
  Serial.printf("[WiFi] AP IP=%s\n", ap.toString().c_str());

  http.on("/", HTTP_GET, handleRoot);
  http.on("/health", HTTP_GET, handleHealth);
  http.begin();

  ws.begin();
  ws.onEvent(wsEvent);
  Serial.printf("[WS] port=%d stream=%d Hz\n", WS_PORT, STREAM_HZ);

  // initial pose
  applyPose("HOME");
}

void loop() {
  http.handleClient();
  ws.loop();

  // Serial line reader
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      String cmd = serial_line;
      serial_line = "";

      String resp = handleCommand(cmd);
      if (resp.length()) {
        // IMPORTANT: Pi waits for a line starting with OK/ERR.
        Serial.println(resp);
        // Also show in dashboard log
        ws.broadcastTXT(resp);
      }
    } else {
      serial_line += c;
      if (serial_line.length() > 200) {
        serial_line = ""; // avoid runaway
      }
    }
  }

  // periodic sensor stream
  streamTick();

  // Process one queued WebSocket command (if any)
  if (ws_cmd_pending) {
    // capture + clear
    uint8_t client = ws_cmd_client;
    String cmd = ws_cmd_line;
    ws_cmd_pending = false;

    String resp = handleCommand(cmd);
    if (resp.length()) {
      ws.sendTXT(client, resp);
      ws.broadcastTXT(resp);
    }
  }
}
