# ESP32 IMU + ToF AP Mode Viewer

ESP32 runs as a Wi‑Fi Access Point, streams BNO055 IMU and VL53L1X ToF data as JSON over WebSocket at 25 Hz. A single HTML viewer on the laptop shows live readouts and a 3D cube driven by the quaternion.

## Hardware

- **ELEGOO ESP32 Dev Board** (USB‑C, CP2102, standard ESP32)
- **Adafruit BNO055** (I2C)
- **VL53L1X ToF** (I2C), sensor pointing **down**

### Wiring

| Signal   | ESP32  | BNO055 / VL53L1X |
|----------|--------|-------------------|
| SDA      | GPIO 21| SDA               |
| SCL      | GPIO 22| SCL               |
| 3.3V     | 3.3V   | VIN               |
| GND      | GND    | GND               |
| (VL53L1X only) XSHUT | GPIO 19 | XSHUT      |

**BNO055 address:** Wire **ADR** (or **AD0**) to **GND** so the IMU is at I2C address **0x28**.

### If I2C scan shows no addresses

- **Power:** Each sensor needs **3.3V → VIN** and **GND → GND**. If only SDA/SCL are connected, nothing will respond.
- **Pins:** SDA must go to **GPIO 21**, SCL to **GPIO 22** on the ESP32. Check your board’s pinout; labels may differ.
- **VL53L1X:** XSHUT must be connected to **GPIO 19** (and driven high in software); otherwise the ToF stays in reset.
- **Connections:** Reseat jumpers and check for loose or broken wires. Try one sensor at a time (e.g. only BNO055) to isolate a bad connection.

## Axis Conventions and `vertical_mm`

- **BNO055** reports gravity in the **sensor (body) frame**: `(gx, gy, gz)` in m/s².
- The ToF is assumed to measure along the body **−Z** axis (physical “down” when the board is level).
- Body **+Z** = up when level, so the “down” unit vector in body frame is **(0, 0, −1)**.
- **Tilt** = angle between the sensor’s down axis (−Z) and the true gravity direction.
  - `cos(tilt) = dot(gravity_unit, down_unit) = -gz / |g|`.
- **vertical_mm** = `raw_mm * cos(tilt)` (clamped so cos(tilt) ∈ [0, 1]), i.e. range projected onto the vertical (gravity) direction.

## Software Setup

### 1. Arduino IDE / PlatformIO

- Install **ESP32** support (Arduino core for ESP32).
- Install these libraries (Arduino Library Manager or GitHub):
  - **Adafruit BNO055**
  - **Adafruit Unified Sensor**
  - **Pololu VL53L1X**
  - **WebSockets** by Markus Sattler (Links2004) — provides `WebSocketsServer.h`

### 2. Flash the ESP32

1. Open `esp32_imu_tof_viewer.ino` in Arduino IDE (the folder name must match the sketch name).
2. Select board: **ESP32 Dev Module** (or ELEGOO ESP32).
3. Set **Upload Speed** and **CPU Frequency** as needed; **115200** for Serial is set in the sketch.
4. Set SSID/password in the config block at the top if you want different AP credentials.
5. Connect the ESP32 via USB and upload.

### 3. Connect Laptop to ESP32 AP

1. On the laptop, open Wi‑Fi settings.
2. Connect to the network:
   - **SSID:** `IMU_TOF_AP`
   - **Password:** `imu12345`
3. The ESP32 AP IP is **192.168.4.1** (default). No router needed.

### 4. Open the Viewer

1. Open `viewer.html` in a browser (double‑click or File → Open).
2. The default WebSocket URL is `ws://192.168.4.1:81`. Change it if you changed port in the sketch.
3. Click **Connect**. You should see:
   - Connection status and packet rate (Hz).
   - Live numeric readouts (raw_mm, vertical_mm, quaternion, Euler, gravity, cal status, range_status).
   - 3D cube rotating with the IMU quaternion.
4. If the cube orientation is wrong, enable **Axis remap** (swap x/y of the quaternion for display).

## Serial Debug (115200)

- I2C scan runs at boot; detected addresses are printed.
- Each JSON packet is printed at 25 Hz.
- If BNO055 or VL53L1X fails to init, the sketch keeps running and sends error/placeholder fields in JSON.

## Config (top of .ino)

- `WIFI_AP_SSID` / `WIFI_AP_PASS` — AP name and password.
- `WS_PORT` — WebSocket server port (default 81).
- `STREAM_HZ` — JSON rate (default 25).
- `I2C_SDA` / `I2C_SCL` — I2C pins (21, 22).
- `VL53L1X_XSHUT` — XSHUT pin (19).

## JSON Keys (WebSocket)

Included in each packet:  
`time_ms`, `raw_mm`, `vertical_mm`, `quat_w`, `quat_x`, `quat_y`, `quat_z`, `yaw`, `pitch`, `roll`, `grav_x`, `grav_y`, `grav_z`, `cal_sys`, `cal_g`, `cal_a`, `cal_m`, `range_status`.  
When BNO055 is OK: `linacc_x/y/z`, `temp_c`, `acc_x/y/z`, `gyro_x/y/z`, `mag_x/y/z`.  
On BNO055 init failure: `bno_error` = 1 and placeholder values for orientation/gravity.
