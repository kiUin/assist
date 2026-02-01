# ASSIST unified ESP32 firmware (Serial + IMU/ToF + Web dashboard)

This folder contains a **single Arduino sketch** (`assist_controller.ino`) that combines:

- **USB Serial command receiver** (so your Raspberry Pi vision code can command the arm)
- **BNO055 IMU + VL53L1X ToF** sensor streaming
- **ESP32 Wi‑Fi Access Point** + a dashboard served at `http://192.168.4.1/`
- **WebSocket JSON stream** at `ws://192.168.4.1:81`

It is designed to be compatible with the command style in your Python script:

- `PING`
- `ARM`
- `TOOL SPOON`
- `POSE FOOD_LEFT` / `POSE FOOD_CENTER` / `POSE FOOD_RIGHT` / `POSE PRESENT`
- `ACTION SCOOP`

The ESP32 replies with **one line** starting with `OK` or `ERR` for each command.

---

## Hardware wiring (default pins)

### I2C sensors

| Device | SDA | SCL | Power | Notes |
|---|---:|---:|---|---|
| BNO055 | GPIO21 | GPIO22 | 3.3V | Default I2C addr 0x28 if ADR->GND |
| VL53L1X | GPIO21 | GPIO22 | 3.3V | Uses `XSHUT` on GPIO19 (edit in code if needed) |

### Servo outputs (optional)

The sketch has **example** servo pins + pulse widths. You must tune them.

| Function | Default pin | Purpose |
|---|---:|---|
| BASE | GPIO16 | left/center/right selection |
| LIFT | GPIO17 | approach toward bowl (ToF stop uses this) |
| TOOL | GPIO18 | spoon "scoop" rotation |

> Safety: the sketch defaults to `#define DRY_RUN 1`, which **disables servo movement**.

---

## Arduino IDE setup

1. Install **ESP32 boards** in Arduino IDE.
2. Install libraries:
   - **WebSockets** (Markus Sattler / Links2004)
   - **Adafruit BNO055**
   - **Adafruit Unified Sensor**
   - **VL53L1X** (Pololu)
3. Open `assist_controller.ino` and select your ESP32 board/port.
4. Upload.

---

## Demo steps

### A) Web dashboard (no Raspberry Pi needed)

1. Power the ESP32.
2. On your laptop/phone, connect to Wi‑Fi:
   - **SSID:** `ASSIST_AP`
   - **Password:** `assist123`
3. Open a browser:
   - `http://192.168.4.1/`
4. You should see live IMU/ToF data. The buttons send commands over WebSocket.

### B) With your Raspberry Pi vision code

1. Plug the ESP32 into the Raspberry Pi over USB.
2. On the Pi, run your Python vision script (example):

```bash
python3 bite_station.py --port auto --baud 115200
```

3. The Pi should print a handshake success after sending `PING`.
4. Press `n` in the OpenCV window to trigger a single cycle. The dashboard will show:
   - last command sent
   - last response
   - current pose/tool

---

## Notes on "stop when close to food"

Because a **base-mounted camera** can’t reliably see the spoon tip depth, the sketch uses the **ToF sensor** to stop the approach:

- During `ACTION SCOOP`, the firmware lowers the **LIFT** servo until `raw_mm <= tof_stop_mm`.
- You can change the stop threshold from the dashboard with:
  - `SET TOF_STOP_MM 60`

This is a simple demo loop. For a real feeding device, add:

- hard mechanical limits
- software limits (max travel, max time)
- an external **E‑Stop** button
- slow speed profiles

