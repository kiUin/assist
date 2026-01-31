#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_DF2301Q.h"

// ================= SERVO =================
static const int SERVO_PIN = 18;
static const int PWM_FREQ  = 50;
static const int PWM_RES   = 16;

uint32_t usToDuty(int us) {
  const int period_us = 20000;
  const uint32_t maxDuty = (1u << PWM_RES) - 1;
  us = constrain(us, 500, 2500);
  return (uint32_t)((uint64_t)us * maxDuty / period_us);
}

void setServoUs(int us) {
  // ESP32 Arduino core 3.x: ledcWrite uses PIN
  ledcWrite(SERVO_PIN, usToDuty(us));
}

// ================= VOICE =================
DFRobot_DF2301Q_I2C voice;
uint8_t lastCmd = 0;

// ---- EDIT THESE BASED ON WHAT YOU OBSERVE ----
const uint8_t CMD_WAKE  = 2;   // "hello robot" (your wake)
const uint8_t CMD_SPOON = 5;   // "spoon" (you observed)

// Optional: add more as you discover them
// const uint8_t CMD_FORK  = 6;
// const uint8_t CMD_STOP  = 7;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\nBOOT OK");
  Serial.println("=== DF2301Q -> SERVO (MAP OBSERVED IDS) ===");

  // Servo PWM (ESP32 core 3.x)
  if (!ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES)) {
    Serial.println("ERROR: ledcAttach failed. Try a different SERVO_PIN.");
    while (1) delay(1000);
  }
  setServoUs(1500);
  Serial.println("Servo centered (1500us).");

  // I2C
  Wire.begin(21, 22);

  if (!voice.begin()) {
    Serial.println("ERROR: DF2301Q not detected!");
    while (1) delay(1000);
  }

  voice.setVolume(10);      // louder helps recognition
  voice.setWakeTime(255);   // stay awake longer for debugging
  Serial.println("DF2301Q ready.");
  Serial.println("Say wake word then commands. Watching CMD IDs...");
}

void loop() {
  uint8_t cmd = voice.getCMDID();

  // only react on edges
  if (cmd != 0 && cmd != lastCmd) {
    lastCmd = cmd;

    Serial.print("CMD = ");
    Serial.println(cmd);

    // 1) Ignore wake command if you want
    if (cmd == CMD_WAKE) {
      Serial.println("(wake detected; ignoring)");
      return;
    }

    // ================= MAP HERE =================
    if (cmd == CMD_SPOON) {
      Serial.println("SPOON -> SERVO RIGHT");
      setServoUs(1900);
    }
    // else if (cmd == CMD_FORK) {
    //   Serial.println("FORK -> SERVO LEFT");
    //   setServoUs(1100);
    // }
    // else if (cmd == CMD_STOP) {
    //   Serial.println("STOP -> SERVO CENTER");
    //   setServoUs(1500);
    // }
    else {
      Serial.println("Unmapped CMD (add it to mapping)");
    }
  }

  // reset latch when idle
  if (cmd == 0) lastCmd = 0;

  delay(50);
}
