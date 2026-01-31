#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_DF2301Q.h"

// VOICE RECOGNITION MODULE COMMANDS: LEARNING WAKE WORD, LEARNING COMMAND WORD, EXIT LEARNING, I WANT TO DELETE. 
// MORE INFROMATION: https://wiki.dfrobot.com/SKU_SEN0539-EN_Gravity_Voice_Recognition_Module_I2C_UART#target_4 
#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_DF2301Q.h"

// ================= SERVO =================
// ESP32 Arduino core 3.x LEDC API: ledcAttach(pin,freq,res) + ledcWrite(pin,duty)
static const int SERVO_PIN = 18;
static const int PWM_FREQ  = 50;   // 50 Hz for servos
static const int PWM_RES   = 16;   // 16-bit resolution

uint32_t usToDuty(int us) {
  const int period_us = 20000;                  // 20ms period at 50Hz
  const uint32_t maxDuty = (1u << PWM_RES) - 1; // 65535
  us = constrain(us, 500, 2500);
  return (uint32_t)((uint64_t)us * maxDuty / period_us);
}

void setServoUs(int us) {
  ledcWrite(SERVO_PIN, usToDuty(us));
}

// ================= VOICE =================
DFRobot_DF2301Q_I2C voice;

// ================= CMD MAPPING =================
const uint8_t CMD_WAKE  = 2;  // "hello robot"
const uint8_t CMD_SPOON = 5;  // "spoon"
const uint8_t CMD_FORK  = 6;  // "fork"

// Edge-detect + cooldown
uint8_t lastCmd = 0;
uint32_t lastFireMs = 0;
const uint32_t COOLDOWN_MS = 600;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\nBOOT OK");
  Serial.println("=== DF2301Q -> SERVO (SPOON/FORK) ===");

  // Servo setup
  if (!ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES)) {
    Serial.println("ERROR: ledcAttach failed (bad pin or LEDC init).");
    while (1) delay(1000);
  }
  setServoUs(1500);
  Serial.println("Servo centered (1500us).");

  // I2C setup (adjust pins if your board uses different defaults)
  Wire.begin(21, 22);
  delay(200);

  // Voice init
  if (!voice.begin()) {
    Serial.println("ERROR: DF2301Q not detected!");
    while (1) delay(1000);
  }

  // Make the module quieter + stable
  voice.setVolume(15);       // lower = quieter prompts; increase if you miss commands
  voice.setWakeTime(255);   // stay awake longer

  Serial.println("DF2301Q ready.");
  Serial.println("Wake -> say 'spoon' or 'fork'. Watching CMD IDs...");
}

void loop() {
  uint8_t cmd = voice.getCMDID();

  // React only when it changes to a nonzero value
  if (cmd != 0 && cmd != lastCmd) {
    lastCmd = cmd;

    Serial.print("CMD = ");
    Serial.println(cmd);

    // Ignore wake command
    if (cmd == CMD_WAKE) {
      Serial.println("(wake detected; ignored)");
      return;
    }

    // Cooldown to reduce double triggers
    uint32_t now = millis();
    if (now - lastFireMs < COOLDOWN_MS) {
      Serial.println("(cooldown; ignored)");
      return;
    }
    lastFireMs = now;

    // Map commands
    if (cmd == CMD_SPOON) {
      Serial.println("SPOON -> SERVO RIGHT (1900us)");
      setServoUs(1900);
    } else if (cmd == CMD_FORK) {
      Serial.println("FORK -> SERVO LEFT (1100us)");
      setServoUs(1100);
    } else {
      Serial.println("Unmapped CMD (add it to mapping).");
    }
  }

  // Reset latch when idle
  if (cmd == 0) lastCmd = 0;

  delay(50);
}
