#ifndef HC12_INTERFACE_H
#define HC12_INTERFACE_H

#include <Arduino.h>

// --- HC-12 pin configuration ---
#define HC12_RX 16
#define HC12_TX 17

// --- Data structure for received control values ---
struct ControlData {
  double lin_vel;              // Linear velocity
  double ang_vel;              // Angular velocity
  int roll;                    // Roll command
  int pitch;                   // Pitch command
  bool jump;                    // Jump action
  bool reset_roll_pitch;        // Reset roll/pitch
  bool disable_movement;        // Disable motion
  bool reset_all;               // Reset all states
  bool decrease_height;         // Lower stance
  bool increase_height;         // Raise stance
  bool tune_level_up;           // Fine-tune up (e.g. L1)
  bool tune_level_down;         // Fine-tune down (e.g. R1)
};

// --- Global instance to hold parsed data ---
ControlData ctrlData;

// --- Internal variables ---
String hc12_rxBuffer = "";
unsigned long lastHCSend = 0;
const unsigned long hc12SendInterval = 20;  // ms
int hc12Counter = 0;

// --- Initialization ---
void initHC12() {
  Serial2.begin(115200, SERIAL_8N1, HC12_RX, HC12_TX);
  // Serial.println("[HC12] Interface initialized.");
}

// --- Optional periodic status message ---
void hc12PeriodicSend(double a = 0, double b=0, double c = 0, double d = 0) {
  unsigned long now = millis();
  if (now - lastHCSend >= hc12SendInterval) {
    lastHCSend = now;
    String msg = String(a) + String(b) + String(c) + String(d) + "\r\n";
    Serial2.print(msg);
    // Serial.print("[HC12] Sent -> ");
    // Serial.print(msg);
  }
}

// --- Parse one tab-separated line ---
void parseHC12Line(const String& line) {
  // Expected format:
  // lin_vel ang_vel roll pitch jump reset_roll_pitch disable_movement reset_all decrease_height increase_height tune_level_up tune_level_down
  // Example: "0.12\t-0.25\t1\t0\t0\t0\t0\t0\t0\t1\t0\t1"

  double values[12] = {0};
  int field = 0;
  int lastIndex = 0;

  for (int i = 0; i < line.length(); i++) {
    if (line[i] == '\t' || i == line.length() - 1) {
      String token = (i == line.length() - 1)
                     ? line.substring(lastIndex, i + 1)
                     : line.substring(lastIndex, i);
      token.trim();
      if (token.length() > 0 && field < 12)
        values[field++] = token.toDouble();
      lastIndex = i + 1;
    }
  }

  // Assign parsed values
  ctrlData.lin_vel          = values[0];
  ctrlData.ang_vel          = values[1];
  ctrlData.roll             = (int)values[2];
  ctrlData.pitch            = (int)values[3];
  ctrlData.jump             = (int)values[4];
  ctrlData.reset_roll_pitch = (int)values[5];
  ctrlData.reset_all        = (int)values[6];
  ctrlData.disable_movement = (int)values[7];
  ctrlData.decrease_height  = (int)values[8];
  ctrlData.increase_height  = (int)values[9];
  ctrlData.tune_level_up    = (int)values[10];
  ctrlData.tune_level_down  = (int)values[11];
}

// --- Read and parse incoming HC-12 data ---
void updateHC12() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      hc12_rxBuffer.trim();
      if (hc12_rxBuffer.length() > 0)
        parseHC12Line(hc12_rxBuffer);
      hc12_rxBuffer = "";
    } else {
      hc12_rxBuffer += c;
    }
  }
}

#endif  // HC12_INTERFACE_H
