#include <Arduino.h>

#include "ScannerConfiguration.h"

namespace {

static constexpr uint32_t REPORT_INTERVAL_MS = 1000;
static constexpr uint16_t SAMPLE_BURST_COUNT = 512;

struct PinMonitor {
  int pin;
  const char* name;
  int last_level;
  uint32_t sampled_edges;
  bool active_in_window;
};

PinMonitor g_pin_monitors[] = {
  {PIN_SCK,  "SCK",  LOW, 0, false},
  {PIN_MOSI, "MOSI", LOW, 0, false},
  {PIN_MISO, "MISO", LOW, 0, false},
  {PIN_CS,   "CS",   LOW, 0, false},
};

template <size_t N>
void resetWindow(PinMonitor (&pins)[N]) {
  for (size_t i = 0; i < N; i++) {
    pins[i].sampled_edges = 0;
    pins[i].active_in_window = false;
  }
}

template <size_t N>
void samplePins(PinMonitor (&pins)[N]) {
  for (size_t i = 0; i < N; i++) {
    const int level = digitalRead(pins[i].pin);
    if (level != pins[i].last_level) {
      pins[i].last_level = level;
      pins[i].sampled_edges++;
      pins[i].active_in_window = true;
    }
  }
}

template <size_t N>
void printWindowReport(const PinMonitor (&pins)[N], uint32_t elapsed_ms) {
  bool any_activity = false;
  bool first_active = true;
  bool first_inactive = true;

  Serial.printf("[SPI-DIAG] %lums window | active:", (unsigned long)elapsed_ms);
  for (size_t i = 0; i < N; i++) {
    if (!pins[i].active_in_window) {
      continue;
    }
    any_activity = true;
    Serial.print(first_active ? " " : ", ");
    first_active = false;
    Serial.printf("%s(edges~%lu level=%s)",
                  pins[i].name,
                  (unsigned long)pins[i].sampled_edges,
                  pins[i].last_level ? "HIGH" : "LOW");
  }
  if (!any_activity) {
    Serial.print(" none");
  }

  Serial.print(" | no activity:");
  for (size_t i = 0; i < N; i++) {
    if (pins[i].active_in_window) {
      continue;
    }
    Serial.print(first_inactive ? " " : ", ");
    first_inactive = false;
    Serial.printf("%s(level=%s)",
                  pins[i].name,
                  pins[i].last_level ? "HIGH" : "LOW");
  }
  if (first_inactive) {
    Serial.print(" none");
  }

  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println("ESP32-C5 SPI activity diagnostic");
  Serial.printf("Monitoring pins: SCK=%d MOSI=%d MISO=%d CS=%d\n",
                PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS);
  Serial.println("Reports are sampled activity windows, not exact edge counts.");

  for (PinMonitor& pin : g_pin_monitors) {
    pinMode(pin.pin, INPUT);
    pin.last_level = digitalRead(pin.pin);
    pin.sampled_edges = 0;
    pin.active_in_window = false;
  }
}

void loop() {
  static uint32_t last_report_ms = millis();

  for (uint16_t i = 0; i < SAMPLE_BURST_COUNT; i++) {
    samplePins(g_pin_monitors);
  }

  const uint32_t now = millis();
  if ((now - last_report_ms) >= REPORT_INTERVAL_MS) {
    const uint32_t elapsed_ms = now - last_report_ms;
    printWindowReport(g_pin_monitors, elapsed_ms);
    resetWindow(g_pin_monitors);
    last_report_ms = now;
  }

  delay(0);
}
