#include "hal/platform.h"
#include <Arduino.h>
#include <SPI.h>
#include "Configuration.h"
#include "spi_protocol_shared.h"
#if defined(SHURIKEN_P4_CONTROLLER)
#include <driver/gpio.h>
static SPIClass scanner_spi(FSPI);
#define SCANNER_SPI scanner_spi
#else
#include "hardware/gpio.h"
#define SCANNER_SPI SPI1
#endif
namespace {
static constexpr size_t SCANNER_FRAME_SIZE = SPI_FRAME_SIZE;
static constexpr int SCANNER_PIN_SCK = ESP_PIN_SCK;
static constexpr int SCANNER_PIN_MOSI = ESP_PIN_MOSI;
static constexpr int SCANNER_PIN_MISO = ESP_PIN_MISO;
#if defined(ESP_PIN_CS)
static constexpr int SCANNER_PIN_CS = ESP_PIN_CS;
#endif
#if SCANNER_USE_SHIFTREG_CS
static constexpr uint8_t SCANNER_SLOT_COUNT = SCANNER_SHIFTREG_OUTPUTS;
static uint8_t scanner_cs_shiftreg_state = 0xFF;
static bool scanner_shiftreg_outputs_enabled = false;
#else
static constexpr uint8_t SCANNER_SLOT_COUNT = 1;
#endif
static uint8_t scanner_active_slot = SCANNER_INITIAL_SLOT;
static inline void scannerSetActiveSlot(uint8_t slot) {
#if SCANNER_USE_SHIFTREG_CS
  // The active slot selects which 74HC595 output will assert CS for the next
  // transfer. With direct CS wiring there is only one logical slot.
  scanner_active_slot = (uint8_t)(slot % SCANNER_SLOT_COUNT);
#else
  (void)slot;
  scanner_active_slot = 0;
#endif
}

#if SCANNER_USE_SHIFTREG_CS
static inline void scannerShiftRegSetOutputsEnabled(bool enabled) {
  scanner_shiftreg_outputs_enabled = enabled;
  // 74HC595 OE is active-low.
  digitalWrite(SCANNER_SHIFTREG_OE_PIN, enabled ? LOW : HIGH);
}

static inline void scannerShiftRegWriteCsState(uint8_t state, bool force) {
  // Updating the shift register itself uses SPI1, so temporarily disable the
  // scanner outputs to avoid clocking garbage into a selected slave.
  if (!force && scanner_cs_shiftreg_state == state) {
    return;
  }
  const bool restore_outputs = scanner_shiftreg_outputs_enabled;
  if (restore_outputs) {
    // Prevent SPI clocks for shift-register updates from reaching any selected scanner.
    scannerShiftRegSetOutputsEnabled(false);
  }
  scanner_cs_shiftreg_state = state;
  SCANNER_SPI.beginTransaction(SPISettings(SCANNER_SHIFTREG_SPI_HZ, MSBFIRST, SPI_MODE0));
  SCANNER_SPI.transfer(scanner_cs_shiftreg_state);
  SCANNER_SPI.endTransaction();
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, HIGH);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  if (restore_outputs) {
    scannerShiftRegSetOutputsEnabled(true);
  }
}
#endif

static void transferFrame(const uint8_t* tx, uint8_t* rx, uint32_t spi_hz) {
#if defined(SHURIKEN_P4_CONTROLLER)
  // Scanner-worker-owned timing: cover every frame, including a new command
  // immediately after a successful reply, not just failed-response polls.
  static bool have_previous_frame = false;
  static uint32_t previous_frame_end_us = 0;
  if (have_previous_frame) {
    const uint32_t elapsed_us = static_cast<uint32_t>(micros()) - previous_frame_end_us;
    if (elapsed_us < P4_SCANNER_FRAME_GAP_US) {
      delayMicroseconds(P4_SCANNER_FRAME_GAP_US - elapsed_us);
    }
  }
#endif
#if SCANNER_USE_SHIFTREG_CS
  // Active-low CS via shift-register output slot.
  const uint8_t assert_state = (uint8_t)(0xFFu & ~(1u << scanner_active_slot));
  scannerShiftRegWriteCsState(assert_state, false);
#else
  digitalWrite(SCANNER_PIN_CS, LOW);
#endif
  SCANNER_SPI.beginTransaction(SPISettings(spi_hz, MSBFIRST, SPI_MODE0));
#if defined(SHURIKEN_P4_CONTROLLER)
  delayMicroseconds(P4_SCANNER_CS_SETUP_US);
#endif

  // The slave protocol is fixed-length and full-duplex, so every command and
  // poll is exactly one 64-byte transfer.
  for (size_t i = 0; i < SCANNER_FRAME_SIZE; i++) {
    const uint8_t t = tx ? tx[i] : 0;
    const uint8_t r = SCANNER_SPI.transfer(t);
    if (rx) {
      rx[i] = r;
    }
  }

#if defined(SHURIKEN_P4_CONTROLLER)
  delayMicroseconds(P4_SCANNER_CS_HOLD_US);
#endif
  SCANNER_SPI.endTransaction();
#if SCANNER_USE_SHIFTREG_CS
  // Deassert all scanner CS outputs.
  const uint8_t deassert_state = 0xFFu;
  scannerShiftRegWriteCsState(deassert_state, false);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
#endif
#if defined(SHURIKEN_P4_CONTROLLER)
  previous_frame_end_us = static_cast<uint32_t>(micros());
  have_previous_frame = true;
#endif
}

bool beginBus() {
  // Bring the scanner bus to a safe idle state before core1 starts polling.
#if SCANNER_USE_SHIFTREG_CS
  pinMode(SCANNER_SHIFTREG_LATCH_PIN, OUTPUT);
  pinMode(SCANNER_SHIFTREG_OE_PIN, OUTPUT);
  digitalWrite(SCANNER_SHIFTREG_LATCH_PIN, LOW);
  // Active-low OE: keep outputs disabled while shift register state is initialized.
  scannerShiftRegSetOutputsEnabled(false);
#else
  digitalWrite(SCANNER_PIN_CS, HIGH);
  pinMode(SCANNER_PIN_CS, OUTPUT);
#endif
#if defined(SHURIKEN_P4_CONTROLLER)
  if (!SCANNER_SPI.begin(SCANNER_PIN_SCK, SCANNER_PIN_MISO, SCANNER_PIN_MOSI, -1)) return false;
#if SCANNER_MISO_PULLUP
  // pinMode() would reclaim MISO from Arduino's peripheral manager and
  // detach its SPI input routing. Change only the pad pull-up instead.
  if (gpio_pullup_en(static_cast<gpio_num_t>(SCANNER_PIN_MISO)) != ESP_OK) {
    SCANNER_SPI.end();
    return false;  // CS outputs remain disabled on initialization failure.
  }
#endif
#else
#if SCANNER_MISO_PULLUP
  gpio_pull_up(SCANNER_PIN_MISO);
#endif
  SPI1.setSCK(SCANNER_PIN_SCK);
  SPI1.setTX(SCANNER_PIN_MOSI);
  SPI1.setRX(SCANNER_PIN_MISO);
  SPI1.begin();
#if SCANNER_MISO_PULLUP
  gpio_pull_up(SCANNER_PIN_MISO);
#endif
#endif
#if SCANNER_USE_SHIFTREG_CS
  scannerSetActiveSlot(SCANNER_INITIAL_SLOT);
  scannerShiftRegWriteCsState(0xFFu, true);
  // Keep outputs enabled in idle once all scanner CS lines are safely deasserted.
  scannerShiftRegSetOutputsEnabled(true);
#endif
  return true;
}

class ArduinoScannerTransport final : public controller::hal::ScannerTransport {
 public:
  bool begin() override { ready_ = beginBus(); return ready_; }
  uint8_t slotCount() const override { return SCANNER_SLOT_COUNT; }
  bool transfer(uint8_t slot, uint32_t hz, const uint8_t* tx, uint8_t* rx) override {
    if (!ready_ || slot >= slotCount() || hz == 0) return false;
    scannerSetActiveSlot(slot);
    transferFrame(tx, rx, hz);
    return true;
  }
 private:
  bool ready_ = false;
};
}
controller::hal::ScannerTransport& controller::hal::scannerTransport() {
  static ArduinoScannerTransport transport;
  return transport;
}
