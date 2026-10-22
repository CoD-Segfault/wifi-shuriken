"""Prevent target selection and hardware APIs from creeping back into the app."""
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parents[1]
failures = []
for folder in (ROOT / "src/controller", ROOT / "include/controller"):
    for source in folder.iterdir():
        text = source.read_text()
        if re.search(r"^\s*#\s*(if|ifdef|ifndef|elif).*\b(SHURIKEN_|P4_|ARDUINO_|RP2350|ESP32)", text, re.M):
            failures.append(f"{source}: target-dependent application code")
        if re.search(r"^\s*#\s*include\s*[<\"](?:p4/|pico/|hardware/|freertos/|driver/)", text, re.M):
            failures.append(f"{source}: hardware-specific include")
scanner = (ROOT / "src/controller/scanner_runtime.cpp").read_text()
if re.search(r"SHURIKEN_|Adafruit_NeoPixel|SdFat|freertos/|hardware/", scanner):
    failures.append("Shared scanner runtime contains target selection or SDK access")
logger = (ROOT / "lib/controller_logging/controller_logging.cpp").read_text()
if re.search(r"SHURIKEN_|Adafruit_|SdFile::|SdSpiConfig|pico/|p4/|Configuration.h", logger):
    failures.append("Shared logger contains target selection or SDK access")
application = (ROOT / "src/controller/application.cpp").read_text()
if "Arduino.h" in application or "Configuration.h" in application:
    failures.append("Application policy must compile without Arduino or board configuration")
main = (ROOT / "src/controller/main.cpp").read_text()
if "#if" in main or len(main.splitlines()) > 10:
    failures.append("Controller entry point must remain a thin, shared framework adapter")
if (ROOT / "src/controller_p4_main.cpp").exists() or (ROOT / "src/p4/controller_runtime.cpp").exists():
    failures.append("Separate P4 application implementation reintroduced")
updater = (ROOT / "src/controller/scanner_update.cpp").read_text()
if re.search(r"SHURIKEN_|Adafruit_|SdFat|freertos/|hardware/|Arduino.h", updater):
    failures.append("Shared scanner updater contains platform dependencies")
for header in (ROOT / "include/hal").glob("*.h"):
    if "pico_logging" in header.read_text():
        failures.append(f"{header}: chip-specific logging namespace")
if (ROOT / "src/hal/rp2350_scanner_update.cpp").exists():
    failures.append("Scanner updater must remain shared")
for source in (ROOT / "include/hal/log_storage.h", ROOT / "lib/controller_logging/controller_logging.h",
               ROOT / "lib/controller_logging/controller_logging.cpp"):
    if re.search(r"SdFat|FsFile|SdSpiConfig|DEDICATED_SPI|sd_spi_|sd_pin_cs|sd_retry_clock|fatDate|fatTime|SHURIKEN_", source.read_text()):
        failures.append(f"{source}: filesystem or storage-device mechanism in shared logging")
if "Arduino" in (ROOT / "include/hal/log_storage.h").read_text():
    failures.append("Storage contracts must compile without Arduino")
if re.search(r"\b(?:millis|micros|delay|time|settimeofday|gettimeofday|mktime|setenv|tzset)\s*\(", logger):
    failures.append("Shared logger must use injected time services and timezone-independent UTC conversion")
if failures:
    raise SystemExit("\n".join(failures))
print("PASS: application boundary has no board selection or hardware-specific includes")
