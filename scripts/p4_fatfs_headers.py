import json
from pathlib import Path

Import("env")

root = Path(env.subst("$PROJECT_DIR")) / "third_party" / "fatfs"
platform_version = json.loads((Path(env.PioPlatform().get_dir()) / "platform.json").read_text())["version"]
framework = Path(env.PioPlatform().get_package_dir("framework-arduinoespressif32"))
framework_version = json.loads((framework / "package.json").read_text())["version"]
if platform_version != "55.03.312" or framework_version != "3.3.12":
    raise RuntimeError("P4 bring-up requires pioarduino 55.03.312 / Arduino 3.3.12")
sdk = Path(env.PioPlatform().get_package_dir("framework-arduinoespressif32-libs"))
sdk_headers = sdk / "esp32p4" / "include" / "fatfs"
version = json.loads((sdk / "package.json").read_text())["version"]
if version != "5.5.5+sha.b774170ff46":
    raise RuntimeError("Update the project-local FatFs sources for SDK " + version)

# Reject SDK updates that could silently mix incompatible FatFs layouts.
for relative in ("src/ff.h", "src/ffconf.h", "src/diskio.h", "vfs/esp_vfs_fat.h"):
    local = (root / relative).read_bytes()
    if relative == "src/ffconf.h":
        local = local.replace(b"#define FF_FS_EXFAT\t\t1", b"#define FF_FS_EXFAT\t\t0")
    if local != (sdk_headers / relative).read_bytes():
        raise RuntimeError("Update the project-local FatFs component for this SDK: " + relative)

# Set before the Arduino core and libraries clone the build environment.
env.Prepend(CPPPATH=[str(root / part) for part in ("src", "diskio", "vfs")])
