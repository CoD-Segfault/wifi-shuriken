"""HAL contracts on RP2350 (shift-register/direct CS), P4, and PC. No hardware."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
subprocess.run(["python3", str(ROOT / "test/check_application_boundary.py")], check=True)
with tempfile.TemporaryDirectory(prefix="shuriken-hal-") as directory:
    build = Path(directory)
    for name, defines in [
        ("rp2350-shift", ["-DSCANNER_USE_SHIFTREG_CS=1"]),
        ("rp2350-direct", ["-DSCANNER_USE_SHIFTREG_CS=0"]),
        ("p4", ["-DSCANNER_USE_SHIFTREG_CS=1", "-DSHURIKEN_P4_CONTROLLER"]),
        ("p4-usb", ["-DSCANNER_USE_SHIFTREG_CS=1", "-DSHURIKEN_P4_CONTROLLER", "-DP4_USB_NMEA=1", "-DP4_USB_MTP=1"]),
    ]:
        executable = build / name
        subprocess.run([
            "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", *defines,
            "-I", str(HERE / "include"), "-I", str(ROOT / "include"),
            str(HERE / "embedded_main.cpp"),
            str(ROOT / "src/arduino/scanner_transport.cpp"),
            str(ROOT / "src/arduino/services.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
    subprocess.run([
        "cmake", "-S", str(ROOT / "pc"), "-B", str(build / "pc"),
        "-DSHURIKEN_TIGARD=OFF", "-DSHURIKEN_SANITIZERS=ON",
        "-DSHURIKEN_SCANNER_PROFILE=ON",
    ], check=True)
    subprocess.run(["cmake", "--build", str(build / "pc"), "-j", "4"], check=True)
    subprocess.run(["ctest", "--test-dir", str(build / "pc"), "--output-on-failure"], check=True)
