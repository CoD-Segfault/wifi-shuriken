"""Run the real logger/TinyGPS parser and P4 VFS backend against disposable host files."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
gps = next(iter((ROOT / ".pio/libdeps").glob("*/TinyGPSPlus/src/TinyGPS++.cpp")), None)
if gps is None:
    raise SystemExit("Build a controller environment first to install TinyGPSPlus.")
for require_time in (0, 1):
  with tempfile.TemporaryDirectory(prefix="shuriken-p4-logger-") as directory:
    executable = Path(directory) / "logger-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-DSHURIKEN_P4_CONTROLLER", "-DARDUINO=10805",
        f"-DP4_REQUIRE_GNSS_TIME_FOR_LOGGING={require_time}",
        "-fsanitize=address,undefined", "-I", str(HERE / "include"), "-I", str(ROOT / "include"),
        "-I", str(ROOT / "lib/controller_logging"), "-I", str(gps.parent), str(HERE / "main.cpp"),
        str(ROOT / "lib/controller_logging/controller_logging.cpp"), str(ROOT / "src/controller/time.cpp"), str(ROOT / "src/arduino/logging.cpp"), str(ROOT / "lib/log_format/log_format.cpp"),
        str(gps), "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable), directory], check=True)
