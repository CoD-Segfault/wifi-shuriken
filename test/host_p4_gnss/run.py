"""Exercise the P4 BLE NMEA parser and source policy using the real TinyGPS++."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
gps = next(iter((ROOT / ".pio/libdeps").glob("*/TinyGPSPlus/src/TinyGPS++.cpp")), None)
if gps is None:
    raise SystemExit("Build a controller environment first to install TinyGPSPlus.")
with tempfile.TemporaryDirectory(prefix="shuriken-p4-gnss-") as directory:
    executable = Path(directory) / "gnss-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-DARDUINO=10805",
        "-fsanitize=address,undefined", "-I", str(ROOT / "test/host_p4_logger/include"),
        "-I", str(ROOT / "src/p4"), "-I", str(gps.parent),
        str(HERE / "main.cpp"), str(gps), "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True)
