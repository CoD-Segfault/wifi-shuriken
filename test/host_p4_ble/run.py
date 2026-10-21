"""Exercise the actual BLE UART module using deterministic single-threaded API fakes."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="shuriken-p4-ble-") as directory:
    executable = Path(directory) / "ble-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
        "-I", str(HERE / "include"), "-I", str(ROOT / "src/p4"),
        str(HERE / "main.cpp"), str(ROOT / "src/p4/ble_uart.cpp"), "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True)
    subprocess.run([str(executable), "startup-failure"], check=True)
