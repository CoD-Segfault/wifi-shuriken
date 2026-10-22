"""Exercise actual P4 boot SPI RPC with threaded FreeRTOS fakes; no hardware."""
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="shuriken-p4-boot-spi-") as directory:
    executable = Path(directory) / "boot-spi-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
        "-fsanitize=address,undefined", "-I", str(HERE / "include"),
        "-I", str(ROOT / "include"), "-I", str(ROOT / "src/p4"),
        str(HERE / "main.cpp"), str(ROOT / "src/p4/scanner_update_transport.cpp"),
        "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True)
