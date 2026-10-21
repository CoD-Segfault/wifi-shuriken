"""Run the real P4 loop against fake time, serial, BLE and SD backends."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="shuriken-p4-loop-") as directory:
    executable = Path(directory) / "loop-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
        "-I", str(HERE / "include"), str(HERE / "main.cpp"), "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True)
