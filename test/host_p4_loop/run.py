"""Compile the hardware-independent controller application against fake ports."""
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="shuriken-application-") as directory:
    executable = Path(directory) / "application-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
        "-I", str(ROOT / "include"), str(HERE / "main.cpp"),
        str(ROOT / "src/controller/application.cpp"), "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True)
