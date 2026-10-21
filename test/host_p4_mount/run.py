"""Exercise the actual P4 linker wrapper with a minimal host API fixture."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="shuriken-p4-mount-") as directory:
    executable = Path(directory) / "mount-test"
    subprocess.run([
        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
        "-I", str(HERE / "include"), str(HERE / "main.cpp"),
        str(ROOT / "src/p4/sdmmc_mount_fix.cpp"), "-o", str(executable)
    ], check=True)
    subprocess.run([str(executable)], check=True)
