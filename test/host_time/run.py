"""Time contracts use fake system calls; never adjust the real host clock."""
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="shuriken-time-") as directory:
    executable = str(Path(directory) / "time-test")
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", "-I", str(ROOT / "include"),
                    str(HERE / "main.cpp"), str(ROOT / "src/controller/time.cpp"),
                    "-o", executable], check=True)
    subprocess.run([executable], check=True)
