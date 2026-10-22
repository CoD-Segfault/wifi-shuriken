"""Neutral storage and backend contracts; only disposable host files and SDK fakes."""
from pathlib import Path
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
with tempfile.TemporaryDirectory(prefix="shuriken-storage-build-") as directory:
    for name in ("posix", "rp"):
        executable = str(Path(directory) / name)
        includes = ["-I", str(HERE / "include")] if name == "rp" else []
        subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                        *includes, "-I", str(ROOT / "include"),
                        str(HERE / f"{name}_main.cpp"), "-o", executable], check=True)
        subprocess.run([executable], check=True)
