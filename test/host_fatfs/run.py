"""Exercise the vendored FatFs against disposable, independently formatted images."""

import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
SDK = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
SDK = SDK / "packages/framework-arduinoespressif32-libs/esp32p4/qio_qspi/include"
SOURCE = ROOT / "third_party/fatfs/src"


def run(*args):
    subprocess.run([str(arg) for arg in args], check=True)


with tempfile.TemporaryDirectory(prefix="p4-fatfs-") as directory:
    temporary = Path(directory)
    executable = temporary / "fatfs-test"
    run("cc", "-std=c11", "-D_FILE_OFFSET_BITS=64", "-D_POSIX_C_SOURCE=200809L",
        "-g", "-fsanitize=address,undefined", "-I", HERE, "-I", SDK, "-I", SOURCE,
        HERE / "main.c", SOURCE / "ff.c", SOURCE / "ffunicode.c", "-o", executable)
    for name, megabytes, fs_type in (("FAT12", 4, 1), ("FAT16", 32, 2),
                                      ("FAT32", 128, 3), ("exFAT", 64, 4)):
        image = temporary / (name + ".img")
        with image.open("wb") as disk:
            disk.truncate(megabytes * 1024 * 1024)
        if name == "exFAT":
            run("mkfs.exfat", image)
        elif name == "FAT32":
            run("mkfs.fat", "-F", "32", "-s", "2", image)
        else:
            run("mkfs.fat", "-F", name[3:], image)
        print(name, flush=True)
        run(executable, image, fs_type)
        if name == "exFAT":
            run("fsck.exfat", "-n", image)
        else:
            run("fsck.fat", "-n", image)

    # Also cover the common partitioned SD-card layout, not just superfloppies.
    partitioned = temporary / "exfat-mbr.img"
    with partitioned.open("wb") as disk:
        mbr = bytearray(512)
        mbr[446:462] = struct.pack("<B3sB3sII", 0, b"\0" * 3, 0x07,
                                   b"\0" * 3, 2048, image.stat().st_size // 512)
        mbr[510:512] = b"\x55\xaa"
        disk.write(mbr)
        disk.seek(2048 * 512)
        with image.open("rb") as volume:
            shutil.copyfileobj(volume, disk)
    print("exFAT with MBR partition", flush=True)
    run(executable, partitioned, 4)
