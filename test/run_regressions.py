"""Run all controller host regressions; never upload or access physical hardware."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--firmware", action="store_true", help="build both controllers before testing")
parser.add_argument("--host-only", action="store_true", help="omit PlatformIO native test matrix")
parser.add_argument("--pio", default=shutil.which("pio") or str(Path.home() / ".platformio/penv/bin/pio"))
args = parser.parse_args()
environment = os.environ.copy()
environment.setdefault("ASAN_OPTIONS", "detect_leaks=0")
checks = []
if args.firmware:
    checks.append(("controller firmware", [args.pio, "run", "-e", "waveshare_p4_controller", "-e", "waveshare_p4_usb_controller", "-e", "xiao_rp2350_controller", "-j", "4"]))
for suite in ("host_hal", "host_time", "host_storage", "host_p4_mount", "host_p4_update_transport", "host_p4_ble", "host_p4_gnss", "host_p4_logger", "host_p4_loop", "host_fatfs"):
    checks.append((suite, [sys.executable, str(ROOT / "test" / suite / "run.py")]))
if not args.host_only:
    checks.append(("native protocol/scheduler/CSV/dedupe matrix", [args.pio, "test", "-e", "native_tests", "-e", "native_tests_hash32", "-e", "native_tests_hash128"]))
failed = []
for name, command in checks:
    print(f"\nRunning {name}", flush=True)
    try:
        result = subprocess.run(command, cwd=ROOT, env=environment)
        if result.returncode: failed.append(name)
    except OSError as error:
        print(error, file=sys.stderr); failed.append(name)
print(f"\n{len(checks) - len(failed)}/{len(checks)} regression groups passed", flush=True)
if failed:
    print("FAILED: " + ", ".join(failed), file=sys.stderr)
    sys.exit(1)
