#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
framework="${ARDUINO_PICO_DIR:-$HOME/.platformio/packages/framework-arduinopico}"
tinyusb="$framework/libraries/Adafruit_TinyUSB_Arduino/src"
build="$(mktemp -d /tmp/shuriken-mtp-tests.XXXXXX)"
trap 'rm -rf "$build"' EXIT
flags=(-g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer '-DCFG_TUSB_CONFIG_FILE="test/host_mtp/include/tusb_config.h"' -I. -Itest/host_mtp/include -Iinclude -I"$tinyusb")
cc -std=c11 "${flags[@]}" -c src/usb/mtp_device.c -o "$build/transport.o"
c++ -std=c++17 "${flags[@]}" test/host_mtp/mtp_test.cpp "$build/transport.o" -o "$build/mtp_test"
ASAN_OPTIONS=detect_leaks=0 "$build/mtp_test"
