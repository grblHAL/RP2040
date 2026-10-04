#!/usr/bin/env bash
#
# Build script for the grblHAL RP2040 driver (BTT SKR Pico 1.0 / RP2040 boards).
#
# Usage:
#   ./build.sh [options]
#
# Options:
#   -b, --board BOARD       PICO_BOARD value (default: pico)
#                           Common values: pico, pico_w, pico2, pico2_w, pimoroni_pga2350
#   -j, --jobs N            Parallel build jobs (default: nproc)
#   -c, --clean             Remove the build directory before configuring
#       --sdk-path PATH     Override PICO_SDK_PATH (default: $PICO_SDK_PATH env or /mnt/projs/pico/pico-sdk)
#       --wifi              Enable ADD_WIFI
#       --ethernet          Enable ADD_ETHERNET
#       --bluetooth         Enable ADD_BLUETOOTH
#       --mdns              Enable ADD_mDNS
#       --mqtt              Enable ADD_MQTT
#       --hpgl              Enable ADD_HPGL
#       --my-plugin         Enable AddMyPlugin (build my_plugin.c)
#   -h, --help              Show this help
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

BOARD="pico"
JOBS="$(nproc 2>/dev/null || echo 4)"
CLEAN=0
SDK_PATH="${PICO_SDK_PATH:-/mnt/projs/pico/pico-sdk}"

ADD_WIFI=OFF
ADD_ETHERNET=OFF
ADD_BLUETOOTH=OFF
ADD_MDNS=OFF
ADD_MQTT=OFF
ADD_HPGL=OFF
ADD_MY_PLUGIN=OFF

usage() {
    grep '^#' "${BASH_SOURCE[0]}" | sed -n '2,/^set -e/p' | sed '$d' | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        -b|--board) BOARD="$2"; shift 2 ;;
        -j|--jobs) JOBS="$2"; shift 2 ;;
        -c|--clean) CLEAN=1; shift ;;
        --sdk-path) SDK_PATH="$2"; shift 2 ;;
        --wifi) ADD_WIFI=ON; shift ;;
        --ethernet) ADD_ETHERNET=ON; shift ;;
        --bluetooth) ADD_BLUETOOTH=ON; shift ;;
        --mdns) ADD_MDNS=ON; shift ;;
        --mqtt) ADD_MQTT=ON; shift ;;
        --hpgl) ADD_HPGL=ON; shift ;;
        --my-plugin) ADD_MY_PLUGIN=ON; shift ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown option: $1" >&2; usage 1 ;;
    esac
done

if [ "$ADD_WIFI" = ON ] && [ "$ADD_ETHERNET" = ON ]; then
    echo "Error: --wifi and --ethernet cannot both be enabled." >&2
    exit 1
fi

if [ ! -d "$SDK_PATH" ]; then
    echo "Error: PICO_SDK_PATH not found at '$SDK_PATH'." >&2
    echo "Pass --sdk-path or set the PICO_SDK_PATH environment variable." >&2
    exit 1
fi
export PICO_SDK_PATH="$SDK_PATH"

for tool in cmake arm-none-eabi-gcc; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "Error: required tool '$tool' not found in PATH." >&2
        exit 1
    fi
done

if command -v ninja >/dev/null 2>&1; then
    GENERATOR="Ninja"
    BUILD_TOOL="ninja"
    BUILD_TOOL_ARGS=(-C "$BUILD_DIR" -j "$JOBS")
else
    GENERATOR="Unix Makefiles"
    BUILD_TOOL="make"
    BUILD_TOOL_ARGS=(-C "$BUILD_DIR" -j "$JOBS")
fi

if [ "$CLEAN" -eq 1 ]; then
    echo "Cleaning build directory: $BUILD_DIR"
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"

echo "== Configuring =="
echo "  PICO_SDK_PATH : $SDK_PATH"
echo "  PICO_BOARD    : $BOARD"
echo "  Generator     : $GENERATOR"
echo "  ADD_WIFI      : $ADD_WIFI"
echo "  ADD_ETHERNET  : $ADD_ETHERNET"
echo "  ADD_BLUETOOTH : $ADD_BLUETOOTH"
echo "  ADD_mDNS      : $ADD_MDNS"
echo "  ADD_MQTT      : $ADD_MQTT"
echo "  ADD_HPGL      : $ADD_HPGL"
echo "  AddMyPlugin   : $ADD_MY_PLUGIN"

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -G "$GENERATOR" \
    -DPICO_BOARD="$BOARD" \
    -DADD_WIFI="$ADD_WIFI" \
    -DADD_ETHERNET="$ADD_ETHERNET" \
    -DADD_BLUETOOTH="$ADD_BLUETOOTH" \
    -DADD_mDNS="$ADD_MDNS" \
    -DADD_MQTT="$ADD_MQTT" \
    -DADD_HPGL="$ADD_HPGL" \
    -DAddMyPlugin="$ADD_MY_PLUGIN"

ACTUAL_SDK_PATH="$(grep -m1 '^PICO_SDK_PATH:' "${BUILD_DIR}/CMakeCache.txt" 2>/dev/null | cut -d= -f2- || true)"
if [ -n "$ACTUAL_SDK_PATH" ] && [ "$ACTUAL_SDK_PATH" != "$SDK_PATH" ]; then
    echo
    echo "Note: CMakeLists.txt's VS Code header pins the SDK to '$ACTUAL_SDK_PATH'"
    echo "      (it takes priority over --sdk-path/PICO_SDK_PATH whenever that path exists)."
fi

echo "== Building =="
"$BUILD_TOOL" "${BUILD_TOOL_ARGS[@]}"

UF2_FILE="${BUILD_DIR}/grblHAL.uf2"
if [ -f "$UF2_FILE" ]; then
    echo
    echo "Build complete: $UF2_FILE"
    echo "Copy this file to the RP2040 in BOOTSEL/mass-storage mode to flash it."
else
    echo
    echo "Build finished but $UF2_FILE was not found; check the log above." >&2
    exit 1
fi
