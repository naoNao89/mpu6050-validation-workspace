#!/usr/bin/env bash
# Build, flash, and monitor the ESP32-C3 MPU6050 bring-up firmware.
#
# Examples:
#   ./run.sh
#   PORT=/dev/cu.usbmodem11301 ./run.sh
#   NO_FLASH=1 ./run.sh
#   NO_MONITOR=1 ./run.sh
#   NO_LOG=1 ./run.sh
#   LOG_FILE=logs/mpu6050.log ./run.sh
#   DURATION=300 ./run.sh
#   MODE=binary DURATION=30 LOG_FILE=logs/motion-binary.log ./run.sh

set -euo pipefail

if [ -f "$HOME/export-esp.sh" ]; then
  # shellcheck disable=SC1091
  . "$HOME/export-esp.sh"
fi

PORT="${PORT:-}"
BAUD="${BAUD:-115200}"
MODE="${MODE:-text}"
TARGET="${TARGET:-riscv32imc-unknown-none-elf}"
LOG_DIR="${LOG_DIR:-logs}"
LOG_FILE="${LOG_FILE:-}"
DURATION="${DURATION:-}"
BIN="target/${TARGET}/release/mpu6050-esp32c3-bringup"

if [ -z "$PORT" ]; then
  PORT="$(./scripts/esp-port.sh --print)"
fi

if [ "${NO_LOG:-0}" != "1" ] && [ -z "$LOG_FILE" ]; then
  LOG_FILE="$LOG_DIR/mpu6050-$(date +%Y%m%d-%H%M%S).log"
fi

build_args=(--manifest-path boards/esp32-c3/Cargo.toml --release --target "$TARGET")
if [ "$MODE" = "binary" ]; then
  build_args+=(--features binary-frames)
fi

echo "--- build release firmware for ${TARGET} (mode=${MODE}) ---"
env -u RUSTFLAGS CARGO_TARGET_DIR=target cargo --config 'patch.crates-io.mpu6050-driver.path="crates/mpu6050-driver"' build "${build_args[@]}"

if [ "${NO_FLASH:-0}" != "1" ]; then
  echo "--- flash ${PORT} ---"
  env -u RUSTFLAGS espflash flash --port "$PORT" "$BIN"
fi

if [ "${NO_MONITOR:-0}" = "1" ]; then
  exit 0
fi

if [ "${NO_LOG:-0}" = "1" ]; then
  monitor_args=(--port "$PORT" --baud "$BAUD" --mode "$MODE")
else
  mkdir -p "$(dirname "$LOG_FILE")"
  monitor_args=(--port "$PORT" --baud "$BAUD" --out "$LOG_FILE" --mode "$MODE")
fi

if [ -n "$DURATION" ]; then
  monitor_args+=(--duration "$DURATION")
fi

cargo run -p imu-tool -- monitor "${monitor_args[@]}"
