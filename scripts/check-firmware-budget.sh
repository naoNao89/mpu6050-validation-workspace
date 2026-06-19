#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <firmware-elf>" >&2
  exit 2
fi

ELF="$1"
FIRMWARE_BUDGET_PROFILE="${FIRMWARE_BUDGET_PROFILE:-esp32-c3-ci}"
TARGET_TRIPLE="${TARGET_TRIPLE:-riscv32imc-unknown-none-elf}"

case "$FIRMWARE_BUDGET_PROFILE" in
  esp32-c3-ci)
    PROFILE_ARCH="riscv32"
    PROFILE_FLASH_BYTES=524288
    PROFILE_STATIC_RAM_BYTES=81920
    PROFILE_NOTE="Conservative ESP32-C3 CI size budget only."
    ;;
  esp32-c3)
    PROFILE_ARCH="riscv32"
    PROFILE_FLASH_BYTES=2097152
    PROFILE_STATIC_RAM_BYTES=409600
    PROFILE_NOTE="ESP32-C3 size profile: 400KB on-chip SRAM; flash varies by chip/module."
    ;;
  esp32-c6)
    PROFILE_ARCH="riscv32"
    PROFILE_FLASH_BYTES=4194304
    PROFILE_STATIC_RAM_BYTES=524288
    PROFILE_NOTE="ESP32-C6 size profile: 512KB on-chip SRAM; flash varies by package/module."
    ;;
  esp32-h2)
    PROFILE_ARCH="riscv32"
    PROFILE_FLASH_BYTES=4194304
    PROFILE_STATIC_RAM_BYTES=327680
    PROFILE_NOTE="ESP32-H2 size profile: 320KB on-chip SRAM; module flash is commonly 4MB."
    ;;
  esp32-p4)
    PROFILE_ARCH="riscv32"
    PROFILE_FLASH_BYTES=8388608
    PROFILE_STATIC_RAM_BYTES=524288
    PROFILE_NOTE="ESP32-P4 coarse size profile: internal RAM is variant-dependent; external flash/PSRAM board design matters."
    ;;
  esp32-s3)
    PROFILE_ARCH="xtensa-lx7"
    PROFILE_FLASH_BYTES=4194304
    PROFILE_STATIC_RAM_BYTES=524288
    PROFILE_NOTE="ESP32-S3 size profile: 512KB on-chip SRAM; Xtensa target, not compatible with this RISC-V build."
    ;;
  *)
    echo "error: unknown FIRMWARE_BUDGET_PROFILE: ${FIRMWARE_BUDGET_PROFILE}" >&2
    echo "supported profiles: esp32-c3-ci esp32-c3 esp32-c6 esp32-h2 esp32-p4 esp32-s3" >&2
    exit 2
    ;;
esac

MAX_FLASH_BYTES="${MAX_FLASH_BYTES:-$PROFILE_FLASH_BYTES}"
MAX_STATIC_RAM_BYTES="${MAX_STATIC_RAM_BYTES:-$PROFILE_STATIC_RAM_BYTES}"

if [[ ! -f "$ELF" ]]; then
  echo "error: firmware ELF not found: $ELF" >&2
  exit 2
fi

SIZE_OUTPUT="$(rust-size -A "$ELF")"
printf '%s\n' "$SIZE_OUTPUT"

section_size() {
  local section="$1"
  awk -v section="$section" '$1 == section { print $2; found = 1 } END { if (!found) print 0 }' <<<"$SIZE_OUTPUT"
}

TEXT_BYTES="$(section_size .text)"
RODATA_BYTES="$(section_size .rodata)"
DATA_BYTES="$(section_size .data)"
BSS_BYTES="$(section_size .bss)"

FLASH_ESTIMATE_BYTES=$((TEXT_BYTES + RODATA_BYTES + DATA_BYTES))
STATIC_RAM_ESTIMATE_BYTES=$((DATA_BYTES + BSS_BYTES))
FLASH_HEADROOM_BYTES=$((MAX_FLASH_BYTES - FLASH_ESTIMATE_BYTES))
STATIC_RAM_HEADROOM_BYTES=$((MAX_STATIC_RAM_BYTES - STATIC_RAM_ESTIMATE_BYTES))
if (( MAX_FLASH_BYTES > 0 )); then
  FLASH_PERCENT=$((FLASH_ESTIMATE_BYTES * 100 / MAX_FLASH_BYTES))
else
  FLASH_PERCENT=0
fi
if (( MAX_STATIC_RAM_BYTES > 0 )); then
  STATIC_RAM_PERCENT=$((STATIC_RAM_ESTIMATE_BYTES * 100 / MAX_STATIC_RAM_BYTES))
else
  STATIC_RAM_PERCENT=0
fi

echo
echo "firmware size budget report"
echo "target_profile=${FIRMWARE_BUDGET_PROFILE}"
echo "profile_arch=${PROFILE_ARCH}"
echo "target_triple=${TARGET_TRIPLE}"
echo "flash_estimate_bytes=${FLASH_ESTIMATE_BYTES} / ${MAX_FLASH_BYTES} (${FLASH_PERCENT}%)"
echo "static_ram_estimate_bytes=${STATIC_RAM_ESTIMATE_BYTES} / ${MAX_STATIC_RAM_BYTES} (${STATIC_RAM_PERCENT}%)"
echo "flash_headroom_bytes=${FLASH_HEADROOM_BYTES}"
echo "static_ram_headroom_bytes=${STATIC_RAM_HEADROOM_BYTES}"
echo "available_profiles=esp32-c3-ci esp32-c3 esp32-c6 esp32-h2 esp32-p4 esp32-s3"
echo "profile_note=${PROFILE_NOTE}"
echo "compatibility_note=Size budget only; this does not imply runtime or chip compatibility."

failed=0
if (( FLASH_ESTIMATE_BYTES > MAX_FLASH_BYTES )); then
  echo "error: firmware flash estimate ${FLASH_ESTIMATE_BYTES} bytes exceeds budget ${MAX_FLASH_BYTES} bytes" >&2
  failed=1
fi

if (( STATIC_RAM_ESTIMATE_BYTES > MAX_STATIC_RAM_BYTES )); then
  echo "error: firmware static RAM estimate ${STATIC_RAM_ESTIMATE_BYTES} bytes exceeds budget ${MAX_STATIC_RAM_BYTES} bytes" >&2
  failed=1
fi

if (( failed == 0 )); then
  echo "size_fit=within configured ${FIRMWARE_BUDGET_PROFILE} size budget"
else
  echo "size_fit=exceeds configured ${FIRMWARE_BUDGET_PROFILE} size budget"
fi

exit "$failed"
