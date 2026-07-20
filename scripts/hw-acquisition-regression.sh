#!/usr/bin/env bash
# ESP32-C3 USB acquisition regression harness (real serial only).
#
# Produces a metrics summary for A/B comparison across API representation changes.
# Does not mock samples. Requires a connected board.
#
# Usage:
#   LABEL=main-f64 DURATION=60 MODE=text ./scripts/hw-acquisition-regression.sh
#   LABEL=branch-f32 DURATION=60 MODE=binary ./scripts/hw-acquisition-regression.sh
#
# Env:
#   PORT       serial port (default: scripts/esp-port.sh)
#   DURATION   monitor seconds (default: 60)
#   MODE       text|binary (default: text)
#   LABEL      run label for log/metrics names (default: run)
#   LOG_DIR    output directory (default: logs/hw-regression)
#   NO_FLASH   1 to skip flash (default: 0)

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [ -f "$HOME/export-esp.sh" ]; then
  # shellcheck disable=SC1091
  . "$HOME/export-esp.sh"
fi

PORT="${PORT:-}"
DURATION="${DURATION:-60}"
MODE="${MODE:-text}"
LABEL="${LABEL:-run}"
LOG_DIR="${LOG_DIR:-logs/hw-regression}"
NO_FLASH="${NO_FLASH:-0}"
BAUD="${BAUD:-115200}"
TARGET="${TARGET:-riscv32imc-unknown-none-elf}"
BIN="target/${TARGET}/release/mpu6050-esp32c3-bringup"

if [ -z "$PORT" ]; then
  PORT="$(./scripts/esp-port.sh --print)"
fi

stamp="$(date +%Y%m%d-%H%M%S)"
safe_label="$(printf '%s' "$LABEL" | tr -c 'A-Za-z0-9._-' '-')"
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_DIR}/${safe_label}-${MODE}-${stamp}.log"
METRICS_FILE="${LOG_DIR}/${safe_label}-${MODE}-${stamp}.metrics.txt"
COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

echo "hw_acquisition_regression label=${LABEL} commit=${COMMIT} port=${PORT} mode=${MODE} duration=${DURATION}s"
echo "log=${LOG_FILE}"

build_args=(--manifest-path boards/esp32-c3/Cargo.toml --release --target "$TARGET")
if [ "$MODE" = "binary" ]; then
  build_args+=(--features binary-frames)
fi

echo "--- build ---"
env -u RUSTFLAGS CARGO_TARGET_DIR=target \
  cargo --config 'patch.crates-io.mpu6050-driver.path="crates/mpu6050-driver"' \
  build "${build_args[@]}"

if [ "$NO_FLASH" != "1" ]; then
  echo "--- flash ${PORT} ---"
  env -u RUSTFLAGS espflash flash --port "$PORT" "$BIN"
fi

echo "--- monitor ${DURATION}s mode=${MODE} ---"
cargo run -q -p imu-tool -- monitor \
  --port "$PORT" \
  --baud "$BAUD" \
  --out "$LOG_FILE" \
  --mode "$MODE" \
  --duration "$DURATION"

python3 - "$LOG_FILE" "$METRICS_FILE" "$LABEL" "$COMMIT" "$MODE" "$DURATION" "$PORT" <<'PY'
import re, sys, math
from pathlib import Path

log_path, metrics_path, label, commit, mode, duration, port = sys.argv[1:8]
text = Path(log_path).read_text(errors="replace")
lines = text.splitlines()

def find_one(pat: str):
    m = re.search(pat, text, re.M)
    return m.group(0) if m else None

def last_summary_field(key: str):
    last = None
    for line in lines:
        if not line.startswith("acquisition_summary"):
            continue
        m = re.search(rf"{re.escape(key)}=(\S+)", line)
        if m:
            last = m.group(1)
    return last

def count(pat: str) -> int:
    return sum(1 for line in lines if re.search(pat, line))

startup = find_one(r"data_ready_startup .*")
blocked = count(r"data_ready_acquisition_blocked")
panics = count(r"(?i)\bpanic\b")
# sequence continuity for binary/text RAW lines that carry sequence=
seqs = []
for line in lines:
    m = re.search(r"\bsequence=(\d+)\b", line)
    if m and line.startswith("RAW"):
        seqs.append(int(m.group(1)))
gaps = 0
if seqs:
    for a, b in zip(seqs, seqs[1:]):
        if b != a + 1 and not (a == 0 and b == 0):
            # allow wrap only if wrapping_add; treat non-monotonic as gap
            if b != (a + 1) % (1 << 64):
                gaps += 1
    # simpler: consecutive +1
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if b != a + 1)

raw_lines = [l for l in lines if l.startswith("RAW")]
# parse accel for magnitude when raw i16 present
mags = []
pat_raw = re.compile(
    r"accel=\((-?\d+),\s*(-?\d+),\s*(-?\d+)\)"
)
for line in raw_lines:
    m = pat_raw.search(line)
    if not m:
        continue
    ax, ay, az = map(int, m.groups())
    g = [v / 16384.0 for v in (ax, ay, az)]
    mags.append(math.sqrt(sum(x * x for x in g)))

finite_ok = all(math.isfinite(m) for m in mags) if mags else True
inband = sum(1 for m in mags if 0.80 <= m <= 1.20)
mag_mean = (sum(mags) / len(mags)) if mags else float("nan")

# integrity_stats if present
integrity = find_one(r"integrity_stats .*")
clean = None
if integrity:
    m = re.search(r"clean_samples=(\d+)", integrity)
    if m:
        clean = int(m.group(1))

sample_rate = last_summary_field("measured_sample_rate_hz")
successful = last_summary_field("successful_samples")
missed = last_summary_field("missed_or_coalesced_events")
i2c_err = last_summary_field("total_i2c_errors")
max_pending = last_summary_field("max_pending")
isr_total = last_summary_field("isr_data_ready_total")
pending_sat = last_summary_field("events_unrecorded_due_to_pending_saturation")
first_sample_us = last_summary_field("first_sample_us")
last_sample_us = last_summary_field("last_sample_us")
watchdog = count(r"(?i)watchdog")
# Binary integrity_stats: clean_samples ≈ CRC-valid decoded frames from imu-tool.
suspicious = None
if integrity:
    m = re.search(r"suspicious_events=(\d+)", integrity)
    if m:
        suspicious = int(m.group(1))
    m = re.search(r"samples=(\d+)", integrity)
    integrity_samples = int(m.group(1)) if m else None
else:
    integrity_samples = None

# Pass gates (hardware regression contract). Binary firmware omits text startup
# / acquisition_summary lines; those gates apply to text mode only.
#
# IMPORTANT: board A/B proves acquisition/transport regression safety after the
# 0.2 consumer migration. It does NOT prove i16→f32 conversion on-device (firmware
# streams raw i16). Numerical f32 accuracy is host conversion_regression only.
checks = []
def check(name, ok, detail=""):
    checks.append((name, bool(ok), detail))

is_binary = mode == "binary"
check("no_acquisition_blocked", blocked == 0, f"blocked_lines={blocked}")
check("no_panic_token", panics == 0, f"panic_lines={panics}")
check("no_watchdog_token", watchdog == 0, f"watchdog_lines={watchdog}")
check(
    "has_motion_evidence",
    bool(raw_lines) or (successful not in (None, "0")),
    f"raw={len(raw_lines)} successful={successful}",
)

if not is_binary:
    check("data_ready_startup_present", startup is not None, startup or "missing")
    if startup:
        check("acquisition_started_true", "acquisition_started=true" in startup, startup)
        check("timing_confirmed_true", "timing_confirmed=true" in startup, startup)
        check(
            "exact_data_ready_readback_true",
            "exact_data_ready_readback=true" in startup,
            startup,
        )
    if successful is not None and successful.isdigit():
        check("successful_samples_gt_0", int(successful) > 0, successful)
    if missed is not None:
        check("missed_or_coalesced_zero", missed in ("0", "Some(0)"), missed)
    if pending_sat is not None:
        check(
            "pending_saturation_zero",
            pending_sat in ("0", "Some(0)"),
            pending_sat,
        )
    if i2c_err is not None:
        check("total_i2c_errors_zero", i2c_err in ("0", "Some(0)"), i2c_err)
    if max_pending is not None and max_pending.isdigit():
        check("max_pending_le_2", int(max_pending) <= 2, max_pending)
    if sample_rate:
        m = re.search(r"([0-9]+(?:\.[0-9]+)?)", sample_rate)
        if m:
            rate = float(m.group(1))
            # measured_sample_rate_hz uses first→last sample span, not wall-clock.
            check("sample_rate_190_210", 190.0 <= rate <= 210.0, sample_rate)
else:
    # Binary stream contract: continuous frames + integrity + physical plausibility.
    decoded = len(raw_lines)
    check("binary_decoded_frames_gt_100", decoded > 100, f"decoded_frames={decoded}")
    check("binary_sequence_present", len(seqs) > 100, f"seqs={len(seqs)}")
    if clean is not None:
        check(
            "binary_crc_valid_matches_decoded",
            clean == decoded or clean == len(seqs),
            f"crc_valid_frames={clean} decoded_frames={decoded} seqs={len(seqs)}",
        )
    if suspicious is not None:
        check("binary_suspicious_events_zero", suspicious == 0, f"suspicious={suspicious}")
    if integrity_samples is not None:
        check(
            "binary_integrity_samples_match_decoded",
            integrity_samples == decoded,
            f"integrity_samples={integrity_samples} decoded={decoded}",
        )

if seqs:
    check(
        "sequence_gaps_zero",
        gaps == 0,
        f"gaps={gaps} n={len(seqs)} first={seqs[0]} last={seqs[-1]}",
    )
if mags:
    check("accel_values_finite", finite_ok, f"n={len(mags)}")
    # Host-side |a| from raw i16 in the log (not on-device f32 conversion).
    check(
        "stationary_accel_mag_inband_ge_95pct",
        (inband / len(mags)) >= 0.95,
        f"inband={inband}/{len(mags)} mean={mag_mean:.4f} note=host_i16_to_g",
    )

failed = [c for c in checks if not c[1]]
verdict = "PASS" if not failed else "FAIL"

out = []
out.append(f"verdict={verdict}")
out.append(f"label={label}")
out.append(f"commit={commit}")
out.append(f"port={port}")
out.append(f"mode={mode}")
out.append(f"harness_wall_clock_capture_s={duration}")
out.append(
    "note=measured_sample_rate_hz uses first_successful_sample→last_successful_sample span, not wall-clock"
)
out.append(
    "note=board_ab_proves_acquisition_transport_not_on_device_f32_conversion"
)
out.append(f"log={log_path}")
out.append(f"data_ready_startup={startup or 'missing'}")
out.append(f"measured_sample_rate_hz={sample_rate}")
out.append(f"measured_rate_span_first_sample_us={first_sample_us}")
out.append(f"measured_rate_span_last_sample_us={last_sample_us}")
out.append(f"successful_samples={successful}")
out.append(f"isr_data_ready_total={isr_total}")
out.append(f"missed_or_coalesced_events={missed}")
out.append(f"events_unrecorded_due_to_pending_saturation={pending_sat}")
out.append(f"total_i2c_errors={i2c_err}")
out.append(f"max_pending={max_pending}")
out.append(f"decoded_frames_or_raw_lines={len(raw_lines)}")
out.append(f"binary_crc_valid_frames={clean}")
out.append(f"binary_suspicious_events={suspicious}")
out.append(f"sequence_samples={len(seqs)}")
out.append(f"sequence_gaps={gaps}")
out.append(f"first_diagnostic_raw_count={min(8, len(raw_lines)) if not is_binary else 'n/a'}")
out.append(
    f"host_i16_accel_mag_mean_g={mag_mean if mags else 'n/a'}"
)
out.append(
    f"host_i16_accel_mag_inband_0.8_1.2={inband if mags else 0}/{len(mags) if mags else 0}"
)
out.append(f"integrity={integrity or 'missing'}")
out.append(f"panic_lines={panics}")
out.append(f"watchdog_lines={watchdog}")
out.append("checks_begin")
for name, ok, detail in checks:
    out.append(f"check {name}={'PASS' if ok else 'FAIL'} detail={detail}")
out.append("checks_end")
Path(metrics_path).write_text("\n".join(out) + "\n")
print("\n".join(out))
if failed:
    sys.exit(1)
PY

echo "metrics=${METRICS_FILE}"
