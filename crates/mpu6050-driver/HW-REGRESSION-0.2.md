# ImuSample 0.2 evidence (host numerical + ESP32-C3 A/B)

Harness: `scripts/hw-acquisition-regression.sh`  
Port used: `/dev/cu.usbmodem11201` (ESP32-C3)  
Conditions: stationary board, path-patched local driver, **real USB serial** (no mocks).

## What each layer proves

| Evidence | Actually proves |
| --- | --- |
| Host `conversion_regression` | `i16 → f32` conversion error stays under **1/8 LSB** vs independent f64 scales |
| ESP32-C3 text A/B | Acquisition, data-ready, I²C, text transport did not regress after 0.2 consumer migrate |
| ESP32-C3 binary A/B | Decoded frame count continuity, sequence gaps, integrity/CRC-valid counts did not regress |
| Board `StampedSample` | Timestamp/sequence stay in the **board/transport** layer after removal from `ImuSample` |

### Interpretation (do not over-claim)

ESP32-C3 firmware still streams **`RawAccelGyroTemp` (`i16`)** on the wire. Text RAW and binary frames pack raw integers plus board stamps. They do **not** call `raw_to_imu_sample()` before transmit.

Therefore:

> ESP32-C3 acquisition and transport did not regress after migrating the board consumer to the physical-only `ImuSample` 0.2 API. Numerical `i16 → f32` conversion accuracy is covered separately by the full-domain host regression test.

Board A/B is **not** a direct on-device proof of `f64 → f32` arithmetic.

## Host numerical

```bash
cargo test -p mpu6050-driver --test conversion_regression
```

- Independent reference scales: `ACCEL_SCALE_REF = 16384.0`, `GYRO_SCALE_REF = 131.0` (not cast from production constants)
- Full `i16` domain + dedicated boundary list
- Tolerance: **1/8 LSB** of ±2 g / ±250 °/s

## Board A/B (same harness)

`measured_sample_rate_hz` uses **first successful sample → last successful sample** span  
`(count - 1) / Δt`, **not** harness wall-clock duration. That is why ~11918 samples in a 60 s wall-clock capture can still report ~201.8 Hz.

| Metric | main-f64 (harness @ `e21cd02`) | branch-f32 (`360f2fa`) |
| --- | ---: | ---: |
| Harness wall-clock capture (text) | 60 s | 60 s |
| successful_samples | 11918 | 11918 |
| measured_sample_rate_hz (first→last sample) | ~201.82 | ~201.83 |
| missed_or_coalesced_events | 0 | 0 |
| events_unrecorded_due_to_pending_saturation | 0 | 0 |
| total_i2c_errors | 0 | 0 |
| max_pending | 1 | 1 |
| acquisition_started | true | true |
| panic / watchdog tokens | 0 | 0 |
| First 8 diagnostic RAW \|a\| mean (host i16→g) | 0.9773 g | 0.9772 g |
| Harness wall-clock capture (binary) | 30 s | 30 s |
| Binary decoded frames | 6002 | 6010 |
| Binary CRC-valid / clean frames | 6002 | 6010 |
| Binary sequence gaps | 0 | 0 |
| Binary host i16→g \|a\| mean | 0.9783 g | 0.9782 g |
| Binary \|a\| in [0.8, 1.2] | 100% | 100% |
| Harness verdict | PASS | PASS |

Notes:

- Binary frame count 6002 vs 6010 across separate captures is normal (start/stop alignment); rate, gaps, and integrity matter more.
- “First 8 diagnostic RAW” is **supplemental smoke**, not primary magnitude evidence (binary ~6k frames is).
- Host-side \|a\| from log `i16` values is a physical sanity check, not on-device `f32` conversion.

### Local metric files (gitignored under `logs/`)

- `logs/hw-regression/main-f64-text-20260720-222644.metrics.txt`
- `logs/hw-regression/main-f64-binary-20260720-222857.metrics.txt`
- `logs/hw-regression/branch-f32-text-20260720-223209.metrics.txt`
- `logs/hw-regression/branch-f32-binary-20260720-223314.metrics.txt`

## Re-run

```bash
# After commit 1 (f64 API still): baseline
LABEL=main-f64 DURATION=60 MODE=text ./scripts/hw-acquisition-regression.sh
LABEL=main-f64 DURATION=30 MODE=binary ./scripts/hw-acquisition-regression.sh

# After breaking + board wrap (f32 API): same harness
LABEL=branch-f32 DURATION=60 MODE=text ./scripts/hw-acquisition-regression.sh
LABEL=branch-f32 DURATION=30 MODE=binary ./scripts/hw-acquisition-regression.sh

cargo test -p mpu6050-driver --test conversion_regression
```

## Commit story on this branch

```text
e21cd02  test: harness + conversion contract (pre-break)
839fa43  breaking: physical-only ImuSample + f32
360f2fa  refactor: board StampedSample / transport stamps
ee810fb  test: A/B evidence doc
(+ follow-up) clarify interpretation + independent f64 refs
```
