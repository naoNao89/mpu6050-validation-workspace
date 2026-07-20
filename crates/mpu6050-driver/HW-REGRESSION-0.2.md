# ImuSample 0.2 hardware A/B regression

Same harness: `scripts/hw-acquisition-regression.sh`  
Port: `/dev/cu.usbmodem11201` (ESP32-C3)  
Conditions: stationary board, path-patched local driver, real USB serial (no mocks).

## Host numerical (always)

`cargo test -p mpu6050-driver --test conversion_regression`

- Full `i16` domain accel/gyro vs independent **f64** reference
- Tolerance: **1/8 LSB** of ±2 g / ±250 °/s scales
- Boundary set: `i16::MIN`, `-16384`, `-1`, `0`, `1`, `16384`, `i16::MAX`

## Board A/B (ESP32-C3 USB)

| Metric | main-f64 (`e21cd02` harness) | branch-f32 (`360f2fa`) |
| --- | ---: | ---: |
| Text duration | 60 s | 60 s |
| Text successful_samples | 11918 | 11918 |
| Text measured_sample_rate_hz | ~201.82 | ~201.83 |
| Text missed_or_coalesced | 0 | 0 |
| Text total_i2c_errors | 0 | 0 |
| Text max_pending | 1 | 1 |
| Text acquisition_started | true | true |
| Text \|a\| mean (8 RAW) | 0.9773 g | 0.9772 g |
| Binary duration | 30 s | 30 s |
| Binary frames | 6002 | 6010 |
| Binary sequence gaps | 0 | 0 |
| Binary integrity clean | 6002 | 6010 |
| Binary \|a\| mean | 0.9783 g | 0.9782 g |
| Binary \|a\| in [0.8, 1.2] | 100% | 100% |
| Harness verdict | PASS | PASS |

### Metric files (local, gitignored logs/)

- `logs/hw-regression/main-f64-text-20260720-222644.metrics.txt`
- `logs/hw-regression/main-f64-binary-20260720-222857.metrics.txt`
- `logs/hw-regression/branch-f32-text-20260720-223209.metrics.txt`
- `logs/hw-regression/branch-f32-binary-20260720-223314.metrics.txt`

## Interpretation

- **End-to-end acquisition did not regress** after physical-only `ImuSample` + `f32`.
- **f32 precision is not proven by board noise**; host full-domain tests cover conversion error ≪ 1 LSB.
- Stream stamps remain on the **board** (`StampedSample` / binary frame), not on the driver type.

## Re-run

```bash
# baseline (f64 API commit / main)
LABEL=main-f64 DURATION=60 MODE=text ./scripts/hw-acquisition-regression.sh
LABEL=main-f64 DURATION=30 MODE=binary ./scripts/hw-acquisition-regression.sh

# after 0.2 API + board wrap
LABEL=branch-f32 DURATION=60 MODE=text ./scripts/hw-acquisition-regression.sh
LABEL=branch-f32 DURATION=30 MODE=binary ./scripts/hw-acquisition-regression.sh
```
