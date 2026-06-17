# MPU6050 driver and validation workspace

Rust workspace for bringing up MPU6050-class IMUs, collecting real sensor logs,
and validating driver behavior with repeatable host-side analysis.

The project is intentionally focused on:

- reusable driver/core data types
- calibration and validation workflows
- independent tests from captured logs
- IEEE-style evidence: raw data, reproducible analysis, and explicit limits

It does **not** try to prove whether a module is genuine from chip markings. Board
photos and `WHO_AM_I` values are useful context, but product claims should be
based on measured motion, calibration, noise, timing, FIFO/interrupt behavior, and
documented test conditions.

Current hardware photos:

<img src="hardware/boards/gy521-blue-clone-v1/photos/front-overview.jpg" alt="Front overview" width="50%" />

## ESP32-C3 reference wiring

Firmware sample: `boards/esp32-c3/src/main.rs`.

| GY-521 / MPU6050 | ESP32-C3 SuperMini-class board |
| --- | --- |
| VCC | 3V |
| GND | GND |
| SCL | GPIO0 |
| SDA | GPIO1 |
| XDA | GPIO3, unused |
| XCL | GPIO4, unused |
| AD0 | GPIO5, driven low |
| INT | GPIO6 |

`AD0` low selects I2C address `0x68`; high selects `0x69`. The sample drives
`AD0` low and probes both addresses for diagnostics.

## Build and flash

Flash and monitor:

```bash
./run.sh
```

Override serial port:

```bash
PORT=/dev/cu.usbmodem11301 ./run.sh
```

Build only:

```bash
NO_FLASH=1 NO_MONITOR=1 ./run.sh
```

Package checks:

```bash
cargo check -p imu-core -p imu-validation -p imu-tool
cargo check -p mpu6050-esp32c3-bringup --target riscv32imc-unknown-none-elf
```

## Independent validation workflow

All validation commands operate on real serial logs. They should not generate or
substitute mock samples when producing hardware evidence.

Capture a stationary log:

```bash
PORT=/dev/cu.usbmodem1101
cargo run -p imu-tool -- capture \
  --port "$PORT" \
  --seconds 60 \
  --out logs/stationary-60s.log
```

Analyze the log:

```bash
cargo run -p imu-tool -- analyze \
  logs/stationary-60s.log \
  --min-samples 50 \
  --min-stationary-samples 45
```

The analyzer checks measured evidence such as:

- expected I2C address in firmware output
- readable identity and power-management registers
- raw accel/temp/gyro sample count
- stationary acceleration magnitude near 1 g
- gyro bias/noise plausibility while still
- temperature raw value not stuck
- advanced scale-range, self-test, FIFO, and interrupt checks when present

## Raw sample integrity layer

The driver keeps `read_raw_accel_gyro_temp()` as an unmodified raw register-read
primitive.

For measurement workflows, use:

- `read_raw_checked()`
- `read_raw_with_retry(RawRetryPolicy::reject_after_retries(1))`

This Level 1 layer rejects sentinel-like or observed invalid raw samples before
calibration. It is not a motion filter and does not remove normal IMU noise.

Normal noise, bias, scale error, axis misalignment, and temperature drift belong
to calibration and signal-processing layers.

## Six-face and orientation tests

Strict six-face capture requires physically rotating the module:

```bash
cargo run -p imu-tool -- sixface-capture \
  --port "$PORT" \
  --seconds-per-face 8 \
  --out logs/sixface.log

cargo run -p imu-tool -- sixface-analyze logs/sixface.log
```

Two modes are supported:

1. **Axis coverage mode**: no mapping file. Validates that gravity appears on all
   six signed accelerometer axes.
2. **Fixture certification mode**: uses a mapping file to prove that physical
   face labels match expected sensor axes.

Mapping mode:

```bash
cp config/sixface-mapping.example.json config/sixface-mapping.local.json
# edit config/sixface-mapping.local.json for the fixture

cargo run -p imu-tool -- sixface-analyze \
  logs/sixface.log \
  --mapping config/sixface-mapping.local.json
```

For faster debug, use free-rotation orientation coverage:

```bash
cargo run -p imu-tool -- orientation-capture \
  --port "$PORT" \
  --seconds 60 \
  --stop-when-covered \
  --min-samples-per-axis 5 \
  --out logs/orientation.log

cargo run -p imu-tool -- orientation-analyze \
  logs/orientation.log \
  --min-samples-per-axis 5
```

Free-rotation coverage is useful for debug, but it does not certify product
fixture face labels unless paired with the mapped six-face test.

## Calibration

Six-position calibration summary:

```bash
cargo run -p imu-tool -- sixface-calibration \
  logs/sixface.log \
  --out logs/sixface-calibration.json
```

The output records per-face accelerometer mean, standard deviation, magnitude,
and coverage evidence. It is a static six-position calibration aid; it does not
yet estimate full misalignment, non-orthogonality, or temperature compensation.

Recommended calibration evidence to keep with a release or hardware report:

- raw six-face log
- mapping file, if fixture certification is claimed
- calibration JSON
- firmware version/commit
- board wiring and photos
- sample rate and capture duration

## Noise and IEEE-style reporting

Long stationary captures support Allan deviation and PSD analysis. This follows
common IMU characterization practice: preserve the raw log, export a time-series
CSV, compute repeatable metrics, and state the limits of the measurement setup.

Automated stationary suite:

```bash
cargo run -p imu-tool -- stationary-suite \
  --port "$PORT" \
  --seconds 600 \
  --sample-rate-hz 10 \
  --validation-mode report \
  --label stationary
```

This creates a timestamped directory similar to:

```text
logs/stationary-20260613-204512/
  raw.log
  samples.csv
  allan.csv
  psd.csv
  stationary-report.json
  manifest.json
```

Manual equivalent:

```bash
cargo run -p imu-tool -- export-csv \
  logs/stationary-10min.log \
  --sample-rate-hz 10 \
  --out logs/stationary-10min.csv

cargo run -p imu-tool -- allan-analyze \
  logs/stationary-10min.csv \
  --sample-rate-hz 10 \
  --out logs/allan-stationary-10min.csv

cargo run -p imu-tool -- psd-analyze \
  logs/stationary-10min.csv \
  --sample-rate-hz 10 \
  --out logs/psd-stationary-10min.csv
```

Reporting rules:

- Keep raw logs and generated CSV/JSON artifacts together.
- Report sample rate, duration, firmware commit, and validation mode.
- Distinguish smoke-level results from publication-grade characterization.
- Do not claim datasheet-grade noise numbers from a short UART text stream.
- Prefer measured timestamps and a deterministic binary/CSV stream for formal
  IEEE-style characterization.

## Current scope

The current firmware can bring up the board, read MPU-style registers, stream raw
motion samples, and exercise advanced register/FIFO/INT behavior. The host tool
can independently validate captured logs and generate calibration/noise evidence.

Useful next work before stronger product claims:

1. Move more register/sample logic out of the board sample into reusable driver
   modules.
2. Add checked calibration application paths, not only calibration reports.
3. Add deterministic timestamping for Allan/PSD captures.
4. Expand fixture-certified six-face tests with saved mapping evidence.
5. Add CI checks for host crates and target-specific board firmware.
