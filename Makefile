PORT ?=
BAUD ?= 115200
TARGET ?= riscv32imc-unknown-none-elf
P4_TARGET ?= riscv32imafc-unknown-none-elf
BOARD_DRIVER_PATCH := patch.crates-io.mpu6050-driver.path="crates/mpu6050-driver"
P4_RUSTFLAGS := -Clink-arg=-Tlinkall.x -Cforce-frame-pointers=yes
P4_FEATURES ?= boot-probe
P4_FEATURE_ARGS := $(if $(strip $(P4_FEATURES)),--features $(P4_FEATURES),)
P4_CARGO ?= cargo +1.95.0
P4_CHIP_REVISION ?= 100
P4_ENV := ESP_HAL_CONFIG_MIN_CHIP_REVISION=$(P4_CHIP_REVISION) ESP_SYNC_CONFIG_MIN_CHIP_REVISION=$(P4_CHIP_REVISION) ESP_HAL_CONFIG_STACK_GUARD_MONITORING=false ESP_HAL_CONFIG_STACK_GUARD_MONITORING_WITH_DEBUGGER_CONNECTED=false
LOG_DIR ?= logs
LOG_FILE ?=
DURATION ?=
MODE ?= text
NO_FLASH ?= 0
NO_MONITOR ?= 0
NO_LOG ?= 0
SECONDS ?= 600
SECONDS_PER_FACE ?= 8
SAMPLE_RATE_HZ ?= 10
LABEL ?= stationary
VALIDATION_MODE ?= report
EXPECTED_ADDRESS ?= 0x68
EXPECTED_IDENTITY ?= any-known
MAG_MIN ?= 0.80
MAG_MAX ?= 1.20
DOMINANCE ?= 0.70
NOISE_PSD_BAND_LOW_HZ ?=
NOISE_PSD_BAND_HIGH_HZ ?=
MIN_SAMPLES_PER_AXIS ?= 10
MIN_SAMPLES_PER_FACE ?= 5
MIN_SAMPLES ?=
MIN_STATIONARY_SAMPLES ?=
MAPPING ?=
CSV_FILE ?=
OUT ?=
BIN := target/$(TARGET)/release/mpu6050-esp32c3-bringup
P4_BIN := target/$(P4_TARGET)/release/mpu6050-esp32p4-bringup

.DEFAULT_GOAL := help

.PHONY: help fmt check check-host check-firmware check-firmware-p4 test clippy build build-p4 build-p4-full flash flash-p4 monitor monitor-p4 run run-p4 clean capture analyze stationary-suite orientation-capture orientation-analyze sixface-capture sixface-analyze sixface-calibration export-csv allan psd smoke validate-stationary validate-orientation imu-tool-smoke

help:
	@printf '%s\n' 'MPU6050 driver workspace'
	@printf '%s\n' ''
	@printf '%s\n' 'Core:'
	@printf '%s\n' '  make fmt                         Format all Rust crates'
	@printf '%s\n' '  make check                       Run format and host checks'
	@printf '%s\n' '  make check-host                  Run host-side package checks'
	@printf '%s\n' '  make check-firmware              Run ESP32-C3 firmware target checks'
	@printf '%s\n' '  make test                        Run host-side tests'
	@printf '%s\n' '  make clippy                      Run clippy checks'
	@printf '%s\n' ''
	@printf '%s\n' 'ESP32-C3:'
	@printf '%s\n' '  make build                       Build firmware'
	@printf '%s\n' '  make flash PORT=...              Flash firmware'
	@printf '%s\n' '  make monitor PORT=...            Monitor serial output'
	@printf '%s\n' '  make monitor PORT=... DURATION=300 MODE=text|binary'
	@printf '%s\n' '  make run PORT=... DURATION=300 MODE=text|binary'
	@printf '%s\n' ''
	@printf '%s\n' 'ESP32-P4:'
	@printf '%s\n' '  Targets ESP32-P4 rev v1.3/ECO2/pre-v3.0 by default; set P4_CHIP_REVISION to match your silicon.'
	@printf '%s\n' '  make check-firmware-p4             Check P4 firmware modes'
	@printf '%s\n' '  make build-p4                      Build P4 boot-probe firmware (safe default)'
	@printf '%s\n' '  make build-p4-full                 Build P4 default/full pipeline firmware'
	@printf '%s\n' '  make build-p4 P4_FEATURES=         Build P4 default/full pipeline firmware'
	@printf '%s\n' '  make build-p4 P4_FEATURES=i2c-probe Build P4 I2C diagnostic'
	@printf '%s\n' '  make build-p4 P4_FEATURES=i2c-bitbang Build P4 bit-bang I2C diagnostic'
	@printf '%s\n' '  make build-p4 P4_FEATURES=mpu-smoke Build P4 firmware'
	@printf '%s\n' '  make flash-p4 PORT=... P4_FEATURES=mpu-smoke Flash P4 firmware'
	@printf '%s\n' '  make monitor-p4 PORT=...          Monitor already-flashed P4 firmware'
	@printf '%s\n' '  make run-p4 PORT=... P4_FEATURES=  Flash and monitor P4 firmware'
	@printf '%s\n' ''
	@printf '%s\n' 'Validation:'
	@printf '%s\n' '  make capture PORT=... LOG_FILE=logs/stationary.log MODE=text|binary'
	@printf '%s\n' '  make analyze LOG_FILE=logs/stationary.log EXPECTED_ADDRESS=0x68 EXPECTED_IDENTITY=any-known'
	@printf '%s\n' '  make stationary-suite PORT=... SECONDS=600'
	@printf '%s\n' '  make orientation-capture PORT=... LOG_FILE=logs/orientation.log MAG_MIN=0.80 MAG_MAX=1.20 DOMINANCE=0.70'
	@printf '%s\n' '  make orientation-analyze LOG_FILE=logs/orientation.log MAG_MIN=0.80 MAG_MAX=1.20 DOMINANCE=0.70'
	@printf '%s\n' '  make sixface-capture PORT=... LOG_FILE=logs/sixface.log'
	@printf '%s\n' '  make sixface-analyze LOG_FILE=logs/sixface.log [MAPPING=config/sixface-mapping.local.json]'
	@printf '%s\n' '  make sixface-calibration LOG_FILE=logs/sixface.log'
	@printf '%s\n' '  make export-csv LOG_FILE=logs/stationary.log'
	@printf '%s\n' '  make allan CSV_FILE=logs/samples.csv'
	@printf '%s\n' '  make psd CSV_FILE=logs/samples.csv'
	@printf '%s\n' '  make smoke'
	@printf '%s\n' ''
	@printf '%s\n' 'Compatibility aliases: validate-stationary validate-orientation imu-tool-smoke'

fmt:
	cargo fmt --all

check: fmt check-host

check-host:
	cargo fmt --all -- --check
	cargo check --locked -p imu-tool -p mpu6050-driver --all-targets --all-features

check-firmware:
	cargo fmt --all -- --check
	env -u RUSTFLAGS CARGO_TARGET_DIR=target cargo --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-c3/Cargo.toml --target $(TARGET)

check-firmware-p4:
	cargo fmt --all -- --check
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-p4/Cargo.toml --target $(P4_TARGET)
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-p4/Cargo.toml --target $(P4_TARGET) --features boot-probe
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-p4/Cargo.toml --target $(P4_TARGET) --features i2c-probe
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-p4/Cargo.toml --target $(P4_TARGET) --features i2c-bitbang
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-p4/Cargo.toml --target $(P4_TARGET) --features mpu-smoke
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' check --locked --manifest-path boards/esp32-p4/Cargo.toml --target $(P4_TARGET) --features pin-wiggle

test:
	cargo test --locked -p mpu6050-driver
	cargo test --locked -p imu-tool -p mpu6050-driver --all-features

clippy:
	cargo clippy --locked -p imu-tool -p mpu6050-driver --all-targets --all-features -- -D warnings

build:
	env -u RUSTFLAGS CARGO_TARGET_DIR=target cargo --config '$(BOARD_DRIVER_PATCH)' build --manifest-path boards/esp32-c3/Cargo.toml --release --target $(TARGET)

build-p4:
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' build --locked --manifest-path boards/esp32-p4/Cargo.toml --release --target $(P4_TARGET) $(P4_FEATURE_ARGS)

build-p4-full:
	env $(P4_ENV) RUSTFLAGS='$(P4_RUSTFLAGS)' CARGO_TARGET_DIR=target $(P4_CARGO) --config '$(BOARD_DRIVER_PATCH)' build --locked --manifest-path boards/esp32-p4/Cargo.toml --release --target $(P4_TARGET)

flash: build
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'env -u RUSTFLAGS espflash flash --port "$$ESP_PORT" "$(BIN)"'

flash-p4: build-p4
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'espflash flash --port "$$ESP_PORT" "$(P4_BIN)"'

monitor:
	PORT="$(PORT)" BAUD="$(BAUD)" TARGET="$(TARGET)" LOG_DIR="$(LOG_DIR)" LOG_FILE="$(LOG_FILE)" DURATION="$(DURATION)" MODE="$(MODE)" NO_FLASH=1 NO_MONITOR="$(NO_MONITOR)" NO_LOG="$(NO_LOG)" ./run.sh

monitor-p4:
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- monitor --port "$$ESP_PORT" --baud "$(BAUD)" $(if $(DURATION),--duration "$(DURATION)") $(if $(LOG_FILE),--out "$(LOG_FILE)") --mode "$(MODE)"'

run:
	PORT="$(PORT)" BAUD="$(BAUD)" TARGET="$(TARGET)" LOG_DIR="$(LOG_DIR)" LOG_FILE="$(LOG_FILE)" DURATION="$(DURATION)" MODE="$(MODE)" NO_FLASH="$(NO_FLASH)" NO_MONITOR="$(NO_MONITOR)" NO_LOG="$(NO_LOG)" ./run.sh

run-p4: flash-p4
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- monitor --port "$$ESP_PORT" --baud "$(BAUD)" $(if $(DURATION),--duration "$(DURATION)") $(if $(LOG_FILE),--out "$(LOG_FILE)") --mode "$(MODE)"'

clean:
	cargo clean

capture:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make capture LOG_FILE=logs/capture.log PORT=/dev/ttyUSB0)' >&2; exit 2; }
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- capture --port "$$ESP_PORT" --seconds "$(SECONDS)" --baud "$(BAUD)" --out "$(LOG_FILE)" --mode "$(MODE)"'

analyze:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make analyze LOG_FILE=logs/capture.log)' >&2; exit 2; }
	cargo run -p imu-tool -- analyze "$(LOG_FILE)" --expected-address "$(EXPECTED_ADDRESS)" --expected-identity "$(EXPECTED_IDENTITY)" $(if $(MIN_SAMPLES),--min-samples "$(MIN_SAMPLES)") $(if $(MIN_STATIONARY_SAMPLES),--min-stationary-samples "$(MIN_STATIONARY_SAMPLES)")

stationary-suite:
	PORT="$(PORT)" NOISE_PSD_BAND_LOW_HZ="$(NOISE_PSD_BAND_LOW_HZ)" NOISE_PSD_BAND_HIGH_HZ="$(NOISE_PSD_BAND_HIGH_HZ)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- stationary-suite --port "$$ESP_PORT" --seconds "$(SECONDS)" --baud "$(BAUD)" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --label "$(LABEL)" --out-dir "$(LOG_DIR)" --validation-mode "$(VALIDATION_MODE)" $${NOISE_PSD_BAND_LOW_HZ:+--noise-psd-band-low-hz "$$NOISE_PSD_BAND_LOW_HZ"} $${NOISE_PSD_BAND_HIGH_HZ:+--noise-psd-band-high-hz "$$NOISE_PSD_BAND_HIGH_HZ"}'

orientation-capture:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make orientation-capture LOG_FILE=logs/orientation.log PORT=/dev/ttyUSB0)' >&2; exit 2; }
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- orientation-capture --port "$$ESP_PORT" --seconds "$(SECONDS)" --baud "$(BAUD)" --stop-when-covered --min-samples-per-axis "$(MIN_SAMPLES_PER_AXIS)" --mag-min "$(MAG_MIN)" --mag-max "$(MAG_MAX)" --dominance "$(DOMINANCE)" --out "$(LOG_FILE)"'

orientation-analyze:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make orientation-analyze LOG_FILE=logs/orientation.log)' >&2; exit 2; }
	cargo run -p imu-tool -- orientation-analyze "$(LOG_FILE)" --min-samples-per-axis "$(MIN_SAMPLES_PER_AXIS)" --mag-min "$(MAG_MIN)" --mag-max "$(MAG_MAX)" --dominance "$(DOMINANCE)"

sixface-capture:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make sixface-capture LOG_FILE=logs/sixface.log PORT=/dev/ttyUSB0)' >&2; exit 2; }
	PORT="$(PORT)" ./scripts/esp-port.sh sh -c 'cargo run -p imu-tool -- sixface-capture --port "$$ESP_PORT" --seconds-per-face "$(SECONDS_PER_FACE)" --baud "$(BAUD)" --out "$(LOG_FILE)"'

sixface-analyze:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make sixface-analyze LOG_FILE=logs/sixface.log)' >&2; exit 2; }
	MAPPING="$(MAPPING)" sh -c 'cargo run -p imu-tool -- sixface-analyze "$(LOG_FILE)" --min-samples-per-face "$(MIN_SAMPLES_PER_FACE)" $${MAPPING:+--mapping "$$MAPPING"}'

sixface-calibration:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make sixface-calibration LOG_FILE=logs/sixface.log)' >&2; exit 2; }
	cargo run -p imu-tool -- sixface-calibration "$(LOG_FILE)" --out "$(or $(OUT),$(LOG_DIR)/sixface-calibration-$$(date +%Y%m%d-%H%M%S).json)"

export-csv:
	@test -n "$(LOG_FILE)" || { printf '%s\n' 'LOG_FILE is required (for example: make export-csv LOG_FILE=logs/capture.log)' >&2; exit 2; }
	cargo run -p imu-tool -- export-csv "$(LOG_FILE)" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --out "$(or $(OUT),$(LOG_DIR)/samples-$$(date +%Y%m%d-%H%M%S).csv)"

allan:
	@test -n "$(or $(CSV_FILE),$(LOG_FILE))" || { printf '%s\n' 'CSV_FILE or LOG_FILE is required (for example: make allan CSV_FILE=logs/samples.csv)' >&2; exit 2; }
	cargo run -p imu-tool -- allan-analyze "$(or $(CSV_FILE),$(LOG_FILE))" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --out "$(or $(OUT),$(LOG_DIR)/allan-$$(date +%Y%m%d-%H%M%S).csv)"

psd:
	@test -n "$(or $(CSV_FILE),$(LOG_FILE))" || { printf '%s\n' 'CSV_FILE or LOG_FILE is required (for example: make psd CSV_FILE=logs/samples.csv)' >&2; exit 2; }
	cargo run -p imu-tool -- psd-analyze "$(or $(CSV_FILE),$(LOG_FILE))" --sample-rate-hz "$(SAMPLE_RATE_HZ)" --out "$(or $(OUT),$(LOG_DIR)/psd-$$(date +%Y%m%d-%H%M%S).csv)"

smoke:
	cargo run --locked -p imu-tool -- analyze tools/imu-tool/tests/fixtures/stationary-60s.log --min-samples 20
	cargo run --locked -p imu-tool -- orientation-analyze tools/imu-tool/tests/fixtures/auto-orientation.log --min-samples-per-axis 3
	cargo run --locked -p imu-tool -- sixface-analyze tools/imu-tool/tests/fixtures/sixface.log --mapping config/sixface-mapping.example.json || test $$? -eq 1
	cargo test --locked -p imu-tool sixface_fixture_parses_real_face_samples

validate-stationary: stationary-suite

validate-orientation: orientation-capture

imu-tool-smoke: smoke
