use regex::Regex;
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{self, Read, Write},
    path::Path,
    time::{Duration, Instant},
};

const ACCEL: f64 = 16384.0;
const GYRO: f64 = 131.0;
const FACES: [&str; 6] = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];
type ToolResult = Result<i32, Box<dyn std::error::Error>>;

/// Binary IMU frame format, little-endian:
/// magic[2]="IM", version u8=1, payload_len u8=32,
/// payload: address u8, reserved u8, sequence u64, timestamp_us u64,
/// ax i16, ay i16, az i16, temp i16, gx i16, gy i16, gz i16,
/// crc16-ccitt over header+payload (not including crc).
pub const BINARY_FRAME_MAGIC: [u8; 2] = *b"IM";
pub const BINARY_FRAME_VERSION: u8 = 1;
pub const BINARY_FRAME_PAYLOAD_LEN: u8 = 32;
pub const BINARY_FRAME_LEN: usize = 2 + 1 + 1 + BINARY_FRAME_PAYLOAD_LEN as usize + 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamMode {
    Text,
    Binary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationMode {
    Report,
    Strict,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExpectedIdentity {
    ClassicMpu6050,
    Mpu6500Compatible,
    AnyKnown,
    Exact(u8),
}

impl ExpectedIdentity {
    fn matches(self, observed: u8) -> bool {
        match self {
            Self::ClassicMpu6050 => observed == 0x68,
            Self::Mpu6500Compatible => observed == 0x70,
            Self::AnyKnown => matches!(observed, 0x68 | 0x70),
            Self::Exact(expected) => observed == expected,
        }
    }
}

fn stationary_suite_exit_code(
    tool_return_codes: [i32; 5],
    validation_mode: ValidationMode,
    physical_thresholds_failed: bool,
) -> i32 {
    let tools_ok = tool_return_codes.into_iter().all(|x| x == 0);
    let validation_ok = match validation_mode {
        ValidationMode::Report => true,
        ValidationMode::Strict => !physical_thresholds_failed,
    };

    if tools_ok && validation_ok { 0 } else { 1 }
}

fn open_serial(
    port: &str,
    baud: u32,
) -> Result<Box<dyn serialport::SerialPort>, serialport::Error> {
    let mut ser = serialport::new(port, baud)
        .timeout(Duration::from_millis(500))
        .open()?;
    let _ = ser.write_data_terminal_ready(false);
    let _ = ser.write_request_to_send(false);
    std::thread::sleep(Duration::from_millis(200));
    Ok(ser)
}

fn write_parent(path: &Path) -> io::Result<()> {
    if let Some(p) = path.parent() {
        fs::create_dir_all(p)?;
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct IntegrityStats {
    total: u64,
    recovered: u64,
    rejected: u64,
    retry_error: u64,
    accepted: u64,
}

impl IntegrityStats {
    fn record_sample(&mut self) {
        self.total = self.total.saturating_add(1);
    }

    fn record_event(&mut self, event: RawIntegrityEvent) {
        match event.outcome.as_str() {
            "recovered" => self.recovered = self.recovered.saturating_add(1),
            "rejected" => self.rejected = self.rejected.saturating_add(1),
            "retry_error" => self.retry_error = self.retry_error.saturating_add(1),
            "accepted" => self.accepted = self.accepted.saturating_add(1),
            _ => {}
        }
    }

    fn record_text(&mut self, text: &str, line_buf: &mut String) {
        line_buf.push_str(text);
        while let Some(pos) = line_buf.find('\n') {
            let mut line = line_buf.drain(..=pos).collect::<String>();
            line.truncate(line.trim_end_matches(['\r', '\n']).len());
            self.record_line(&line);
        }
    }

    fn record_line(&mut self, line: &str) {
        if parse_raw_line(line).is_some() {
            self.record_sample();
        }
        if let Some(event) = parse_raw_integrity_event(line) {
            self.record_event(event);
        }
    }

    fn suspicious(&self) -> u64 {
        self.recovered
            .saturating_add(self.rejected)
            .saturating_add(self.retry_error)
            .saturating_add(self.accepted)
    }

    fn clean(&self) -> u64 {
        self.total
            .saturating_sub(self.recovered.saturating_add(self.accepted))
    }

    fn summary_line(&self) -> String {
        format!(
            "integrity_stats samples={} clean_samples={} suspicious_events={} recovered={} rejected={} retry_error={} accepted={}",
            self.total,
            self.clean(),
            self.suspicious(),
            self.recovered,
            self.rejected,
            self.retry_error,
            self.accepted
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct RawIntegrityEvent {
    outcome: String,
}

fn parse_raw_integrity_event(line: &str) -> Option<RawIntegrityEvent> {
    let mut fields = line.split_whitespace();
    if fields.next()? != "raw_integrity_event" {
        return None;
    }
    let mut outcome = None;
    for field in fields {
        let Some((key, value)) = field.split_once('=') else {
            continue;
        };
        if key == "outcome" && !value.is_empty() {
            outcome = Some(value.to_ascii_lowercase());
        }
    }
    Some(RawIntegrityEvent { outcome: outcome? })
}

fn emit_integrity_stats<W: Write>(stats: &IntegrityStats, log: Option<&mut W>) -> io::Result<()> {
    let line = stats.summary_line();
    println!("{line}");
    if let Some(log) = log {
        writeln!(log, "{line}")?;
        log.flush()?;
    }
    Ok(())
}

fn record_partial_text_line(stats: &mut IntegrityStats, line_buf: &str) {
    let line = line_buf.trim_end_matches(['\r', '\n']).trim();
    if !line.is_empty() {
        stats.record_line(line);
    }
}

fn read_serial_for<W: Write>(
    #[allow(unused_mut)] mut ser: &mut dyn serialport::SerialPort,
    seconds: f64,
    out: &mut W,
    mut on_text: impl FnMut(&str),
) -> io::Result<()> {
    let deadline = Instant::now() + Duration::from_secs_f64(seconds.max(0.0));
    let mut buf = [0u8; 2048];
    while Instant::now() < deadline {
        match ser.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                let text = String::from_utf8_lossy(&buf[..n]);
                print!("{text}");
                io::stdout().flush()?;
                out.write_all(text.as_bytes())?;
                out.flush()?;
                on_text(&text);
            }
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

fn monitor_deadline(duration: Option<f64>, now: Instant) -> Option<Instant> {
    duration.map(|s| now + Duration::from_secs_f64(s.max(0.0)))
}

fn before_monitor_deadline(deadline: Option<Instant>, now: Instant) -> bool {
    deadline.is_none_or(|deadline| now < deadline)
}

fn read_serial_binary_for<W: Write>(
    ser: &mut dyn serialport::SerialPort,
    seconds: Option<f64>,
    out: &mut W,
    mut stats: Option<&mut IntegrityStats>,
) -> io::Result<()> {
    let deadline = monitor_deadline(seconds, Instant::now());
    let mut buf = [0u8; 2048];
    let mut dec = BinaryFrameDecoder::new();
    while before_monitor_deadline(deadline, Instant::now()) {
        match ser.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                for ev in dec.push(&buf[..n]) {
                    match ev {
                        BinaryDecodeEvent::Sample(s) => {
                            if let Some(stats) = stats.as_mut() {
                                stats.record_sample();
                            }
                            let line = raw_sample_line(&s);
                            println!("{line}");
                            writeln!(out, "{line}")?;
                        }
                        BinaryDecodeEvent::Warning(w) => eprintln!("binary_frame_warning: {w}"),
                    }
                }
                out.flush()?;
            }
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

pub fn capture(port: &str, baud: u32, seconds: f64, out: &Path, mode: StreamMode) -> ToolResult {
    write_parent(out)?;
    let mut ser = open_serial(port, baud)?;
    let mut f = fs::File::create(out)?;
    writeln!(
        f,
        "# capture_start port={port} baud={baud} seconds={seconds} mode={mode:?}"
    )?;
    println!(
        "capturing real serial data from {port} for {seconds:.1}s -> {}",
        out.display()
    );
    match mode {
        StreamMode::Text => read_serial_for(&mut *ser, seconds, &mut f, |_| {})?,
        StreamMode::Binary => read_serial_binary_for(&mut *ser, Some(seconds), &mut f, None)?,
    }
    writeln!(f, "\n# capture_end")?;
    Ok(0)
}

pub fn monitor(
    port: &str,
    baud: u32,
    duration: Option<f64>,
    out: Option<&Path>,
    mode: StreamMode,
) -> ToolResult {
    let mut ser = open_serial(port, baud)?;
    let mut log = if let Some(path) = out {
        write_parent(path)?;
        Some(fs::File::create(path)?)
    } else {
        None
    };
    println!("--- monitoring {port} @{baud}, Ctrl-C to quit ---");
    let mut stats = IntegrityStats::default();
    if mode == StreamMode::Binary {
        let result = if let Some(f) = log.as_mut() {
            read_serial_binary_for(&mut *ser, duration, f, Some(&mut stats))
        } else {
            read_serial_binary_for(&mut *ser, duration, &mut io::sink(), Some(&mut stats))
        };
        emit_integrity_stats(&stats, log.as_mut())?;
        result?;
    } else {
        let mut buf = [0u8; 2048];
        let mut line_buf = String::new();
        let deadline = monitor_deadline(duration, Instant::now());
        while before_monitor_deadline(deadline, Instant::now()) {
            match ser.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    let text = String::from_utf8_lossy(&buf[..n]);
                    print!("{text}");
                    io::stdout().flush()?;
                    if let Some(f) = log.as_mut() {
                        f.write_all(text.as_bytes())?;
                        f.flush()?;
                    }
                    stats.record_text(&text, &mut line_buf);
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => {
                    record_partial_text_line(&mut stats, &line_buf);
                    if !line_buf.is_empty() {
                        println!();
                        if let Some(f) = log.as_mut() {
                            writeln!(f)?;
                            f.flush()?;
                        }
                    }
                    emit_integrity_stats(&stats, log.as_mut())?;
                    return Err(Box::new(e));
                }
            }
        }
        if !line_buf.is_empty() {
            record_partial_text_line(&mut stats, &line_buf);
            println!();
            if let Some(f) = log.as_mut() {
                writeln!(f)?;
                f.flush()?;
            }
        }
        emit_integrity_stats(&stats, log.as_mut())?;
    }
    Ok(0)
}

#[allow(clippy::too_many_arguments)]
pub fn orientation_capture(
    port: &str,
    baud: u32,
    seconds: f64,
    out: &Path,
    stop_when_covered: bool,
    min_samples_per_axis: usize,
    mag_min: f64,
    mag_max: f64,
    dominance: f64,
) -> ToolResult {
    println!("Auto orientation capture instructions:");
    println!("- rotate/tumble the board slowly through many orientations");
    println!("- pause 1-2 seconds on as many different faces as possible");
    println!("- avoid fast shaking; dynamic acceleration will be filtered out");
    if !stop_when_covered {
        return capture(port, baud, seconds, out, StreamMode::Text);
    }
    write_parent(out)?;
    println!(
        "orientation_capture_opening port={port} baud={baud} out={}",
        out.display()
    );
    let mut ser = open_serial(port, baud)?;
    let mut f = fs::File::create(out)?;
    writeln!(
        f,
        "# orientation_capture_start port={port} baud={baud} max_seconds={seconds} stop_when_covered=true"
    )?;
    let mut bins: BTreeMap<&str, usize> = FACES.into_iter().map(|x| (x, 0)).collect();
    let (mut rm, mut rd) = (0usize, 0usize);
    let mut line_buf = String::new();
    let deadline = Instant::now() + Duration::from_secs_f64(seconds.max(0.0));
    let mut last_progress = Instant::now() - Duration::from_secs(1);
    maybe_print_orientation_progress(&bins, min_samples_per_axis, rm, rd, &mut last_progress)?;
    let mut buf = [0u8; 2048];
    while Instant::now() < deadline && !bins.values().all(|&v| v >= min_samples_per_axis) {
        match ser.read(&mut buf) {
            Ok(0) => {
                maybe_print_orientation_progress(
                    &bins,
                    min_samples_per_axis,
                    rm,
                    rd,
                    &mut last_progress,
                )?;
            }
            Ok(n) => {
                let text = String::from_utf8_lossy(&buf[..n]);
                f.write_all(text.as_bytes())?;
                f.flush()?;
                line_buf.push_str(&text);
                while let Some(pos) = line_buf.find(['\n', '\r']) {
                    let line: String = line_buf.drain(..=pos).collect();
                    if let Some(s) = parse_raw_line(line.trim_end_matches(['\r', '\n'])) {
                        match classify_orientation_sample(&s, mag_min, mag_max, dominance) {
                            (Some(axis), _) => *bins.get_mut(axis).unwrap() += 1,
                            (_, Some("magnitude")) => rm += 1,
                            _ => rd += 1,
                        }
                        maybe_print_orientation_progress(
                            &bins,
                            min_samples_per_axis,
                            rm,
                            rd,
                            &mut last_progress,
                        )?;
                    }
                }
            }
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                maybe_print_orientation_progress(
                    &bins,
                    min_samples_per_axis,
                    rm,
                    rd,
                    &mut last_progress,
                )?;
            }
            Err(e) => return Err(Box::new(e)),
        }
    }
    let covered = bins.values().all(|&v| v >= min_samples_per_axis);
    let missing = orientation_missing_faces(&bins, min_samples_per_axis);
    if covered {
        println!(
            "orientation_capture_complete covered=true bins={} rejected_mag={rm} rejected_dominance={rd}",
            orientation_bins_summary(&bins, min_samples_per_axis)
        );
    } else {
        println!(
            "orientation_capture_incomplete covered=false bins={} missing={} rejected_mag={rm} rejected_dominance={rd}",
            orientation_bins_summary(&bins, min_samples_per_axis),
            missing.join(",")
        );
    }
    writeln!(
        f,
        "\n# orientation_capture_end covered={covered} bins={bins:?} rejected_mag={rm} rejected_dominance={rd}"
    )?;
    drop(f);

    if covered {
        println!("orientation_capture_report_begin");
        let report_code =
            orientation_analyze(out, min_samples_per_axis, mag_min, mag_max, dominance)?;
        println!("orientation_capture_report_end");
        Ok(report_code)
    } else {
        println!(
            "orientation_capture_report_skipped reason=incomplete missing={}",
            missing.join(",")
        );
        Ok(1)
    }
}

fn maybe_print_orientation_progress(
    bins: &BTreeMap<&str, usize>,
    required: usize,
    rejected_mag: usize,
    rejected_dominance: usize,
    last_progress: &mut Instant,
) -> io::Result<()> {
    if last_progress.elapsed() >= Duration::from_secs(1) {
        *last_progress = Instant::now();
        let covered = FACES
            .iter()
            .filter(|face| bins.get(**face).copied().unwrap_or(0) >= required)
            .count();
        println!(
            "orientation_capture_progress covered={}/{} faces={} missing={} rejected_mag={} rejected_dominance={}",
            covered,
            FACES.len(),
            orientation_bins_summary(bins, required),
            orientation_missing_faces(bins, required).join(","),
            rejected_mag,
            rejected_dominance
        );
        io::stdout().flush()?;
    }
    Ok(())
}

fn orientation_bins_summary(bins: &BTreeMap<&str, usize>, required: usize) -> String {
    FACES
        .iter()
        .map(|face| {
            let count = bins.get(face).copied().unwrap_or(0);
            if count >= required {
                format!("{} OK", face)
            } else {
                format!("{} {}/{}", face, count, required)
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

fn orientation_missing_faces(bins: &BTreeMap<&str, usize>, required: usize) -> Vec<&'static str> {
    FACES
        .iter()
        .copied()
        .filter(|face| bins.get(face).copied().unwrap_or(0) < required)
        .collect()
}

fn safe_label(label: &str) -> String {
    let s: String = label
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.') {
                c
            } else {
                '-'
            }
        })
        .collect();
    let s = s.trim_matches('-');
    if s.is_empty() { "run".into() } else { s.into() }
}

fn make_timestamped_run_dir(out_dir: &Path, label: &str) -> io::Result<std::path::PathBuf> {
    let stamp = chrono::Local::now().format("%Y%m%d-%H%M%S");
    let base = safe_label(label);
    for suffix in 0.. {
        let name = if suffix == 0 {
            format!("{base}-{stamp}")
        } else {
            format!("{base}-{stamp}-{suffix}")
        };
        let p = out_dir.join(name);
        match fs::create_dir_all(&p) {
            Ok(()) if p.exists() => return Ok(p),
            Ok(()) => return Ok(p),
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}

#[allow(clippy::too_many_arguments)]
pub fn stationary_suite(
    port: &str,
    baud: u32,
    seconds: f64,
    sample_rate_hz: f64,
    label: &str,
    out_dir: &Path,
    validation_mode: ValidationMode,
    noise_psd_band_low_hz: Option<f64>,
    noise_psd_band_high_hz: Option<f64>,
) -> ToolResult {
    let noise_psd_band = validate_noise_psd_band(noise_psd_band_low_hz, noise_psd_band_high_hz)?;
    let run_dir = make_timestamped_run_dir(out_dir, label)?;
    let raw = run_dir.join("raw.log");
    let csv = run_dir.join("samples.csv");
    let allan = run_dir.join("allan.csv");
    let psd = run_dir.join("psd.csv");
    let manifest = run_dir.join("manifest.json");
    let stationary_report = run_dir.join("stationary-report.json");
    let noise_report = run_dir.join("noise-report.json");
    let calibration_report = run_dir.join("calibration.json");
    let expected = (seconds * sample_rate_hz) as usize;
    let min_samples = std::cmp::max(10, (expected as f64 * 0.80) as usize);
    let min_stationary = std::cmp::max(10, (expected as f64 * 0.60) as usize);
    println!("stationary_suite_run_dir={}", run_dir.display());
    let started = chrono::Local::now().to_rfc3339();
    let capture_rc = capture(port, baud, seconds, &raw, StreamMode::Text)?;
    let export_rc = export_csv(&raw, &csv, sample_rate_hz)?;
    let analyze_rc = analyze(
        &raw,
        min_samples,
        min_stationary,
        0x68,
        ExpectedIdentity::AnyKnown,
    )?;
    let (_, raw_samples) = parse_log(&raw)?;
    let validation_samples: Vec<_> = raw_samples
        .iter()
        .map(|s| imu_validation::ImuSample {
            accel_g: s.accel_g(),
            gyro_dps: s.gyro_dps(),
            timestamp_s: s.timestamp_s,
            sequence: s.sequence,
        })
        .collect();
    let validation_cfg = imu_validation::StationaryConfig {
        nominal_rate_hz: Some(sample_rate_hz),
        min_samples,
        min_stationary_fraction: min_stationary as f64 / expected.max(1) as f64,
        ..Default::default()
    };
    let report = imu_validation::analyze_stationary(&validation_samples, &validation_cfg);
    let physical_failed = report.physical_thresholds_failed();
    fs::write(
        &stationary_report,
        serde_json::to_string_pretty(&report)? + "\n",
    )?;
    let calibration_json = imu_validation::estimate_gyro_bias_calibration(
        &validation_samples,
        &imu_validation::GyroBiasCalibrationConfig::default(),
    );
    fs::write(
        &calibration_report,
        serde_json::to_string_pretty(&calibration_json)? + "\n",
    )?;
    let noise_json = imu_validation::noise::analyze_imu_noise(
        &validation_samples,
        &imu_validation::noise::ImuNoiseReportConfig {
            timing: imu_validation::noise::NoiseTimingConfig {
                nominal_rate_hz: Some(sample_rate_hz),
                observed_rate_tolerance_fraction: 0.05,
                jitter_ratio_max: 0.05,
            },
            psd_band_hz: noise_psd_band,
        },
    );
    fs::write(
        &noise_report,
        serde_json::to_string_pretty(&noise_json)? + "\n",
    )?;
    let allan_rc = allan_analyze(&csv, sample_rate_hz, &allan)?;
    let psd_rc = psd_analyze(&csv, sample_rate_hz, &psd)?;
    let ended = chrono::Local::now().to_rfc3339();
    fs::write(
        &manifest,
        serde_json::to_string_pretty(&json!({
            "label": label,
            "started": started,
            "ended": ended,
            "port": port,
            "baud": baud,
            "seconds_requested": seconds,
            "sample_rate_hz_assumed": sample_rate_hz,
            "min_samples": min_samples,
            "min_stationary_samples": min_stationary,
            "files": {"raw_log": raw, "samples_csv": csv, "allan_csv": allan, "psd_csv": psd, "stationary_report_json": stationary_report, "noise_report_json": noise_report, "calibration_json": calibration_report},
            "validation_mode": match validation_mode { ValidationMode::Report => "report", ValidationMode::Strict => "strict" },
            "physical_thresholds_failed": physical_failed,
            "return_codes": {"capture": capture_rc, "export_csv": export_rc, "analyze": analyze_rc, "allan": allan_rc, "psd": psd_rc},
            "publication_grade": false,
            "note": "Automated stationary suite uses real serial data. Current timestamps are assumed from sample_rate_hz; use measured timestamps for publication-grade timing/noise metrology."
        }))? + "\n",
    )?;
    println!("stationary_suite_manifest={}", manifest.display());
    println!("stationary_report_json={}", stationary_report.display());
    Ok(stationary_suite_exit_code(
        [capture_rc, export_rc, analyze_rc, allan_rc, psd_rc],
        validation_mode,
        physical_failed,
    ))
}

fn validate_noise_psd_band(
    low: Option<f64>,
    high: Option<f64>,
) -> Result<Option<[f64; 2]>, Box<dyn std::error::Error>> {
    match (low, high) {
        (None, None) => Ok(None),
        (Some(l), Some(h)) if l.is_finite() && h.is_finite() && l > 0.0 && h > l => {
            Ok(Some([l, h]))
        }
        (Some(_), None) | (None, Some(_)) => {
            Err("both --noise-psd-band-low-hz and --noise-psd-band-high-hz are required".into())
        }
        _ => Err("invalid noise PSD band: require finite 0 < low < high".into()),
    }
}

pub fn sixface_capture(port: &str, baud: u32, seconds_per_face: f64, out: &Path) -> ToolResult {
    write_parent(out)?;
    let mut ser = open_serial(port, baud)?;
    let mut f = fs::File::create(out)?;
    let help = [
        (
            "+X",
            "make the sensor/module +X axis point upward; keep still",
        ),
        ("-X", "flip so the -X axis points upward; keep still"),
        ("+Y", "make the +Y axis point upward; keep still"),
        ("-Y", "flip so the -Y axis points upward; keep still"),
        (
            "+Z",
            "make the top/package side or expected +Z side point upward; keep still",
        ),
        ("-Z", "flip so the opposite side points upward; keep still"),
    ];
    println!("Six-face capture uses real sensor data only.");
    for (face, text) in help {
        println!("\n=== Face {face} ===\n{text}");
        println!("Press Enter when the board is still...");
        let mut s = String::new();
        io::stdin().read_line(&mut s)?;
        for n in (1..=3).rev() {
            println!("settling {n}...");
            std::thread::sleep(Duration::from_secs(1));
            let _ = ser.clear(serialport::ClearBuffer::Input);
        }
        writeln!(f, "# face_begin {face}\n# face_help {face} {text}")?;
        println!("capturing face {face} for {seconds_per_face:.1}s");
        read_serial_for(&mut *ser, seconds_per_face, &mut f, |_| {})?;
        writeln!(f, "# face_end {face}")?;
    }
    Ok(0)
}

#[derive(Clone, Debug, PartialEq)]
pub struct RawSample {
    pub address: i32,
    pub ax: i32,
    pub ay: i32,
    pub az: i32,
    pub temp_raw: i32,
    pub gx: i32,
    pub gy: i32,
    pub gz: i32,
    pub timestamp_s: Option<f64>,
    pub sequence: Option<u64>,
}
impl RawSample {
    pub fn accel_g(&self) -> [f64; 3] {
        [
            self.ax as f64 / ACCEL,
            self.ay as f64 / ACCEL,
            self.az as f64 / ACCEL,
        ]
    }
    pub fn gyro_dps(&self) -> [f64; 3] {
        [
            self.gx as f64 / GYRO,
            self.gy as f64 / GYRO,
            self.gz as f64 / GYRO,
        ]
    }
    pub fn accel_mag_g(&self) -> f64 {
        let a = self.accel_g();
        (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
    }
}

pub fn raw_sample_line(s: &RawSample) -> String {
    let timestamp_us = s
        .timestamp_s
        .map(|v| (v * 1_000_000.0).round() as u64)
        .unwrap_or(0);
    format!(
        "RAW 0x{:02x}: accel=({}, {}, {}) temp_raw={} gyro=({}, {}, {}) timestamp_us={} sequence={}",
        s.address,
        s.ax,
        s.ay,
        s.az,
        s.temp_raw,
        s.gx,
        s.gy,
        s.gz,
        timestamp_us,
        s.sequence.unwrap_or(0)
    )
}

pub fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub fn encode_binary_frame(s: &RawSample) -> [u8; BINARY_FRAME_LEN] {
    let mut b = [0u8; BINARY_FRAME_LEN];
    b[0..2].copy_from_slice(&BINARY_FRAME_MAGIC);
    b[2] = BINARY_FRAME_VERSION;
    b[3] = BINARY_FRAME_PAYLOAD_LEN;
    b[4] = s.address as u8;
    b[6..14].copy_from_slice(&s.sequence.unwrap_or(0).to_le_bytes());
    let ts = s
        .timestamp_s
        .map(|v| (v * 1_000_000.0).round() as u64)
        .unwrap_or(0);
    b[14..22].copy_from_slice(&ts.to_le_bytes());
    for (off, v) in [
        (22, s.ax),
        (24, s.ay),
        (26, s.az),
        (28, s.temp_raw),
        (30, s.gx),
        (32, s.gy),
        (34, s.gz),
    ] {
        b[off..off + 2].copy_from_slice(&(v as i16).to_le_bytes());
    }
    let crc = crc16_ccitt_false(&b[..BINARY_FRAME_LEN - 2]);
    b[BINARY_FRAME_LEN - 2..].copy_from_slice(&crc.to_le_bytes());
    b
}

#[derive(Debug, PartialEq)]
pub enum BinaryDecodeEvent {
    Sample(RawSample),
    Warning(String),
}

#[derive(Default)]
pub struct BinaryFrameDecoder {
    buf: Vec<u8>,
    last_seq: Option<u64>,
}

impl BinaryFrameDecoder {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn push(&mut self, bytes: &[u8]) -> Vec<BinaryDecodeEvent> {
        self.buf.extend_from_slice(bytes);
        let mut ev = Vec::new();
        loop {
            let Some(pos) = self.buf.windows(2).position(|w| w == BINARY_FRAME_MAGIC) else {
                if !self.buf.is_empty() {
                    ev.push(BinaryDecodeEvent::Warning(format!(
                        "discarded {} byte(s) before magic",
                        self.buf.len()
                    )));
                    self.buf.clear();
                }
                break;
            };
            if pos > 0 {
                self.buf.drain(..pos);
                ev.push(BinaryDecodeEvent::Warning(format!(
                    "discarded {pos} byte(s) before magic"
                )));
            }
            if self.buf.len() < BINARY_FRAME_LEN {
                break;
            }
            if self.buf[2] != BINARY_FRAME_VERSION || self.buf[3] != BINARY_FRAME_PAYLOAD_LEN {
                ev.push(BinaryDecodeEvent::Warning(format!(
                    "bad header version={} length={}",
                    self.buf[2], self.buf[3]
                )));
                self.buf.drain(..1);
                continue;
            }
            let got = u16::from_le_bytes([
                self.buf[BINARY_FRAME_LEN - 2],
                self.buf[BINARY_FRAME_LEN - 1],
            ]);
            let want = crc16_ccitt_false(&self.buf[..BINARY_FRAME_LEN - 2]);
            if got != want {
                ev.push(BinaryDecodeEvent::Warning(format!(
                    "crc mismatch got=0x{got:04x} expected=0x{want:04x}"
                )));
                self.buf.drain(..1);
                continue;
            }
            let seq = u64::from_le_bytes(self.buf[6..14].try_into().unwrap());
            if let Some(prev) = self.last_seq
                && seq != prev.wrapping_add(1)
            {
                ev.push(BinaryDecodeEvent::Warning(format!(
                    "sequence gap previous={prev} current={seq}"
                )));
            }
            self.last_seq = Some(seq);
            let i16le = |o| i16::from_le_bytes([self.buf[o], self.buf[o + 1]]) as i32;
            let ts = u64::from_le_bytes(self.buf[14..22].try_into().unwrap());
            let address = self.buf[4] as i32;
            let ax = i16le(22);
            let ay = i16le(24);
            let az = i16le(26);
            let temp_raw = i16le(28);
            let gx = i16le(30);
            let gy = i16le(32);
            let gz = i16le(34);
            let gyro_all_minus_one = gx == -1 && gy == -1 && gz == -1;
            let has_sentinel = [ax, ay, az, temp_raw, gx, gy, gz]
                .iter()
                .any(|v| *v == i16::MAX as i32 || *v == i16::MIN as i32);
            if gyro_all_minus_one || has_sentinel {
                let reason = match (gyro_all_minus_one, has_sentinel) {
                    (true, true) => "gyro all -1; sentinel i16 min/max field",
                    (true, false) => "gyro all -1",
                    (false, true) => "sentinel i16 min/max field",
                    (false, false) => unreachable!(),
                };
                ev.push(BinaryDecodeEvent::Warning(format!(
                    "suspicious sample address=0x{address:02x} sequence={seq}: {reason}"
                )));
            }
            ev.push(BinaryDecodeEvent::Sample(RawSample {
                address,
                ax,
                ay,
                az,
                temp_raw,
                gx,
                gy,
                gz,
                timestamp_s: Some(ts as f64 / 1_000_000.0),
                sequence: Some(seq),
            }));
            self.buf.drain(..BINARY_FRAME_LEN);
        }
        ev
    }
}
fn raw_re() -> Regex {
    Regex::new(r"^RAW 0x(?P<addr>[0-9a-fA-F]{2}): accel=\((?P<ax>-?\d+), (?P<ay>-?\d+), (?P<az>-?\d+)\) temp_raw=(?P<temp>-?\d+) gyro=\((?P<gx>-?\d+), (?P<gy>-?\d+), (?P<gz>-?\d+)\)(?P<trailing>.*)$").unwrap()
}
pub fn parse_raw_line(line: &str) -> Option<RawSample> {
    let c = raw_re().captures(line)?;
    let mut timestamp_s = None;
    let mut sequence = None;
    for kv in c
        .name("trailing")
        .map(|m| m.as_str())
        .unwrap_or("")
        .split_whitespace()
    {
        let Some((key, value)) = kv.split_once('=') else {
            continue;
        };
        match key {
            "timestamp_s" => {
                if let Ok(v) = value.parse::<f64>()
                    && v.is_finite()
                {
                    timestamp_s = Some(v);
                }
            }
            "timestamp_us" | "ts_us" => {
                if let Ok(v) = value.parse::<f64>()
                    && v.is_finite()
                {
                    timestamp_s = Some(v / 1_000_000.0);
                }
            }
            "timestamp_ns" | "ts_ns" => {
                if let Ok(v) = value.parse::<f64>()
                    && v.is_finite()
                {
                    timestamp_s = Some(v / 1_000_000_000.0);
                }
            }
            "sequence" | "seq" => {
                if let Ok(v) = value.parse::<u64>() {
                    sequence = Some(v);
                }
            }
            _ => {}
        }
    }
    Some(RawSample {
        address: i32::from_str_radix(&c["addr"], 16).ok()?,
        ax: c["ax"].parse().ok()?,
        ay: c["ay"].parse().ok()?,
        az: c["az"].parse().ok()?,
        temp_raw: c["temp"].parse().ok()?,
        gx: c["gx"].parse().ok()?,
        gy: c["gy"].parse().ok()?,
        gz: c["gz"].parse().ok()?,
        timestamp_s,
        sequence,
    })
}
#[allow(clippy::type_complexity)]
pub fn parse_log(path: &Path) -> std::io::Result<(BTreeMap<String, Vec<String>>, Vec<RawSample>)> {
    let text = fs::read_to_string(path)?;
    let kv_re = Regex::new(r"^([a-zA-Z0-9_]+)=(.+)$").unwrap();
    let mut kv = BTreeMap::new();
    let mut samples = Vec::new();
    let mut summary = false;
    for l in text.lines() {
        let line = l.trim();
        match line {
            "verification_summary_begin" => {
                summary = true;
                continue;
            }
            "verification_summary_end" => {
                summary = false;
                continue;
            }
            _ => {}
        }
        if let Some(s) = parse_raw_line(line) {
            samples.push(s);
            continue;
        }
        if summary && let Some(c) = kv_re.captures(line) {
            kv.entry(c[1].to_string())
                .or_insert_with(Vec::new)
                .push(c[2].to_string())
        }
    }
    Ok((kv, samples))
}
fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        f64::NAN
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}
fn stdev(v: &[f64]) -> f64 {
    if v.len() < 2 {
        0.0
    } else {
        let m = mean(v);
        (v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (v.len() - 1) as f64).sqrt()
    }
}
pub fn classify_orientation_sample(
    s: &RawSample,
    mag_min: f64,
    mag_max: f64,
    dom: f64,
) -> (Option<&'static str>, Option<&'static str>) {
    let a = s.accel_g();
    let mag = s.accel_mag_g();
    if !(mag_min..=mag_max).contains(&mag) {
        return (None, Some("magnitude"));
    }
    let opts = [
        (a[0].abs(), if a[0] >= 0.0 { "+X" } else { "-X" }),
        (a[1].abs(), if a[1] >= 0.0 { "+Y" } else { "-Y" }),
        (a[2].abs(), if a[2] >= 0.0 { "+Z" } else { "-Z" }),
    ];
    let best = opts
        .into_iter()
        .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
        .unwrap();
    if best.0 < dom {
        (None, Some("dominance"))
    } else {
        (Some(best.1), None)
    }
}
pub fn orientation_analyze(
    path: &Path,
    min: usize,
    mag_min: f64,
    mag_max: f64,
    dom: f64,
) -> std::io::Result<i32> {
    let (_, samples) = parse_log(path)?;
    let mut bins: BTreeMap<&str, Vec<f64>> = FACES.into_iter().map(|f| (f, Vec::new())).collect();
    let (mut rm, mut rd) = (0, 0);
    for s in &samples {
        match classify_orientation_sample(s, mag_min, mag_max, dom) {
            (Some(a), _) => bins.get_mut(a).unwrap().push(s.accel_mag_g()),
            (_, Some("magnitude")) => rm += 1,
            _ => rd += 1,
        }
    }
    println!(
        "auto_orientation_report_begin\nlog={}\nsamples={}\nrejected_by_magnitude={}\nrejected_by_dominance={}",
        path.display(),
        samples.len(),
        rm,
        rd
    );
    let mut missing = Vec::new();
    for f in FACES {
        let v = &bins[f];
        if v.len() < min {
            missing.push(f)
        };
        println!(
            "axis={} samples={} mean_mag_g={:.4} std_mag_g={:.4} passed={}",
            f,
            v.len(),
            mean(v),
            stdev(v),
            v.len() >= min
        )
    }
    println!(
        "missing_axes={}\nauto_orientation_coverage_passed={}\nauto_orientation_report_end",
        if missing.is_empty() {
            "none".into()
        } else {
            missing.join(",")
        },
        missing.is_empty()
    );
    Ok(if missing.is_empty() { 0 } else { 1 })
}
fn parse_sixface(path: &Path) -> std::io::Result<BTreeMap<String, Vec<RawSample>>> {
    let mut m: BTreeMap<String, Vec<RawSample>> =
        FACES.iter().map(|f| (f.to_string(), Vec::new())).collect();
    let mut cur = None;
    for l in fs::read_to_string(path)?.lines() {
        let line = l.trim();
        if let Some(f) = line.strip_prefix("# face_begin ") {
            cur = Some(f.to_string());
            continue;
        }
        if line.starts_with("# face_end ") {
            cur = None;
            continue;
        }
        if let (Some(f), Some(s)) = (cur.as_ref(), parse_raw_line(line))
            && let Some(v) = m.get_mut(f)
        {
            v.push(s)
        }
    }
    Ok(m)
}

pub fn load_sixface_mapping(path: &Path) -> std::io::Result<BTreeMap<String, String>> {
    let value: Value = serde_json::from_str(&fs::read_to_string(path)?)?;
    let mapping = value.get("faces").unwrap_or(&value);
    let obj = mapping.as_object().ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            "mapping JSON must be an object or contain a 'faces' object",
        )
    })?;
    let allowed: BTreeSet<&str> = FACES.into_iter().collect();
    let mut out = BTreeMap::new();
    for (face, axis_value) in obj {
        let axis = axis_value.as_str().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "mapping axis values must be strings",
            )
        })?;
        if !allowed.contains(face.as_str()) || !allowed.contains(axis) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid face mapping {face} -> {axis}"),
            ));
        }
        out.insert(face.clone(), axis.to_string());
    }
    Ok(out)
}

pub fn sixface_analyze(
    path: &Path,
    min: usize,
    mapping_path: Option<&Path>,
) -> std::io::Result<i32> {
    let faces = parse_sixface(path)?;
    let mapping = mapping_path.map(load_sixface_mapping).transpose()?;
    let mut passed = 0;
    let mut mapped_passed = 0;
    let mut dom_axes = BTreeSet::new();
    let mut signed_axes = BTreeSet::new();
    println!(
        "sixface_report_begin\nlog={}\nmapping={}",
        path.display(),
        mapping_path
            .map(|p| p.display().to_string())
            .unwrap_or_else(|| "none".into())
    );
    for (face, s) in faces {
        if s.is_empty() {
            let expected = mapping
                .as_ref()
                .and_then(|m| m.get(&face))
                .map_or("unmapped", String::as_str);
            println!(
                "face={face} samples=0 expected_signed_axis={expected} coverage_passed=false mapped_passed=false"
            );
            continue;
        }
        let xs: Vec<_> = s.iter().map(|x| x.accel_g()[0]).collect();
        let ys: Vec<_> = s.iter().map(|x| x.accel_g()[1]).collect();
        let zs: Vec<_> = s.iter().map(|x| x.accel_g()[2]).collect();
        let (mx, my, mz) = (mean(&xs), mean(&ys), mean(&zs));
        let vals = [
            (mx.abs(), "X", mx),
            (my.abs(), "Y", my),
            (mz.abs(), "Z", mz),
        ];
        let b = vals
            .into_iter()
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
            .unwrap();
        let signed = format!("{}{}", if b.2 >= 0.0 { "+" } else { "-" }, b.1);
        dom_axes.insert(b.1.to_string());
        signed_axes.insert(signed.clone());
        let mag = (mx * mx + my * my + mz * mz).sqrt();
        let cross = vals
            .iter()
            .filter(|(_, axis, _)| *axis != b.1)
            .map(|(v, _, _)| *v)
            .fold(0.0, f64::max);
        let ok = s.len() >= min && (0.75..=1.25).contains(&mag);
        let expected = mapping.as_ref().and_then(|m| m.get(&face));
        let mapped_ok = ok && expected.is_none_or(|e| e == &signed);
        if ok {
            passed += 1
        }
        if mapped_ok {
            mapped_passed += 1
        }
        println!(
            "face={} samples={} mean_g=({:.4},{:.4},{:.4}) mag_g={:.4} dominant_axis={} dominant_signed_axis={} expected_signed_axis={} dominant_abs_g={:.4} cross_axis_max_g={:.4} coverage_passed={} mapped_passed={}",
            face,
            s.len(),
            mx,
            my,
            mz,
            mag,
            b.1,
            signed,
            expected.map_or("unmapped", String::as_str),
            b.0,
            cross,
            ok,
            mapped_ok
        )
    }
    let ok = passed == 6 && dom_axes.len() >= 3 && signed_axes.len() >= 6;
    let mapped_all_ok = mapping.is_some() && mapped_passed == 6;
    println!(
        "sixface_passed_faces={}\nsixface_mapped_passed_faces={}\nsixface_distinct_dominant_axes={}\nsixface_distinct_signed_axes={}\nsixface_signed_axes={}\nsixface_coverage_plausible={}\nsixface_mapping_certified={}",
        passed,
        mapping
            .as_ref()
            .map(|_| mapped_passed.to_string())
            .unwrap_or_else(|| "not_applicable".into()),
        dom_axes.len(),
        signed_axes.len(),
        if signed_axes.is_empty() {
            "none".into()
        } else {
            signed_axes.into_iter().collect::<Vec<_>>().join(",")
        },
        ok,
        if mapping.is_some() {
            mapped_all_ok.to_string()
        } else {
            "not_applicable".into()
        }
    );
    if !ok {
        println!(
            "sixface_note=all six faces should pass magnitude and cover +X,-X,+Y,-Y,+Z,-Z dominant signed axes; if physical face labels are unknown, repeat with clearer orientation marks"
        );
    }
    if mapping.is_some() && !mapped_all_ok {
        println!(
            "sixface_mapping_note=coverage may pass while board-face certification fails; update mapping only if your fixture labels were wrong, not to hide bad data"
        );
    }
    println!("sixface_report_end");
    let passed = if mapping.is_some() { mapped_all_ok } else { ok };
    Ok(if passed { 0 } else { 1 })
}
pub fn export_csv(log: &Path, out: &Path, rate: f64) -> std::io::Result<i32> {
    let (_, s) = parse_log(log)?;
    if let Some(p) = out.parent() {
        fs::create_dir_all(p)?
    }
    let mut f = fs::File::create(out)?;
    writeln!(
        f,
        "sample,t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_raw,timestamp_s_measured,sequence"
    )?;
    for (i, x) in s.iter().enumerate() {
        let a = x.accel_g();
        let g = x.gyro_dps();
        writeln!(
            f,
            "{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{}",
            i,
            i as f64 / rate,
            a[0],
            a[1],
            a[2],
            g[0],
            g[1],
            g[2],
            x.temp_raw,
            x.timestamp_s.map(|v| format!("{v:.9}")).unwrap_or_default(),
            x.sequence.map(|v| v.to_string()).unwrap_or_default()
        )?
    }
    println!(
        "exported_csv={} samples={} sample_rate_hz={}",
        out.display(),
        s.len(),
        rate
    );
    Ok(if s.is_empty() { 1 } else { 0 })
}
fn read_cols(p: &Path) -> std::io::Result<BTreeMap<String, Vec<f64>>> {
    let text = fs::read_to_string(p)?;
    let headers: Vec<_> = text
        .lines()
        .next()
        .unwrap_or("")
        .split(',')
        .map(str::to_string)
        .collect();
    let mut m: BTreeMap<String, Vec<f64>> =
        headers.iter().map(|h| (h.clone(), Vec::new())).collect();
    for l in text.lines().skip(1) {
        for (h, v) in headers.iter().zip(l.split(',')) {
            m.get_mut(h).unwrap().push(v.parse().unwrap_or(0.0))
        }
    }
    Ok(m)
}
fn allan_sizes(n: usize) -> Vec<usize> {
    let mut r = Vec::new();
    let mut m = 1;
    while 2 * m < n {
        r.push(m);
        m = std::cmp::max(m + 1, (m as f64 * 1.5) as usize)
    }
    r
}
fn adev(v: &[f64], m: usize) -> f64 {
    if v.len() < 2 * m + 1 {
        return f64::NAN;
    }
    let mut p = vec![0.0];
    for x in v {
        p.push(p.last().unwrap() + x)
    }
    let avg = |i: usize| (p[i + m] - p[i]) / m as f64;
    let mut t = 0.0;
    let mut c = 0;
    for i in 0..=v.len() - 2 * m {
        let d = avg(i + m) - avg(i);
        t += d * d;
        c += 1
    }
    (t / (2.0 * c as f64)).sqrt()
}
pub fn allan_analyze(csv: &Path, rate: f64, out: &Path) -> std::io::Result<i32> {
    let c = read_cols(csv)?;
    if let Some(p) = out.parent() {
        fs::create_dir_all(p)?
    }
    let n = c.get("t_s").map_or(0, Vec::len);
    let sizes = allan_sizes(n);
    let mut f = fs::File::create(out)?;
    writeln!(f, "axis,m,tau_s,allan_deviation")?;
    for ax in ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"] {
        for m in &sizes {
            writeln!(
                f,
                "{},{},{:.9},{:.12}",
                ax,
                m,
                *m as f64 / rate,
                adev(&c[ax], *m)
            )?
        }
    }
    println!(
        "allan_csv={} samples={} sample_rate_hz={} taus={}",
        out.display(),
        n,
        rate,
        sizes.len()
    );
    Ok(if n >= 10 && !sizes.is_empty() { 0 } else { 1 })
}
fn period(v: &[f64], k: usize, rate: f64) -> f64 {
    let n = v.len();
    let (mut re, mut im) = (0.0, 0.0);
    for (i, x) in v.iter().enumerate() {
        let a = -2.0 * std::f64::consts::PI * k as f64 * i as f64 / n as f64;
        re += x * a.cos();
        im += x * a.sin()
    }
    (re * re + im * im) / (rate * n as f64)
}
pub fn psd_analyze(csv: &Path, rate: f64, out: &Path) -> std::io::Result<i32> {
    let c = read_cols(csv)?;
    if let Some(p) = out.parent() {
        fs::create_dir_all(p)?
    }
    let n = c.get("t_s").map_or(0, Vec::len);
    let mut f = fs::File::create(out)?;
    writeln!(f, "axis,frequency_hz,psd")?;
    for ax in ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"] {
        let mut v = c[ax].clone();
        let m = mean(&v);
        for x in &mut v {
            *x -= m
        }
        for k in 1..=n / 2 {
            writeln!(
                f,
                "{},{:.9},{:.12}",
                ax,
                k as f64 * rate / n as f64,
                period(&v, k, rate)
            )?
        }
    }
    println!(
        "psd_csv={} samples={} sample_rate_hz={}",
        out.display(),
        n,
        rate
    );
    Ok(if n >= 10 { 0 } else { 1 })
}
pub fn sixface_calibration(log: &Path, out: &Path) -> std::io::Result<i32> {
    let faces = parse_sixface(log)?;
    if let Some(p) = out.parent() {
        fs::create_dir_all(p)?
    }
    let mut spec_samples = Vec::new();
    for (face, s) in faces {
        let face = match face.as_str() {
            "+X" => imu_validation::spec::SignedAxis::PosX,
            "-X" => imu_validation::spec::SignedAxis::NegX,
            "+Y" => imu_validation::spec::SignedAxis::PosY,
            "-Y" => imu_validation::spec::SignedAxis::NegY,
            "+Z" => imu_validation::spec::SignedAxis::PosZ,
            "-Z" => imu_validation::spec::SignedAxis::NegZ,
            _ => continue,
        };
        spec_samples.push(imu_validation::spec::FaceAccelSamples {
            face,
            accel_g: s.iter().map(RawSample::accel_g).collect(),
        });
    }
    let report = imu_validation::spec::analyze_sixface_accel(
        &spec_samples,
        &imu_validation::spec::SixFaceAccelConfig {
            min_samples_per_face: 5,
            reference_gravity_g: 1.0,
            ..Default::default()
        },
    );
    fs::write(out, serde_json::to_string_pretty(&report)? + "\n")?;
    let passed = report.overall_status == imu_validation::VerdictStatus::Pass;
    println!(
        "sixface_calibration_json={} sixface_accel_status={:?}",
        out.display(),
        report.overall_status
    );
    Ok(if passed { 0 } else { 1 })
}
pub fn analyze(
    path: &Path,
    min_samples: usize,
    _min_stationary: usize,
    addr: i32,
    expected_identity: ExpectedIdentity,
) -> std::io::Result<i32> {
    let (kv, s) = parse_log(path)?;
    let address_ok = kv.get("bus_address").is_some_and(|v| {
        v.iter()
            .any(|x| x.eq_ignore_ascii_case(&format!("0x{addr:02x}")))
    });
    let identity_observed = kv.get("who_am_i").and_then(|v| {
        v.iter().rev().find_map(|x| {
            let x = x.trim();
            let x = x.strip_prefix("0x").unwrap_or(x);
            u8::from_str_radix(x, 16).ok()
        })
    });
    let who_ok = kv.get("who_am_i").is_some_and(|v| {
        v.iter().any(|x| {
            let x = x.trim();
            let x = x.strip_prefix("0x").unwrap_or(x);
            u8::from_str_radix(x, 16)
                .ok()
                .is_some_and(|who| expected_identity.matches(who))
        })
    });
    let identity_profile = match identity_observed {
        Some(0x68) => "classic_mpu6050",
        Some(0x70) => "mpu6500_compatible_or_clone",
        Some(_) => "unknown",
        None => "missing",
    };
    let identity_observed_text = identity_observed
        .map(|who| format!("0x{who:02x}"))
        .unwrap_or_else(|| "missing".into());
    let pwr_ok = kv
        .get("pwr_mgmt_1")
        .is_some_and(|v| v.last().is_some_and(|x| x != "unreadable"));
    println!(
        "validation_report_begin\nlog={}\nsamples={}\nbus_address_values={}\nwho_am_i_values={}\nidentity_observed={}\nidentity_profile={}\nclaim=not_genuine_proof\nhost_validation_score={}\nvalidation_report_end",
        path.display(),
        s.len(),
        kv.get("bus_address")
            .map(|v| v.join(","))
            .unwrap_or_else(|| "missing".into()),
        kv.get("who_am_i")
            .map(|v| v.join(","))
            .unwrap_or_else(|| "missing".into()),
        identity_observed_text,
        identity_profile,
        if address_ok && who_ok && pwr_ok && s.len() >= min_samples {
            10
        } else {
            0
        }
    );
    Ok(
        if address_ok && who_ok && pwr_ok && s.len() >= min_samples {
            0
        } else {
            1
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;
    #[test]
    fn parses_fixture() {
        let (_, s) = parse_log(Path::new("tests/fixtures/stationary-60s.log")).unwrap();
        assert!(s.len() > 20);
        assert_eq!(s[0].address, 0x68);
        assert!(s[0].accel_mag_g() > 0.5)
    }
    #[test]
    fn stationary_fixture_generates_validation_report() {
        let (_, s) = parse_log(Path::new("tests/fixtures/stationary-60s.log")).unwrap();
        let samples: Vec<_> = s
            .iter()
            .map(|x| imu_validation::ImuSample {
                accel_g: x.accel_g(),
                gyro_dps: x.gyro_dps(),
                timestamp_s: x.timestamp_s,
                sequence: x.sequence,
            })
            .collect();
        let report = imu_validation::analyze_stationary(
            &samples,
            &imu_validation::StationaryConfig {
                nominal_rate_hz: Some(10.0),
                min_samples: 20,
                ..Default::default()
            },
        );
        assert_eq!(report.sample_count, s.len());
        assert_eq!(
            report.timing_quality.status,
            imu_validation::VerdictStatus::Unavailable
        );
        assert!(
            serde_json::to_string(&report)
                .unwrap()
                .contains("accel_norm")
        );
        let calibration = imu_validation::estimate_gyro_bias_calibration(
            &samples,
            &imu_validation::GyroBiasCalibrationConfig::default(),
        );
        let calibration_json = serde_json::to_value(&calibration).unwrap();
        assert_eq!(calibration_json["source"], "stationary_zero_rate");
        assert!(calibration_json.get("publication_grade").is_some());
    }

    #[test]
    fn raw_line_old_format_has_no_measured_timing() {
        let s =
            parse_raw_line("RAW 0x68: accel=(1, -2, 16384) temp_raw=42 gyro=(3, -4, 5)").unwrap();
        assert_eq!(s.address, 0x68);
        assert_eq!(s.timestamp_s, None);
        assert_eq!(s.sequence, None);
    }

    #[test]
    fn raw_line_parses_timestamp_us_and_sequence() {
        let s = parse_raw_line(
            "RAW 0x68: accel=(1, 2, 3) temp_raw=4 gyro=(5, 6, 7) timestamp_us=1234567 sequence=42",
        )
        .unwrap();
        assert!((s.timestamp_s.unwrap() - 1.234567).abs() < 1e-12);
        assert_eq!(s.sequence, Some(42));
    }

    #[test]
    fn raw_line_malformed_optional_kvs_do_not_break_core_parse() {
        let s = parse_raw_line(
            "RAW 0x68: accel=(1, 2, 3) temp_raw=4 gyro=(5, 6, 7) timestamp_us=oops seq=-1 unknown=ok",
        )
        .unwrap();
        assert_eq!(s.gx, 5);
        assert_eq!(s.timestamp_s, None);
        assert_eq!(s.sequence, None);
    }

    #[test]
    fn raw_integrity_event_parser_accepts_outcome_with_extra_fields() {
        let event = parse_raw_integrity_event(
            "raw_integrity_event seq=28 outcome=recovered retries=1 address=0x68",
        )
        .unwrap();
        assert_eq!(event.outcome, "recovered");

        assert_eq!(parse_raw_integrity_event("RAW 0x68: accel=(1, 2, 3)"), None);
        assert_eq!(
            parse_raw_integrity_event("raw_integrity_event seq=28"),
            None
        );
    }

    #[test]
    fn raw_integrity_event_parser_normalizes_outcome_and_ignores_noise() {
        let event = parse_raw_integrity_event(
            "raw_integrity_event reason=GyroAllMinusOne outcome=RECOVERED retries=1",
        )
        .unwrap();
        assert_eq!(event.outcome, "recovered");

        assert_eq!(
            parse_raw_integrity_event("raw_integrity_event_extra outcome=recovered"),
            None
        );
        assert_eq!(
            parse_raw_integrity_event("raw_integrity_event outcome="),
            None
        );
        assert_eq!(
            parse_raw_integrity_event("raw_integrity_event outcome"),
            None
        );
    }

    #[test]
    fn integrity_stats_counts_samples_events_and_summary() {
        let mut stats = IntegrityStats::default();
        let mut line_buf = String::new();
        stats.record_text(
            "RAW 0x68: accel=(1, 2, 3) temp_raw=4 gyro=(5, 6, 7)\nraw_integrity_event seq=1 outcome=recovered retries=1\nraw_integrity_event seq=2 outcome=rejected\n",
            &mut line_buf,
        );
        stats.record_line("raw_integrity_event seq=3 outcome=retry_error");
        stats.record_line("raw_integrity_event seq=4 outcome=accepted");
        stats.record_sample();

        assert_eq!(stats.total, 2);
        assert_eq!(stats.clean(), 0);
        assert_eq!(stats.suspicious(), 4);
        assert_eq!(
            stats.summary_line(),
            "integrity_stats samples=2 clean_samples=0 suspicious_events=4 recovered=1 rejected=1 retry_error=1 accepted=1"
        );
    }

    #[test]
    fn integrity_stats_buffers_partial_lines_until_newline() {
        let mut stats = IntegrityStats::default();
        let mut line_buf = String::new();

        stats.record_text(
            "RAW 0x68: accel=(1, 2, 3) temp_raw=4 gyro=(5, 6, 7)",
            &mut line_buf,
        );
        assert_eq!(stats.total, 0);
        assert!(!line_buf.is_empty());

        stats.record_text(
            " timestamp_us=100 sequence=7 timestamp_source=device_instant\r\nraw_integrity_event seq=7 outcome=recovered retries=1\r\n",
            &mut line_buf,
        );
        assert_eq!(line_buf, "");
        assert_eq!(stats.total, 1);
        assert_eq!(stats.recovered, 1);
        assert_eq!(stats.clean(), 0);
    }

    #[test]
    fn integrity_stats_counts_trimmed_final_partial_line() {
        let mut stats = IntegrityStats::default();
        let line_buf = "RAW 0x68: accel=(1, 2, 3) temp_raw=4 gyro=(5, 6, 7)   ";

        record_partial_text_line(&mut stats, line_buf);

        assert_eq!(stats.total, 1);
        assert_eq!(stats.clean(), 1);
    }

    #[test]
    fn integrity_stats_ignores_unknown_outcomes_and_saturates_clean() {
        let mut stats = IntegrityStats::default();
        stats.record_sample();
        stats.record_line("raw_integrity_event seq=1 outcome=recovered retries=1");
        stats.record_line("raw_integrity_event seq=2 outcome=rejected retries=1");
        stats.record_line("raw_integrity_event seq=3 outcome=weird retries=1");

        assert_eq!(stats.total, 1);
        assert_eq!(stats.suspicious(), 2);
        assert_eq!(stats.clean(), 0);
        assert_eq!(
            stats.summary_line(),
            "integrity_stats samples=1 clean_samples=0 suspicious_events=2 recovered=1 rejected=1 retry_error=0 accepted=0"
        );
    }

    #[test]
    fn integrity_stats_rejected_and_retry_error_do_not_reduce_clean_samples() {
        let mut stats = IntegrityStats::default();
        stats.record_sample();
        stats.record_sample();
        stats.record_line("raw_integrity_event seq=1 outcome=rejected retries=1");
        stats.record_line("raw_integrity_event seq=2 outcome=retry_error retries=1");

        assert_eq!(stats.suspicious(), 2);
        assert_eq!(stats.clean(), 2);
        assert_eq!(
            stats.summary_line(),
            "integrity_stats samples=2 clean_samples=2 suspicious_events=2 recovered=0 rejected=1 retry_error=1 accepted=0"
        );
    }

    #[test]
    fn binary_frame_round_trips_raw_sample() {
        let s = RawSample {
            address: 0x68,
            ax: 1,
            ay: -2,
            az: 16384,
            temp_raw: 99,
            gx: -4,
            gy: 5,
            gz: -6,
            timestamp_s: Some(1.25),
            sequence: Some(7),
        };
        let mut d = BinaryFrameDecoder::new();
        let ev = d.push(&encode_binary_frame(&s));
        assert_eq!(ev, vec![BinaryDecodeEvent::Sample(s)]);
    }

    #[test]
    fn regression_binary_decoder_warns_on_crc_valid_gyro_all_minus_one() {
        let s = RawSample {
            address: 0x68,
            ax: 1,
            ay: 2,
            az: 3,
            temp_raw: 25,
            gx: -1,
            gy: -1,
            gz: -1,
            timestamp_s: Some(0.001),
            sequence: Some(42),
        };
        let ev = BinaryFrameDecoder::new().push(&encode_binary_frame(&s));
        assert!(
            ev.iter().any(|e| matches!(
                e,
                BinaryDecodeEvent::Warning(w)
                    if w.contains("suspicious")
                        && w.contains("address=0x68")
                        && w.contains("sequence=42")
            )),
            "expected suspicious warning with address and sequence, got {ev:?}"
        );
        assert!(
            ev.iter()
                .any(|e| matches!(e, BinaryDecodeEvent::Sample(sample) if sample == &s))
        );
    }

    #[test]
    fn regression_binary_decoder_warns_on_crc_valid_i16_sentinel_field() {
        let s = RawSample {
            address: 0x69,
            ax: i16::MAX as i32,
            ay: 2,
            az: 3,
            temp_raw: 25,
            gx: 4,
            gy: i16::MIN as i32,
            gz: 6,
            timestamp_s: Some(0.002),
            sequence: Some(43),
        };
        let ev = BinaryFrameDecoder::new().push(&encode_binary_frame(&s));
        assert!(
            ev.iter().any(|e| matches!(
                e,
                BinaryDecodeEvent::Warning(w)
                    if w.contains("suspicious")
                        && w.contains("address=0x69")
                        && w.contains("sequence=43")
            )),
            "expected suspicious warning with address and sequence, got {ev:?}"
        );
        assert!(
            ev.iter()
                .any(|e| matches!(e, BinaryDecodeEvent::Sample(sample) if sample == &s))
        );
    }

    #[test]
    fn binary_decoder_reports_crc_corruption_and_resyncs() {
        let s = RawSample {
            address: 0x68,
            ax: 1,
            ay: 2,
            az: 3,
            temp_raw: 4,
            gx: 5,
            gy: 6,
            gz: 7,
            timestamp_s: Some(0.001),
            sequence: Some(0),
        };
        let mut bad = encode_binary_frame(&s);
        bad[22] ^= 0x55;
        let good = encode_binary_frame(&s);
        let mut bytes = Vec::from(b"junk".as_slice());
        bytes.extend_from_slice(&bad);
        bytes.extend_from_slice(&good);
        let ev = BinaryFrameDecoder::new().push(&bytes);
        assert!(
            ev.iter()
                .any(|e| matches!(e, BinaryDecodeEvent::Warning(w) if w.contains("discarded")))
        );
        assert!(
            ev.iter()
                .any(|e| matches!(e, BinaryDecodeEvent::Warning(w) if w.contains("crc mismatch")))
        );
        assert!(ev.iter().any(|e| matches!(e, BinaryDecodeEvent::Sample(_))));
    }

    #[test]
    fn binary_decoder_reports_sequence_gap() {
        let mut d = BinaryFrameDecoder::new();
        let a = sample_with_timing(Some(0.0), Some(1));
        let b = sample_with_timing(Some(0.1), Some(3));
        assert!(matches!(
            d.push(&encode_binary_frame(&a)).as_slice(),
            [BinaryDecodeEvent::Sample(_)]
        ));
        let ev = d.push(&encode_binary_frame(&b));
        assert!(
            ev.iter()
                .any(|e| matches!(e, BinaryDecodeEvent::Warning(w) if w.contains("sequence gap")))
        );
    }

    #[test]
    fn parsed_timestamps_populate_stationary_timing_quality() {
        let raw: Vec<_> = (0..3)
            .map(|i| {
                parse_raw_line(&format!(
                    "RAW 0x68: accel=(0, 0, 16384) temp_raw=0 gyro=(0, 0, 0) timestamp_us={} sequence={}",
                    i * 100_000,
                    i
                ))
                .unwrap()
            })
            .collect();
        let samples: Vec<_> = raw
            .iter()
            .map(|x| imu_validation::ImuSample {
                accel_g: x.accel_g(),
                gyro_dps: x.gyro_dps(),
                timestamp_s: x.timestamp_s,
                sequence: x.sequence,
            })
            .collect();
        let report = imu_validation::analyze_stationary(&samples, &Default::default());
        assert_eq!(
            report.timing_quality.status,
            imu_validation::VerdictStatus::Pass
        );
        assert!(report.timing_quality.timestamp_present);
        assert!(report.timing_quality.sequence_present);
    }

    #[test]
    fn csv_header_preserves_prefix_and_appends_measured_fields() {
        let tmp = std::env::temp_dir().join(format!("imu-tool-csv-test-{}", std::process::id()));
        fs::create_dir_all(&tmp).unwrap();
        let log = tmp.join("raw.log");
        let csv = tmp.join("samples.csv");
        fs::write(
            &log,
            "RAW 0x68: accel=(0, 0, 16384) temp_raw=0 gyro=(0, 0, 0) timestamp_us=1000 seq=7\n",
        )
        .unwrap();
        assert_eq!(export_csv(&log, &csv, 10.0).unwrap(), 0);
        let text = fs::read_to_string(&csv).unwrap();
        let header = text.lines().next().unwrap();
        assert!(header.starts_with("sample,t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_raw"));
        assert!(header.ends_with(",timestamp_s_measured,sequence"));
        assert!(text.lines().nth(1).unwrap().ends_with(",0.001000000,7"));
        let _ = fs::remove_dir_all(tmp);
    }

    #[test]
    fn monitor_deadline_is_absent_when_duration_is_unbounded() {
        assert!(monitor_deadline(None, Instant::now()).is_none());
        assert!(before_monitor_deadline(None, Instant::now()));
    }

    #[test]
    fn monitor_deadline_uses_positive_duration() {
        let now = Instant::now();
        let deadline = monitor_deadline(Some(1.5), now).unwrap();

        assert_eq!(deadline.duration_since(now), Duration::from_secs_f64(1.5));
        assert!(before_monitor_deadline(Some(deadline), now));
        assert!(!before_monitor_deadline(Some(deadline), deadline));
        assert!(!before_monitor_deadline(
            Some(deadline),
            deadline + Duration::from_nanos(1)
        ));
    }

    #[test]
    fn monitor_deadline_clamps_negative_duration_to_immediate_expiry() {
        let now = Instant::now();
        let deadline = monitor_deadline(Some(-2.0), now).unwrap();

        assert_eq!(deadline, now);
        assert!(!before_monitor_deadline(Some(deadline), now));
    }

    fn sample_with_timing(timestamp_s: Option<f64>, sequence: Option<u64>) -> RawSample {
        RawSample {
            address: 0x68,
            ax: 0,
            ay: 0,
            az: 16384,
            temp_raw: 0,
            gx: 0,
            gy: 0,
            gz: 0,
            timestamp_s,
            sequence,
        }
    }

    fn imu_samples(samples: &[RawSample]) -> Vec<imu_validation::ImuSample> {
        samples
            .iter()
            .map(|s| imu_validation::ImuSample {
                accel_g: s.accel_g(),
                gyro_dps: s.gyro_dps(),
                timestamp_s: s.timestamp_s,
                sequence: s.sequence,
            })
            .collect()
    }

    fn timing_decision_json(samples: &[RawSample], nominal_rate_hz: f64) -> Value {
        serde_json::to_value(imu_validation::noise::decide_noise_timing(
            &imu_samples(samples),
            &imu_validation::noise::NoiseTimingConfig {
                nominal_rate_hz: Some(nominal_rate_hz),
                observed_rate_tolerance_fraction: 0.05,
                jitter_ratio_max: 0.05,
            },
        ))
        .unwrap()
    }

    fn noise_report_json(
        samples: &[RawSample],
        nominal_rate_hz: f64,
        psd_band_hz: Option<[f64; 2]>,
    ) -> Value {
        serde_json::to_value(imu_validation::noise::analyze_imu_noise(
            &imu_samples(samples),
            &imu_validation::noise::ImuNoiseReportConfig {
                timing: imu_validation::noise::NoiseTimingConfig {
                    nominal_rate_hz: Some(nominal_rate_hz),
                    observed_rate_tolerance_fraction: 0.05,
                    jitter_ratio_max: 0.05,
                },
                psd_band_hz,
            },
        ))
        .unwrap()
    }

    #[test]
    fn timing_decision_old_logs_use_nominal_rate() {
        let samples = vec![sample_with_timing(None, None); 5];
        let decision = timing_decision_json(&samples, 10.0);
        assert_eq!(decision["timing_source"], "nominal_rate_assumed");
        assert_eq!(decision["tau0_s"], 0.1);
        assert_eq!(decision["reason"], "missing_or_partial_timestamps");
    }

    #[test]
    fn timing_decision_low_jitter_complete_timestamps_are_trusted() {
        let samples: Vec<_> = (0..6)
            .map(|i| sample_with_timing(Some(i as f64 * 0.1), Some(i)))
            .collect();
        let decision = timing_decision_json(&samples, 10.0);
        assert_eq!(decision["timing_source"], "trusted_timestamps");
        assert_eq!(decision["reason"], "trusted_timestamps");
        assert_eq!(decision["sequence_gaps"], 0);
        assert_eq!(decision["sequence_decreases_or_duplicates"], 0);
    }

    #[test]
    fn timing_decision_partial_timestamps_fallback_nominal() {
        let samples = vec![
            sample_with_timing(Some(0.0), None),
            sample_with_timing(None, None),
            sample_with_timing(Some(0.2), None),
        ];
        let decision = timing_decision_json(&samples, 10.0);
        assert_eq!(decision["timing_source"], "nominal_rate_assumed");
        assert_eq!(decision["reason"], "missing_or_partial_timestamps");
    }

    #[test]
    fn timing_decision_partial_sequence_coverage_fallback_nominal() {
        let samples = vec![
            sample_with_timing(Some(0.0), Some(0)),
            sample_with_timing(Some(0.1), None),
            sample_with_timing(Some(0.2), Some(2)),
        ];
        let decision = timing_decision_json(&samples, 10.0);
        assert_eq!(decision["timing_source"], "nominal_rate_assumed");
        assert_eq!(decision["reason"], "partial_sequence_coverage");
        assert_eq!(decision["sequence_samples_present"], 2);
        assert_eq!(decision["sequence_gaps"], Value::Null);
        assert_eq!(decision["sequence_decreases_or_duplicates"], Value::Null);
    }

    #[test]
    fn timing_decision_sequence_gap_or_decrease_fallback_nominal() {
        let gap: Vec<_> = [0, 1, 3]
            .into_iter()
            .enumerate()
            .map(|(i, seq)| sample_with_timing(Some(i as f64 * 0.1), Some(seq)))
            .collect();
        let decision = timing_decision_json(&gap, 10.0);
        assert_eq!(decision["timing_source"], "nominal_rate_assumed");
        assert_eq!(decision["reason"], "sequence_gap_decrease_or_duplicate");
        assert_eq!(decision["sequence_gaps"], 1);

        let dup: Vec<_> = [0, 1, 1]
            .into_iter()
            .enumerate()
            .map(|(i, seq)| sample_with_timing(Some(i as f64 * 0.1), Some(seq)))
            .collect();
        let decision = timing_decision_json(&dup, 10.0);
        assert_eq!(decision["timing_source"], "nominal_rate_assumed");
        assert_eq!(decision["sequence_decreases_or_duplicates"], 1);
    }

    #[test]
    fn timing_decision_invalid_nominal_and_noise_json_are_safe() {
        let samples = vec![sample_with_timing(None, None); 8];
        let decision = timing_decision_json(&samples, f64::NAN);
        assert_eq!(decision["timing_source"], "nominal_rate_assumed");
        assert_eq!(decision["tau0_s"], Value::Null);
        let report = noise_report_json(&samples, f64::NAN, None);
        let text = serde_json::to_string(&report).unwrap();
        assert!(!text.contains("NaN"));
        assert!(!text.contains("Infinity"));
    }

    #[test]
    fn noise_report_json_shape_has_axes_and_no_psd() {
        let samples: Vec<_> = (0..16)
            .map(|i| sample_with_timing(Some(i as f64 * 0.1), Some(i)))
            .collect();
        let report = noise_report_json(&samples, 10.0, None);
        assert_eq!(report["publication_grade"], false);
        assert_eq!(report["noise_report_note"], "informational_only");
        assert_eq!(report["axes"].as_array().unwrap().len(), 6);
        assert!(report.get("psd").is_none());
        assert!(
            report["axes"].as_array().unwrap()[0]
                .get("psd_floor")
                .is_none()
        );
    }

    #[test]
    fn noise_report_valid_band_adds_psd_floor_nominal_unverified() {
        let samples = vec![sample_with_timing(None, None); 32];
        let report = noise_report_json(&samples, 100.0, Some([5.0, 20.0]));
        let psd = &report["axes"].as_array().unwrap()[0]["psd_floor"];
        assert_eq!(psd["timing_source"], "nominal_rate_assumed");
        assert_eq!(psd["timing_verified"], false);
        assert_eq!(psd["publication_grade"], false);
    }

    #[test]
    fn noise_report_trusted_timing_psd_verified() {
        let samples: Vec<_> = (0..32)
            .map(|i| sample_with_timing(Some(i as f64 * 0.01), Some(i)))
            .collect();
        let report = noise_report_json(&samples, 100.0, Some([5.0, 20.0]));
        let psd = &report["axes"].as_array().unwrap()[0]["psd_floor"];
        assert_eq!(psd["timing_source"], "trusted_timestamps");
        assert_eq!(psd["timing_verified"], true);
        assert_eq!(psd["publication_grade"], false);
    }

    #[test]
    fn noise_report_band_above_nyquist_psd_unavailable_safe() {
        let samples = vec![sample_with_timing(None, None); 32];
        let report = noise_report_json(&samples, 100.0, Some([60.0, 70.0]));
        let psd = &report["axes"].as_array().unwrap()[0]["psd_floor"];
        assert_eq!(psd["status"], "unavailable");
        let text = serde_json::to_string(&report).unwrap();
        assert!(!text.contains("NaN"));
        assert!(!text.contains("Infinity"));
    }

    #[test]
    fn noise_psd_band_config_requires_both_bounds() {
        assert!(validate_noise_psd_band(Some(1.0), None).is_err());
        assert!(validate_noise_psd_band(None, Some(2.0)).is_err());
        assert!(validate_noise_psd_band(Some(0.0), Some(2.0)).is_err());
        assert!(validate_noise_psd_band(Some(3.0), Some(2.0)).is_err());
        assert_eq!(validate_noise_psd_band(None, None).unwrap(), None);
        assert_eq!(
            validate_noise_psd_band(Some(1.0), Some(2.0)).unwrap(),
            Some([1.0, 2.0])
        );
    }

    #[test]
    fn stationary_suite_exit_code_preserves_report_mode_behavior() {
        assert_eq!(
            stationary_suite_exit_code([0, 0, 0, 0, 0], ValidationMode::Report, true),
            0
        );
        assert_eq!(
            stationary_suite_exit_code([0, 0, 0, 0, 0], ValidationMode::Strict, true),
            1
        );
        assert_eq!(
            stationary_suite_exit_code([0, 0, 1, 0, 0], ValidationMode::Report, false),
            1
        );
    }
    #[test]
    fn orientation_fixture_covers_axes() {
        assert_eq!(
            orientation_analyze(
                Path::new("tests/fixtures/auto-orientation.log"),
                3,
                0.8,
                1.2,
                0.7
            )
            .unwrap(),
            0
        )
    }
    #[test]
    fn sixface_fixture_parses_real_face_samples() {
        let faces = parse_sixface(Path::new("tests/fixtures/sixface.log")).unwrap();
        assert!(faces.values().map(Vec::len).sum::<usize>() > 10);
        assert!(sixface_analyze(Path::new("tests/fixtures/sixface.log"), 5, None).is_ok())
    }
    #[test]
    fn sixface_mapping_parser_accepts_example() {
        let m =
            load_sixface_mapping(Path::new("../../config/sixface-mapping.example.json")).unwrap();
        assert_eq!(m["+X"], "+X");
        assert_eq!(m["-Z"], "-Z");
        assert_eq!(m.len(), 6);
    }
    #[test]
    fn partial_sixface_fixture_fails_strict_mapping() {
        assert_eq!(
            sixface_analyze(
                Path::new("tests/fixtures/sixface.log"),
                5,
                Some(Path::new("../../config/sixface-mapping.example.json"))
            )
            .unwrap(),
            1
        );
    }
}
