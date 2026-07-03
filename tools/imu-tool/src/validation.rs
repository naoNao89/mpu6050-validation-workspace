use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImuSample {
    pub accel_g: [f64; 3],
    pub gyro_dps: [f64; 3],
    pub timestamp_s: Option<f64>,
    pub sequence: Option<u64>,
}

impl ImuSample {
    #[allow(dead_code)]
    pub fn from_g_dps(accel_g: [f64; 3], gyro_dps: [f64; 3]) -> Self {
        Self {
            accel_g,
            gyro_dps,
            timestamp_s: None,
            sequence: None,
        }
    }

    #[allow(dead_code)]
    pub fn from_si(accel_mps2: [f64; 3], gyro_radps: [f64; 3]) -> Self {
        const STANDARD_GRAVITY_MPS2: f64 = 9.80665;
        Self::from_g_dps(
            accel_mps2.map(|v| v / STANDARD_GRAVITY_MPS2),
            gyro_radps.map(f64::to_degrees),
        )
    }

    #[allow(dead_code)]
    pub fn new(accel_g: [f64; 3], gyro_dps: [f64; 3]) -> Self {
        Self::from_g_dps(accel_g, gyro_dps)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AccelCalibration {
    pub offset_g: [f64; 3],
    pub scale: [f64; 3],
}

impl AccelCalibration {
    pub const fn identity() -> Self {
        Self {
            offset_g: [0.0; 3],
            scale: [1.0; 3],
        }
    }
}

impl Default for AccelCalibration {
    fn default() -> Self {
        Self::identity()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GyroCalibration {
    pub bias_dps: [f64; 3],
}

impl GyroCalibration {
    pub const fn identity() -> Self {
        Self { bias_dps: [0.0; 3] }
    }
}

impl Default for GyroCalibration {
    fn default() -> Self {
        Self::identity()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[allow(dead_code)]
pub struct ImuCalibration {
    pub accel: AccelCalibration,
    pub gyro: GyroCalibration,
}

impl ImuCalibration {
    #[allow(dead_code)]
    pub const fn identity() -> Self {
        Self {
            accel: AccelCalibration::identity(),
            gyro: GyroCalibration::identity(),
        }
    }

    #[allow(dead_code)]
    pub fn apply(&self, sample: &ImuSample) -> ImuSample {
        let mut accel_g = sample.accel_g;
        let mut gyro_dps = sample.gyro_dps;
        for i in 0..3 {
            let raw = sample.accel_g[i];
            let offset = self.accel.offset_g[i];
            let scale = self.accel.scale[i];
            let corrected = (raw - offset) / scale;
            if raw.is_finite()
                && offset.is_finite()
                && scale.is_finite()
                && scale != 0.0
                && corrected.is_finite()
            {
                accel_g[i] = corrected;
            }
            let raw_g = sample.gyro_dps[i];
            let bias = self.gyro.bias_dps[i];
            let corrected_g = raw_g - bias;
            if raw_g.is_finite() && bias.is_finite() && corrected_g.is_finite() {
                gyro_dps[i] = corrected_g;
            }
        }
        ImuSample {
            accel_g,
            gyro_dps,
            timestamp_s: sample.timestamp_s,
            sequence: sample.sequence,
        }
    }
}

impl Default for ImuCalibration {
    fn default() -> Self {
        Self::identity()
    }
}

pub mod spec {
    use super::{AccelCalibration, VerdictStatus};
    use serde::{Deserialize, Serialize};

    #[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
    pub enum SignedAxis {
        PosX,
        NegX,
        PosY,
        NegY,
        PosZ,
        NegZ,
    }

    #[derive(Clone, Debug)]
    pub struct FaceAccelSamples {
        pub face: SignedAxis,
        pub accel_g: Vec<[f64; 3]>,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct SixFaceAccelConfig {
        pub min_samples_per_face: usize,
        pub reference_gravity_g: f64,
        pub magnitude_tolerance_g: f64,
        pub scale_factor_min: f64,
        pub scale_factor_max: f64,
    }

    impl Default for SixFaceAccelConfig {
        fn default() -> Self {
            Self {
                min_samples_per_face: 5,
                reference_gravity_g: 1.0,
                magnitude_tolerance_g: 0.25,
                scale_factor_min: 0.2,
                scale_factor_max: 5.0,
            }
        }
    }

    impl SignedAxis {
        fn axis_index(self) -> usize {
            match self {
                Self::PosX | Self::NegX => 0,
                Self::PosY | Self::NegY => 1,
                Self::PosZ | Self::NegZ => 2,
            }
        }

        fn is_positive(self) -> bool {
            matches!(self, Self::PosX | Self::PosY | Self::PosZ)
        }
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct SixFaceAccelReport {
        pub report_version: u32,
        pub standard_alignment: &'static str,
        pub compliance_claimed: bool,
        pub traceability: &'static str,
        pub test_conditions_available: bool,
        pub test_conditions_status: VerdictStatus,
        pub face_convention: &'static str,
        pub overall_status: VerdictStatus,
        pub faces: Vec<FaceReport>,
        pub axes: Vec<AxisPairReport>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub calibration: Option<AccelCalibration>,
        pub unsupported_unavailable_metrics: Vec<&'static str>,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct FaceReport {
        pub face: SignedAxis,
        pub sample_count: usize,
        pub finite_sample_count: usize,
        pub mean_accel_g: [Option<f64>; 3],
        pub std_accel_g: [Option<f64>; 3],
        pub dominant_axis: Option<&'static str>,
        pub dominant_signed_axis: Option<SignedAxis>,
        pub enough_finite_samples: bool,
        pub mean_magnitude_g: Option<f64>,
        pub expected_axis_dominates: bool,
        pub expected_sign_matches: bool,
        pub magnitude_plausible: bool,
        pub status: VerdictStatus,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct AxisPairReport {
        pub axis: &'static str,
        pub scale_factor: Option<f64>,
        pub zero_g_offset_g: Option<f64>,
        pub status: VerdictStatus,
    }

    fn norm3(v: [f64; 3]) -> Option<f64> {
        finite((v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt())
    }

    fn finite(x: f64) -> Option<f64> {
        x.is_finite().then_some(x)
    }
    fn mean(xs: &[f64]) -> Option<f64> {
        (!xs.is_empty())
            .then(|| xs.iter().sum::<f64>() / xs.len() as f64)
            .and_then(finite)
    }
    fn std(xs: &[f64]) -> Option<f64> {
        if xs.len() < 2 {
            None
        } else {
            let m = mean(xs)?;
            finite(
                (xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (xs.len() - 1) as f64).sqrt(),
            )
        }
    }

    pub fn analyze_sixface_accel(
        samples: &[FaceAccelSamples],
        cfg: &SixFaceAccelConfig,
    ) -> SixFaceAccelReport {
        let g_ref = cfg
            .reference_gravity_g
            .is_finite()
            .then_some(cfg.reference_gravity_g)
            .filter(|g| *g > 0.0);
        let min_samples_per_face = cfg.min_samples_per_face.max(1);
        let magnitude_tolerance_g = if cfg.magnitude_tolerance_g.is_finite() {
            cfg.magnitude_tolerance_g.abs()
        } else {
            0.25
        };
        let scale_min = if cfg.scale_factor_min.is_finite() {
            cfg.scale_factor_min.abs()
        } else {
            0.2
        }
        .max(1.0e-6);
        let scale_max = if cfg.scale_factor_max.is_finite() {
            cfg.scale_factor_max.abs()
        } else {
            5.0
        }
        .max(scale_min);
        let mut faces = Vec::new();
        for face in [
            SignedAxis::PosX,
            SignedAxis::NegX,
            SignedAxis::PosY,
            SignedAxis::NegY,
            SignedAxis::PosZ,
            SignedAxis::NegZ,
        ] {
            let all: Vec<[f64; 3]> = samples
                .iter()
                .filter(|s| s.face == face)
                .flat_map(|s| s.accel_g.iter().copied())
                .collect();
            let finite_rows: Vec<[f64; 3]> = all
                .iter()
                .copied()
                .filter(|r| r.iter().all(|v| v.is_finite()))
                .collect();
            let cols: [Vec<f64>; 3] = [0, 1, 2].map(|i| finite_rows.iter().map(|r| r[i]).collect());
            let means = [mean(&cols[0]), mean(&cols[1]), mean(&cols[2])];
            let stds = [std(&cols[0]), std(&cols[1]), std(&cols[2])];
            let mean_vec = match (means[0], means[1], means[2]) {
                (Some(x), Some(y), Some(z)) => Some([x, y, z]),
                _ => None,
            };
            let mean_magnitude_g = mean_vec.and_then(norm3);
            let dom_i = means
                .iter()
                .enumerate()
                .filter_map(|(i, v)| v.map(|x| (i, x)))
                .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap());
            let dominant_axis = dom_i.map(|(i, _)| ["X", "Y", "Z"][i]);
            let dominant_signed_axis = dom_i.map(|(i, v)| match (i, v >= 0.0) {
                (0, true) => SignedAxis::PosX,
                (0, false) => SignedAxis::NegX,
                (1, true) => SignedAxis::PosY,
                (1, false) => SignedAxis::NegY,
                (2, true) => SignedAxis::PosZ,
                _ => SignedAxis::NegZ,
            });
            let expected_i = face.axis_index();
            let expected_axis_dominates = dom_i.is_some_and(|(i, _)| i == expected_i);
            let expected_sign_matches =
                means[expected_i].is_some_and(|v| (v >= 0.0) == face.is_positive());
            let magnitude_plausible = match (mean_magnitude_g, g_ref) {
                (Some(m), Some(g)) => (m - g).abs() <= magnitude_tolerance_g,
                _ => false,
            };
            let enough_finite_samples = finite_rows.len() >= min_samples_per_face;
            let status = if !enough_finite_samples || g_ref.is_none() || mean_vec.is_none() {
                VerdictStatus::InsufficientData
            } else if expected_axis_dominates && expected_sign_matches && magnitude_plausible {
                VerdictStatus::Pass
            } else {
                VerdictStatus::Fail
            };
            faces.push(FaceReport {
                face,
                sample_count: all.len(),
                finite_sample_count: finite_rows.len(),
                mean_accel_g: means,
                std_accel_g: stds,
                dominant_axis,
                dominant_signed_axis,
                enough_finite_samples,
                mean_magnitude_g,
                expected_axis_dominates,
                expected_sign_matches,
                magnitude_plausible,
                status,
            });
        }
        let axis_names = ["X", "Y", "Z"];
        let pairs = [
            (SignedAxis::PosX, SignedAxis::NegX),
            (SignedAxis::PosY, SignedAxis::NegY),
            (SignedAxis::PosZ, SignedAxis::NegZ),
        ];
        let axes: Vec<_> = pairs
            .iter()
            .enumerate()
            .map(|(i, (p, n))| {
                let pf = faces.iter().find(|f| f.face == *p).unwrap();
                let nf = faces.iter().find(|f| f.face == *n).unwrap();
                let (scale_factor, zero_g_offset_g) = match (
                    g_ref,
                    pf.enough_finite_samples,
                    nf.enough_finite_samples,
                    pf.mean_accel_g[i],
                    nf.mean_accel_g[i],
                ) {
                    (Some(g), true, true, Some(pm), Some(nm)) => {
                        (finite((pm - nm) / (2.0 * g)), finite((pm + nm) / 2.0))
                    }
                    _ => (None, None),
                };
                let status = match scale_factor {
                    Some(s)
                        if zero_g_offset_g.is_some()
                            && s.abs() >= scale_min
                            && s.abs() <= scale_max =>
                    {
                        VerdictStatus::Pass
                    }
                    Some(_) => VerdictStatus::Fail,
                    None => VerdictStatus::InsufficientData,
                };
                AxisPairReport {
                    axis: axis_names[i],
                    scale_factor,
                    zero_g_offset_g,
                    status,
                }
            })
            .collect();
        let overall_status = if faces.iter().all(|f| f.status == VerdictStatus::Pass)
            && axes.iter().all(|a| a.status == VerdictStatus::Pass)
        {
            VerdictStatus::Pass
        } else if faces.iter().any(|f| f.status == VerdictStatus::Fail)
            || axes.iter().any(|a| a.status == VerdictStatus::Fail)
        {
            VerdictStatus::Fail
        } else {
            VerdictStatus::InsufficientData
        };
        let calibration = (overall_status == VerdictStatus::Pass).then(|| AccelCalibration {
            offset_g: [
                axes[0].zero_g_offset_g.unwrap_or(0.0),
                axes[1].zero_g_offset_g.unwrap_or(0.0),
                axes[2].zero_g_offset_g.unwrap_or(0.0),
            ],
            scale: [
                axes[0].scale_factor.unwrap_or(1.0),
                axes[1].scale_factor.unwrap_or(1.0),
                axes[2].scale_factor.unwrap_or(1.0),
            ],
        });
        SixFaceAccelReport {
            report_version: 1,
            standard_alignment: "ieee_2700_style_partial",
            compliance_claimed: false,
            traceability: "uncalibrated_fixture",
            test_conditions_available: false,
            test_conditions_status: VerdictStatus::Unavailable,
            face_convention: "+X means sensor +X axis aligned upward/gravity-positive, should measure +1g on X",
            overall_status,
            faces,
            axes,
            calibration,
            unsupported_unavailable_metrics: vec![
                "bandwidth",
                "temperature_drift",
                "gyro_scale",
                "traceable_accuracy",
                "cross_axis_sensitivity_full_3x3_calibration",
                "noise_density",
            ],
        }
    }
}

pub mod noise {
    use super::{ImuSample, VerdictStatus};
    use serde::{Deserialize, Serialize};

    #[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
    #[serde(rename_all = "snake_case")]
    pub enum TimingSource {
        TrustedTimestamps,
        NominalRateAssumed,
    }

    #[derive(Clone, Debug)]
    pub struct AllanConfig {
        pub tau0_s: f64,
        pub timing_source: TimingSource,
        pub cluster_sizes: Vec<usize>,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct AllanPoint {
        pub cluster_size: usize,
        pub tau_s: Option<f64>,
        pub deviation: Option<f64>,
        pub term_count: usize,
        pub status: VerdictStatus,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct AllanReport {
        pub report_version: u32,
        pub sample_count: usize,
        pub timing_source: TimingSource,
        pub timing_verified: bool,
        pub publication_grade: bool,
        pub input_unit: &'static str,
        pub points: Vec<AllanPoint>,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct NoiseFitReport {
        pub status: VerdictStatus,
        pub coefficient_name: &'static str,
        pub white_noise_coefficient: Option<f64>,
        pub coefficient_unit: &'static str,
        pub slope: Option<f64>,
        pub r_squared: Option<f64>,
        pub points_used: usize,
        pub start_index: Option<usize>,
        pub end_index_exclusive: Option<usize>,
        pub reason: Option<&'static str>,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct PsdFloorReport {
        pub status: VerdictStatus,
        pub sample_count: usize,
        pub input_unit: &'static str,
        pub timing_source: TimingSource,
        pub timing_verified: bool,
        pub publication_grade: bool,
        pub sample_rate_hz: Option<f64>,
        pub band_hz: [Option<f64>; 2],
        pub bins_used: usize,
        pub psd_convention: &'static str,
        pub psd_unit: String,
        pub floor_psd: Option<f64>,
        pub noise_density: Option<f64>,
        pub noise_density_unit: String,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct NoiseTimingConfig {
        pub nominal_rate_hz: Option<f64>,
        pub observed_rate_tolerance_fraction: f64,
        pub jitter_ratio_max: f64,
    }

    #[derive(Clone, Copy, Debug, Serialize)]
    pub struct NoiseTimingThresholds {
        pub observed_rate_tolerance_fraction: f64,
        pub jitter_ratio_max: f64,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct NoiseTimingDecision {
        pub timing_source: TimingSource,
        pub tau0_s: Option<f64>,
        pub sample_rate_hz_used: Option<f64>,
        pub observed_rate_hz: Option<f64>,
        pub mean_dt_s: Option<f64>,
        pub std_dt_s: Option<f64>,
        pub jitter_ratio: Option<f64>,
        pub sequence_gaps: Option<u64>,
        pub sequence_decreases_or_duplicates: Option<usize>,
        pub sequence_samples_present: usize,
        pub reason: &'static str,
        pub thresholds_used: NoiseTimingThresholds,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct ImuNoiseReportConfig {
        pub timing: NoiseTimingConfig,
        pub psd_band_hz: Option<[f64; 2]>,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct AxisNoiseReport {
        pub axis: &'static str,
        pub allan: AllanReport,
        pub white_noise_fit: NoiseFitReport,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub psd_floor: Option<PsdFloorReport>,
    }

    #[derive(Clone, Debug, Serialize)]
    pub struct ImuNoiseReport {
        pub report_version: u32,
        pub publication_grade: bool,
        pub noise_report_note: &'static str,
        pub timing_decision: NoiseTimingDecision,
        pub axes: Vec<AxisNoiseReport>,
    }

    fn finite(x: f64) -> Option<f64> {
        x.is_finite().then_some(x)
    }

    /// Returns logarithmically spaced Allan cluster sizes `m` for `n` samples.
    /// Sizes are unique, start at 1, and only include values with at least one
    /// overlapping Allan term (`n > 2*m`).
    pub fn cluster_sizes_log(n: usize) -> Vec<usize> {
        let mut out = Vec::new();
        let mut m = 1usize;
        while n > 2 * m {
            if out.last() != Some(&m) {
                out.push(m);
            }
            let next = ((m as f64) * 1.5).ceil() as usize;
            m = next.max(m + 1);
        }
        out
    }

    pub fn allan_overlapping(
        samples: &[f64],
        input_unit: &'static str,
        cfg: &AllanConfig,
    ) -> AllanReport {
        let n = samples.len();
        let timing_ok = cfg.tau0_s.is_finite() && cfg.tau0_s > 0.0;
        let data_ok = samples.iter().all(|x| x.is_finite());
        let sizes = if cfg.cluster_sizes.is_empty() {
            cluster_sizes_log(n)
        } else {
            cfg.cluster_sizes.clone()
        };
        let mut prefix = Vec::with_capacity(n + 1);
        prefix.push(0.0);
        if data_ok {
            for x in samples {
                prefix.push(prefix.last().copied().unwrap_or(0.0) + x);
            }
        }
        let mut points = Vec::new();
        for m in sizes {
            let term_count = if m > 0 && n > 2 * m { n - 2 * m + 1 } else { 0 };
            if !timing_ok || !data_ok || m == 0 || term_count == 0 {
                points.push(AllanPoint {
                    cluster_size: m,
                    tau_s: (timing_ok && m > 0)
                        .then_some(cfg.tau0_s * m as f64)
                        .and_then(finite),
                    deviation: None,
                    term_count,
                    status: if !data_ok || !timing_ok || m == 0 {
                        VerdictStatus::Unavailable
                    } else {
                        VerdictStatus::InsufficientData
                    },
                });
                continue;
            }
            let avg = |i: usize| (prefix[i + m] - prefix[i]) / m as f64;
            let mut sum = 0.0;
            for i in 0..term_count {
                let d = avg(i + m) - avg(i);
                sum += d * d;
            }
            let deviation = finite((sum / (2.0 * term_count as f64)).sqrt());
            points.push(AllanPoint {
                cluster_size: m,
                tau_s: finite(cfg.tau0_s * m as f64),
                deviation,
                term_count,
                status: if deviation.is_some() {
                    VerdictStatus::Pass
                } else {
                    VerdictStatus::Unavailable
                },
            });
        }
        AllanReport {
            report_version: 1,
            sample_count: n,
            timing_source: cfg.timing_source,
            timing_verified: cfg.timing_source == TimingSource::TrustedTimestamps && timing_ok,
            publication_grade: false,
            input_unit,
            points,
        }
    }

    fn regression(xs: &[f64], ys: &[f64]) -> Option<(f64, f64, f64)> {
        let n = xs.len();
        if n < 2 || n != ys.len() {
            return None;
        }
        let mx = xs.iter().sum::<f64>() / n as f64;
        let my = ys.iter().sum::<f64>() / n as f64;
        let sxx = xs.iter().map(|x| (x - mx).powi(2)).sum::<f64>();
        if sxx <= 0.0 {
            return None;
        }
        let sxy = xs
            .iter()
            .zip(ys)
            .map(|(x, y)| (x - mx) * (y - my))
            .sum::<f64>();
        let slope = sxy / sxx;
        let intercept = my - slope * mx;
        let ss_tot = ys.iter().map(|y| (y - my).powi(2)).sum::<f64>();
        let ss_res = xs
            .iter()
            .zip(ys)
            .map(|(x, y)| (y - (slope * x + intercept)).powi(2))
            .sum::<f64>();
        let r2 = if ss_tot == 0.0 {
            1.0
        } else {
            1.0 - ss_res / ss_tot
        };
        Some((slope, intercept, r2))
    }

    pub fn fit_white_noise_coefficient(
        report: &AllanReport,
        slope_tolerance: f64,
        min_points: usize,
        min_r_squared: f64,
        coefficient_unit: &'static str,
    ) -> NoiseFitReport {
        let need = min_points.max(4);
        let tol = if slope_tolerance.is_finite() && slope_tolerance >= 0.0 {
            slope_tolerance
        } else {
            0.15
        };
        let r2_min = if min_r_squared.is_finite() {
            min_r_squared.clamp(0.0, 1.0)
        } else {
            0.8
        };
        let pts: Vec<(usize, f64, f64)> = report
            .points
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                Some((i, p.tau_s?, p.deviation?))
                    .filter(|(_, t, d)| *t > 0.0 && *d > 0.0 && t.is_finite() && d.is_finite())
            })
            .collect();
        let mut best: Option<(usize, usize, f64, f64, f64)> = None;
        for start in 0..pts.len() {
            for end in start + need..=pts.len() {
                if !pts[start..end].windows(2).all(|w| w[0].0 + 1 == w[1].0) {
                    continue;
                }
                let xs: Vec<_> = pts[start..end].iter().map(|(_, t, _)| t.ln()).collect();
                let ys: Vec<_> = pts[start..end].iter().map(|(_, _, d)| d.ln()).collect();
                if let Some((slope, intercept, r2)) = regression(&xs, &ys)
                    && (slope + 0.5).abs() <= tol
                    && r2 >= r2_min
                {
                    let score = end - start;
                    if best.is_none_or(|b| score > b.1 - b.0) {
                        best = Some((start, end, slope, intercept, r2));
                    }
                }
            }
        }
        if let Some((start, end, slope, intercept, r2)) = best {
            NoiseFitReport {
                status: VerdictStatus::Pass,
                coefficient_name: "white_noise_coefficient",
                white_noise_coefficient: finite(intercept.exp()),
                coefficient_unit,
                slope: finite(slope),
                r_squared: finite(r2),
                points_used: end - start,
                start_index: Some(pts[start].0),
                end_index_exclusive: Some(pts[end - 1].0 + 1),
                reason: None,
            }
        } else {
            NoiseFitReport {
                status: VerdictStatus::Unavailable,
                coefficient_name: "white_noise_coefficient",
                white_noise_coefficient: None,
                coefficient_unit,
                slope: None,
                r_squared: None,
                points_used: pts.len(),
                start_index: None,
                end_index_exclusive: None,
                reason: Some(
                    "no_contiguous_points_met_white_noise_slope_and_r_squared_requirements",
                ),
            }
        }
    }

    pub fn estimate_psd_floor(
        samples: &[f64],
        input_unit: &'static str,
        sample_rate_hz: f64,
        band_hz: [f64; 2],
        timing_source: TimingSource,
    ) -> PsdFloorReport {
        let timing_verified = timing_source == TimingSource::TrustedTimestamps
            && sample_rate_hz.is_finite()
            && sample_rate_hz > 0.0;
        let base = |status, bins, floor: Option<f64>| PsdFloorReport {
            status,
            sample_count: samples.len(),
            input_unit,
            timing_source,
            timing_verified,
            publication_grade: false,
            sample_rate_hz: finite(sample_rate_hz),
            band_hz: [finite(band_hz[0]), finite(band_hz[1])],
            bins_used: bins,
            psd_convention: "one-sided real-signal periodogram, demeaned input, rectangular window",
            psd_unit: format!("{}^2/Hz", input_unit),
            floor_psd: floor,
            noise_density: floor.and_then(|x| finite(x.sqrt())),
            noise_density_unit: format!("{}/sqrt(Hz)", input_unit),
        };
        let n = samples.len();
        if n < 4
            || !sample_rate_hz.is_finite()
            || sample_rate_hz <= 0.0
            || !band_hz[0].is_finite()
            || !band_hz[1].is_finite()
            || samples.iter().any(|x| !x.is_finite())
        {
            return base(VerdictStatus::Unavailable, 0, None);
        }
        let nyq = sample_rate_hz / 2.0;
        if !(0.0 < band_hz[0] && band_hz[0] < band_hz[1] && band_hz[1] < nyq) {
            return base(VerdictStatus::Unavailable, 0, None);
        }
        let mean = samples.iter().sum::<f64>() / n as f64;
        let mut vals = Vec::new();
        for k in 1..=n / 2 {
            let f = k as f64 * sample_rate_hz / n as f64;
            if f < band_hz[0] || f > band_hz[1] {
                continue;
            }
            let mut re = 0.0;
            let mut im = 0.0;
            for (j, x) in samples.iter().enumerate() {
                let a = -2.0 * std::f64::consts::PI * k as f64 * j as f64 / n as f64;
                re += (x - mean) * a.cos();
                im += (x - mean) * a.sin();
            }
            let mut psd = (re * re + im * im) / (sample_rate_hz * n as f64);
            if k != n / 2 || n % 2 == 1 {
                psd *= 2.0;
            }
            if let Some(p) = finite(psd).filter(|p| *p >= 0.0) {
                vals.push(p);
            }
        }
        if vals.is_empty() {
            return base(VerdictStatus::InsufficientData, 0, None);
        }
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mid = vals.len() / 2;
        let med = if vals.len() % 2 == 0 {
            (vals[mid - 1] + vals[mid]) / 2.0
        } else {
            vals[mid]
        };
        base(VerdictStatus::Pass, vals.len(), finite(med))
    }

    pub fn decide_noise_timing(
        samples: &[ImuSample],
        cfg: &NoiseTimingConfig,
    ) -> NoiseTimingDecision {
        let obs_tol = cfg.observed_rate_tolerance_fraction;
        let jitter_max = cfg.jitter_ratio_max;
        let nominal_rate_hz = cfg.nominal_rate_hz.unwrap_or(f64::NAN);
        let nominal_ok = nominal_rate_hz.is_finite() && nominal_rate_hz > 0.0;
        let timestamps: Option<Vec<f64>> = samples.iter().map(|s| s.timestamp_s).collect();
        let mut observed_rate_hz = None;
        let mut mean_dt_s = None;
        let mut std_dt_s = None;
        let mut jitter_ratio = None;
        let mut sequence_gaps = None;
        let mut sequence_bad = None;
        let sequence_present_count = samples.iter().filter(|s| s.sequence.is_some()).count();
        let partial_sequence_coverage =
            sequence_present_count > 0 && sequence_present_count < samples.len();
        let mut trusted = false;
        let mut reason = "missing_or_partial_timestamps";
        if let Some(ts) = timestamps.filter(|ts| !ts.is_empty() && ts.iter().all(|t| t.is_finite()))
        {
            let dts: Vec<_> = ts.windows(2).map(|w| w[1] - w[0]).collect();
            let strictly_increasing = dts.iter().all(|dt| dt.is_finite() && *dt > 0.0);
            if strictly_increasing && !dts.is_empty() {
                let mean = dts.iter().sum::<f64>() / dts.len() as f64;
                let std = if dts.len() > 1 {
                    (dts.iter().map(|dt| (dt - mean).powi(2)).sum::<f64>() / (dts.len() - 1) as f64)
                        .sqrt()
                } else {
                    0.0
                };
                mean_dt_s = finite(mean);
                std_dt_s = finite(std);
                observed_rate_hz = (mean > 0.0).then_some(1.0 / mean).and_then(finite);
                jitter_ratio = (mean > 0.0).then_some(std / mean).and_then(finite);
                if !partial_sequence_coverage {
                    let seqs: Option<Vec<u64>> = samples.iter().map(|s| s.sequence).collect();
                    if let Some(seqs) = seqs {
                        let mut gaps = 0u64;
                        let mut bad = 0usize;
                        for w in seqs.windows(2) {
                            if w[1] <= w[0] {
                                bad += 1;
                            } else if w[1] > w[0] + 1 {
                                gaps += w[1] - w[0] - 1;
                            }
                        }
                        sequence_gaps = Some(gaps);
                        sequence_bad = Some(bad);
                    }
                }
                let rate_ok = nominal_ok
                    && obs_tol.is_finite()
                    && observed_rate_hz.is_some_and(|r| {
                        ((r - nominal_rate_hz) / nominal_rate_hz).abs() <= obs_tol
                    });
                let jitter_ok =
                    jitter_max.is_finite() && jitter_ratio.is_some_and(|j| j <= jitter_max);
                let seq_ok = !partial_sequence_coverage
                    && sequence_gaps.unwrap_or(0) == 0
                    && sequence_bad.unwrap_or(0) == 0;
                if rate_ok && jitter_ok && seq_ok {
                    trusted = true;
                    reason = "trusted_timestamps";
                } else if !rate_ok {
                    reason = "observed_rate_outside_tolerance";
                } else if !jitter_ok {
                    reason = "timestamp_jitter_too_high";
                } else if partial_sequence_coverage {
                    reason = "partial_sequence_coverage";
                } else {
                    reason = "sequence_gap_decrease_or_duplicate";
                }
            } else {
                reason = "timestamps_not_strictly_increasing";
            }
        }
        let timing_source = if trusted {
            TimingSource::TrustedTimestamps
        } else {
            TimingSource::NominalRateAssumed
        };
        let tau0_s = if trusted {
            mean_dt_s
        } else {
            nominal_ok.then_some(1.0 / nominal_rate_hz).and_then(finite)
        };
        let sample_rate_hz_used = if trusted {
            observed_rate_hz
        } else {
            cfg.nominal_rate_hz.and_then(finite).filter(|r| *r > 0.0)
        };
        NoiseTimingDecision {
            timing_source,
            tau0_s,
            sample_rate_hz_used,
            observed_rate_hz,
            mean_dt_s,
            std_dt_s,
            jitter_ratio,
            sequence_gaps,
            sequence_decreases_or_duplicates: sequence_bad,
            sequence_samples_present: sequence_present_count,
            reason: if nominal_ok || trusted {
                reason
            } else {
                "invalid_nominal_rate_and_untrusted_timestamps"
            },
            thresholds_used: NoiseTimingThresholds {
                observed_rate_tolerance_fraction: obs_tol,
                jitter_ratio_max: jitter_max,
            },
        }
    }

    pub fn analyze_imu_noise(samples: &[ImuSample], cfg: &ImuNoiseReportConfig) -> ImuNoiseReport {
        let timing_decision = decide_noise_timing(samples, &cfg.timing);
        let timing_source = timing_decision.timing_source;
        let tau0_s = timing_decision.tau0_s.unwrap_or(f64::NAN);
        let sample_rate_hz = timing_decision.sample_rate_hz_used.unwrap_or(f64::NAN);
        let allan_cfg = AllanConfig {
            tau0_s,
            timing_source,
            cluster_sizes: cluster_sizes_log(samples.len()),
        };
        let axes_data = [
            (
                "ax_g",
                "g",
                "g/sqrt(Hz)",
                samples.iter().map(|s| s.accel_g[0]).collect::<Vec<_>>(),
            ),
            (
                "ay_g",
                "g",
                "g/sqrt(Hz)",
                samples.iter().map(|s| s.accel_g[1]).collect::<Vec<_>>(),
            ),
            (
                "az_g",
                "g",
                "g/sqrt(Hz)",
                samples.iter().map(|s| s.accel_g[2]).collect::<Vec<_>>(),
            ),
            (
                "gx_dps",
                "dps",
                "dps/sqrt(Hz)",
                samples.iter().map(|s| s.gyro_dps[0]).collect::<Vec<_>>(),
            ),
            (
                "gy_dps",
                "dps",
                "dps/sqrt(Hz)",
                samples.iter().map(|s| s.gyro_dps[1]).collect::<Vec<_>>(),
            ),
            (
                "gz_dps",
                "dps",
                "dps/sqrt(Hz)",
                samples.iter().map(|s| s.gyro_dps[2]).collect::<Vec<_>>(),
            ),
        ];
        let axes = axes_data
            .into_iter()
            .map(|(axis, unit, coef_unit, vals)| {
                let allan = allan_overlapping(&vals, unit, &allan_cfg);
                let white_noise_fit = fit_white_noise_coefficient(&allan, 0.25, 4, 0.7, coef_unit);
                let psd_floor = cfg.psd_band_hz.map(|band| {
                    estimate_psd_floor(&vals, unit, sample_rate_hz, band, timing_source)
                });
                AxisNoiseReport {
                    axis,
                    allan,
                    white_noise_fit,
                    psd_floor,
                }
            })
            .collect();
        ImuNoiseReport {
            report_version: 1,
            publication_grade: false,
            noise_report_note: "informational_only",
            timing_decision,
            axes,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StationaryThresholds {
    pub accel_norm_mean_error_max_g: f64,
    pub accel_norm_rms_error_max_g: f64,
    pub accel_norm_std_max_g: f64,
    pub gyro_bias_abs_max_dps: f64,
    pub gyro_noise_std_max_dps: f64,
    pub gyro_rms_max_dps: f64,
}

impl Default for StationaryThresholds {
    fn default() -> Self {
        Self {
            accel_norm_mean_error_max_g: 0.05,
            accel_norm_rms_error_max_g: 0.08,
            accel_norm_std_max_g: 0.03,
            gyro_bias_abs_max_dps: 5.0,
            gyro_noise_std_max_dps: 2.0,
            gyro_rms_max_dps: 5.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StationaryConfig {
    pub nominal_rate_hz: Option<f64>,
    pub window_s: f64,
    pub min_samples: usize,
    pub min_stationary_fraction: f64,
    pub thresholds: StationaryThresholds,
}

impl Default for StationaryConfig {
    fn default() -> Self {
        Self {
            nominal_rate_hz: Some(10.0),
            window_s: 0.5,
            min_samples: 10,
            min_stationary_fraction: 0.60,
            thresholds: StationaryThresholds::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum VerdictStatus {
    Pass,
    Fail,
    Warn,
    Unavailable,
    InsufficientData,
}

#[derive(Clone, Debug, Serialize)]
#[allow(dead_code)]
pub struct AxisStats {
    pub mean: Option<f64>,
    pub std: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct AccelNormStats {
    pub mean_g: Option<f64>,
    pub std_g: Option<f64>,
    pub rms_error_g: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct GyroStats {
    pub bias_dps: [Option<f64>; 3],
    pub std_dps: [Option<f64>; 3],
    pub combined_rms_dps: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct StationarySegment {
    pub start_index: usize,
    pub end_index_exclusive: usize,
    pub start_time_s: Option<f64>,
    pub end_time_s: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct TimingQuality {
    pub status: VerdictStatus,
    pub timestamp_present: bool,
    pub sequence_present: bool,
    pub observed_rate_hz: Option<f64>,
    pub mean_dt_s: Option<f64>,
    pub std_dt_s: Option<f64>,
    pub duplicates: Option<usize>,
    pub reordered: Option<usize>,
    pub sequence_gaps: Option<u64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ThresholdsReport {
    pub accel_norm_mean_error_max_g: f64,
    pub accel_norm_rms_error_max_g: f64,
    pub accel_norm_std_max_g: f64,
    pub gyro_bias_abs_max_dps: f64,
    pub gyro_noise_std_max_dps: f64,
    pub gyro_rms_max_dps: f64,
    pub min_stationary_fraction: f64,
    pub min_samples: usize,
    pub window_s: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct Verdict {
    pub name: &'static str,
    pub status: VerdictStatus,
    pub value: Option<f64>,
    pub threshold: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct StationaryReport {
    pub report_version: u32,
    pub sample_count: usize,
    pub nominal_duration_s: Option<f64>,
    pub observed_duration_s: Option<f64>,
    pub nominal_rate_hz: Option<f64>,
    pub observed_rate_hz: Option<f64>,
    pub timing_quality: TimingQuality,
    pub accel_norm: AccelNormStats,
    pub gyro: GyroStats,
    pub stationary_segments: Vec<StationarySegment>,
    pub stationary_fraction: Option<f64>,
    pub thresholds_used: ThresholdsReport,
    pub verdicts: Vec<Verdict>,
}

#[derive(Clone, Copy, Debug)]
pub struct GyroBiasCalibrationConfig {
    pub min_samples: usize,
    pub accel_norm_mean_error_max_g: f64,
    pub accel_norm_rms_error_max_g: f64,
    pub accel_norm_std_max_g: f64,
    pub gyro_noise_std_max_dps: f64,
}

impl Default for GyroBiasCalibrationConfig {
    fn default() -> Self {
        let t = StationaryThresholds::default();
        Self {
            min_samples: 10,
            accel_norm_mean_error_max_g: t.accel_norm_mean_error_max_g,
            accel_norm_rms_error_max_g: t.accel_norm_rms_error_max_g,
            accel_norm_std_max_g: t.accel_norm_std_max_g,
            gyro_noise_std_max_dps: t.gyro_noise_std_max_dps,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct GyroBiasCalibrationThresholdsReport {
    pub min_samples: usize,
    pub accel_norm_mean_error_max_g: f64,
    pub accel_norm_rms_error_max_g: f64,
    pub accel_norm_std_max_g: f64,
    pub gyro_noise_std_max_dps: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct GyroBiasCalibrationReport {
    pub report_version: u32,
    pub status: VerdictStatus,
    pub source: &'static str,
    pub assumption: &'static str,
    pub publication_grade: bool,
    pub sample_count: usize,
    pub bias_dps: [Option<f64>; 3],
    pub std_dps: [Option<f64>; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calibration: Option<GyroCalibration>,
    pub thresholds_used: GyroBiasCalibrationThresholdsReport,
    pub units: &'static str,
    pub notes: Vec<&'static str>,
}

impl StationaryReport {
    pub fn physical_thresholds_failed(&self) -> bool {
        self.verdicts.iter().any(|v| {
            v.status == VerdictStatus::Fail
                && matches!(
                    v.name,
                    "stationary_fraction"
                        | "accel_norm_mean_error_g"
                        | "accel_norm_rms_error_g"
                        | "accel_norm_std_g"
                        | "gyro_combined_rms_dps"
                        | "gyro_x_bias_abs_dps"
                        | "gyro_y_bias_abs_dps"
                        | "gyro_z_bias_abs_dps"
                        | "gyro_x_noise_std_dps"
                        | "gyro_y_noise_std_dps"
                        | "gyro_z_noise_std_dps"
                )
        })
    }
}

fn finite(x: f64) -> Option<f64> {
    x.is_finite().then_some(x)
}
fn norm3(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
fn mean(xs: &[f64]) -> Option<f64> {
    (!xs.is_empty())
        .then(|| xs.iter().sum::<f64>() / xs.len() as f64)
        .and_then(finite)
}
fn std(xs: &[f64]) -> Option<f64> {
    if xs.len() < 2 {
        None
    } else {
        let m = mean(xs)?;
        finite((xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (xs.len() - 1) as f64).sqrt())
    }
}
fn rms(xs: &[f64]) -> Option<f64> {
    (!xs.is_empty())
        .then(|| (xs.iter().map(|x| x * x).sum::<f64>() / xs.len() as f64).sqrt())
        .and_then(finite)
}
fn verdict(name: &'static str, value: Option<f64>, threshold: f64, less_equal: bool) -> Verdict {
    Verdict {
        name,
        status: value.map_or(VerdictStatus::InsufficientData, |v| {
            if (less_equal && v <= threshold) || (!less_equal && v >= threshold) {
                VerdictStatus::Pass
            } else {
                VerdictStatus::Fail
            }
        }),
        value,
        threshold: finite(threshold),
    }
}

pub fn estimate_gyro_bias_calibration(
    samples: &[ImuSample],
    cfg: &GyroBiasCalibrationConfig,
) -> GyroBiasCalibrationReport {
    let finite_samples: Vec<_> = samples
        .iter()
        .filter(|s| {
            s.accel_g.iter().all(|v| v.is_finite()) && s.gyro_dps.iter().all(|v| v.is_finite())
        })
        .collect();
    let norms: Vec<_> = finite_samples
        .iter()
        .map(|s| norm3(s.accel_g))
        .filter(|v| v.is_finite())
        .collect();
    let errors: Vec<_> = norms.iter().map(|n| n - 1.0).collect();
    let accel_mean_error = mean(&errors).map(f64::abs);
    let accel_rms_error = rms(&errors);
    let accel_std = std(&norms);
    let mut gv = [Vec::new(), Vec::new(), Vec::new()];
    for s in &finite_samples {
        for (i, axis) in gv.iter_mut().enumerate() {
            axis.push(s.gyro_dps[i]);
        }
    }
    let bias_dps = [mean(&gv[0]), mean(&gv[1]), mean(&gv[2])];
    let std_dps = [std(&gv[0]), std(&gv[1]), std(&gv[2])];
    let thresholds_used = GyroBiasCalibrationThresholdsReport {
        min_samples: cfg.min_samples,
        accel_norm_mean_error_max_g: cfg.accel_norm_mean_error_max_g,
        accel_norm_rms_error_max_g: cfg.accel_norm_rms_error_max_g,
        accel_norm_std_max_g: cfg.accel_norm_std_max_g,
        gyro_noise_std_max_dps: cfg.gyro_noise_std_max_dps,
    };
    let enough = finite_samples.len() >= cfg.min_samples.max(1);
    let thresholds_finite = cfg.accel_norm_mean_error_max_g.is_finite()
        && cfg.accel_norm_rms_error_max_g.is_finite()
        && cfg.accel_norm_std_max_g.is_finite()
        && cfg.gyro_noise_std_max_dps.is_finite();
    let eligible = enough
        && thresholds_finite
        && accel_mean_error.is_some_and(|v| v <= cfg.accel_norm_mean_error_max_g)
        && accel_rms_error.is_some_and(|v| v <= cfg.accel_norm_rms_error_max_g)
        && accel_std.is_some_and(|v| v <= cfg.accel_norm_std_max_g)
        && std_dps
            .iter()
            .all(|v| v.is_some_and(|x| x <= cfg.gyro_noise_std_max_dps))
        && bias_dps.iter().all(Option::is_some);
    let calibration = eligible.then(|| GyroCalibration {
        bias_dps: [
            bias_dps[0].unwrap_or(0.0),
            bias_dps[1].unwrap_or(0.0),
            bias_dps[2].unwrap_or(0.0),
        ],
    });
    GyroBiasCalibrationReport {
        report_version: 1,
        status: if eligible {
            VerdictStatus::Pass
        } else if enough {
            VerdictStatus::Fail
        } else {
            VerdictStatus::InsufficientData
        },
        source: "stationary_zero_rate",
        assumption: "device_stationary_no_rotation",
        publication_grade: false,
        sample_count: samples.len(),
        bias_dps,
        std_dps,
        calibration,
        thresholds_used,
        units: "dps",
        notes: vec!["gyro bias estimate assumes no rotation during stationary capture"],
    }
}

pub fn analyze_stationary(samples: &[ImuSample], cfg: &StationaryConfig) -> StationaryReport {
    let n = samples.len();
    let rate = cfg.nominal_rate_hz.filter(|r| r.is_finite() && *r > 0.0);
    let duration = rate.map(|r| n as f64 / r);
    let norms: Vec<_> = samples
        .iter()
        .map(|s| norm3(s.accel_g))
        .filter(|x| x.is_finite())
        .collect();
    let errors: Vec<_> = norms.iter().map(|x| x - 1.0).collect();
    let accel = AccelNormStats {
        mean_g: mean(&norms),
        std_g: std(&norms),
        rms_error_g: rms(&errors),
    };
    let mut gv = [Vec::new(), Vec::new(), Vec::new()];
    for s in samples {
        for (i, axis) in gv.iter_mut().enumerate() {
            if s.gyro_dps[i].is_finite() {
                axis.push(s.gyro_dps[i]);
            }
        }
    }
    let gyro = GyroStats {
        bias_dps: [mean(&gv[0]), mean(&gv[1]), mean(&gv[2])],
        std_dps: [std(&gv[0]), std(&gv[1]), std(&gv[2])],
        combined_rms_dps: rms(&samples
            .iter()
            .map(|s| norm3(s.gyro_dps))
            .filter(|x| x.is_finite())
            .collect::<Vec<_>>()),
    };
    let win = rate
        .map(|r| (cfg.window_s * r).round() as usize)
        .unwrap_or(0)
        .max(2);
    let mut flags = vec![false; n];
    if n >= win && rate.is_some() {
        for start in 0..=n - win {
            let a: Vec<_> = samples[start..start + win]
                .iter()
                .map(|s| norm3(s.accel_g))
                .collect();
            let g: Vec<_> = samples[start..start + win]
                .iter()
                .map(|s| norm3(s.gyro_dps))
                .collect();
            if mean(&a)
                .is_some_and(|m| (m - 1.0).abs() <= cfg.thresholds.accel_norm_mean_error_max_g)
                && rms(&g).is_some_and(|x| x <= cfg.thresholds.gyro_rms_max_dps)
            {
                for flag in flags.iter_mut().skip(start).take(win) {
                    *flag = true;
                }
            }
        }
    }
    let mut segments = Vec::new();
    let mut i = 0;
    while i < n {
        if !flags[i] {
            i += 1;
            continue;
        }
        let start = i;
        while i < n && flags[i] {
            i += 1;
        }
        segments.push(StationarySegment {
            start_index: start,
            end_index_exclusive: i,
            start_time_s: samples[start].timestamp_s.filter(|x| x.is_finite()),
            end_time_s: samples[i - 1].timestamp_s.filter(|x| x.is_finite()),
        });
    }
    let stationary_fraction =
        (n > 0).then_some(flags.iter().filter(|x| **x).count() as f64 / n as f64);
    let tq = timing_quality(samples);
    let timestamps = samples
        .iter()
        .filter_map(|s| s.timestamp_s.filter(|x| x.is_finite()))
        .collect::<Vec<_>>();
    let observed_duration_s = timestamps
        .first()
        .zip(timestamps.last())
        .and_then(|(start, end)| finite(end - start))
        .filter(|d| *d >= 0.0);
    let th = cfg.thresholds;
    let mut verdicts = vec![
        verdict(
            "sample_count",
            finite(n as f64),
            cfg.min_samples as f64,
            false,
        ),
        verdict(
            "stationary_fraction",
            stationary_fraction,
            cfg.min_stationary_fraction,
            false,
        ),
        verdict(
            "accel_norm_mean_error_g",
            accel.mean_g.map(|m| (m - 1.0).abs()),
            th.accel_norm_mean_error_max_g,
            true,
        ),
        verdict(
            "accel_norm_rms_error_g",
            accel.rms_error_g,
            th.accel_norm_rms_error_max_g,
            true,
        ),
        verdict(
            "accel_norm_std_g",
            accel.std_g,
            th.accel_norm_std_max_g,
            true,
        ),
        verdict(
            "gyro_combined_rms_dps",
            gyro.combined_rms_dps,
            th.gyro_rms_max_dps,
            true,
        ),
    ];
    for axis in 0..3 {
        verdicts.push(verdict(
            match axis {
                0 => "gyro_x_bias_abs_dps",
                1 => "gyro_y_bias_abs_dps",
                _ => "gyro_z_bias_abs_dps",
            },
            gyro.bias_dps[axis].map(f64::abs),
            th.gyro_bias_abs_max_dps,
            true,
        ));
        verdicts.push(verdict(
            match axis {
                0 => "gyro_x_noise_std_dps",
                1 => "gyro_y_noise_std_dps",
                _ => "gyro_z_noise_std_dps",
            },
            gyro.std_dps[axis],
            th.gyro_noise_std_max_dps,
            true,
        ));
    }
    StationaryReport {
        report_version: 1,
        sample_count: n,
        nominal_duration_s: duration.and_then(finite),
        observed_duration_s,
        nominal_rate_hz: rate,
        observed_rate_hz: tq.observed_rate_hz,
        timing_quality: tq,
        accel_norm: accel,
        gyro,
        stationary_segments: segments,
        stationary_fraction,
        thresholds_used: ThresholdsReport {
            accel_norm_mean_error_max_g: th.accel_norm_mean_error_max_g,
            accel_norm_rms_error_max_g: th.accel_norm_rms_error_max_g,
            accel_norm_std_max_g: th.accel_norm_std_max_g,
            gyro_bias_abs_max_dps: th.gyro_bias_abs_max_dps,
            gyro_noise_std_max_dps: th.gyro_noise_std_max_dps,
            gyro_rms_max_dps: th.gyro_rms_max_dps,
            min_stationary_fraction: cfg.min_stationary_fraction,
            min_samples: cfg.min_samples,
            window_s: cfg.window_s,
        },
        verdicts,
    }
}

fn timing_quality(samples: &[ImuSample]) -> TimingQuality {
    let timestamp_present = samples.iter().any(|s| s.timestamp_s.is_some());
    let sequence_present = samples.iter().any(|s| s.sequence.is_some());
    let ts: Vec<_> = samples
        .iter()
        .filter_map(|s| s.timestamp_s.filter(|x| x.is_finite()))
        .collect();
    let seq: Vec<_> = samples.iter().filter_map(|s| s.sequence).collect();
    if !timestamp_present && !sequence_present {
        return TimingQuality {
            status: VerdictStatus::Unavailable,
            timestamp_present,
            sequence_present,
            observed_rate_hz: None,
            mean_dt_s: None,
            std_dt_s: None,
            duplicates: None,
            reordered: None,
            sequence_gaps: None,
        };
    }
    if ts.len() < 2 && seq.len() < 2 {
        return TimingQuality {
            status: VerdictStatus::InsufficientData,
            timestamp_present,
            sequence_present,
            observed_rate_hz: None,
            mean_dt_s: None,
            std_dt_s: None,
            duplicates: (ts.len() >= 2).then_some(0),
            reordered: (ts.len() >= 2).then_some(0),
            sequence_gaps: (seq.len() >= 2).then_some(0),
        };
    }

    let raw_dts: Vec<_> = ts.windows(2).map(|w| w[1] - w[0]).collect();
    let dts: Vec<_> = raw_dts
        .iter()
        .copied()
        .filter(|d| d.is_finite() && *d > 0.0)
        .collect();
    let mean_dt = mean(&dts);
    let observed = mean_dt.map(|d| 1.0 / d).and_then(finite);
    let duplicates = (ts.len() >= 2).then(|| ts.windows(2).filter(|w| w[1] == w[0]).count());
    let reordered = (ts.len() >= 2).then(|| ts.windows(2).filter(|w| w[1] < w[0]).count());
    let gaps = (seq.len() >= 2).then(|| {
        seq.windows(2)
            .filter(|w| w[1] > w[0])
            .map(|w| w[1] - w[0] - 1)
            .sum()
    });
    let invalid_dt = raw_dts.iter().any(|d| !d.is_finite() || *d <= 0.0);
    let sequence_reordered_or_duplicate = seq.len() >= 2 && seq.windows(2).any(|w| w[1] <= w[0]);
    let has_failures = invalid_dt
        || duplicates.is_some_and(|x| x > 0)
        || reordered.is_some_and(|x| x > 0)
        || gaps.is_some_and(|x: u64| x > 0)
        || sequence_reordered_or_duplicate;
    let has_clean_check = (ts.len() >= 2 && !dts.is_empty()) || seq.len() >= 2;
    TimingQuality {
        status: if has_failures {
            VerdictStatus::Fail
        } else if has_clean_check {
            VerdictStatus::Pass
        } else {
            VerdictStatus::InsufficientData
        },
        timestamp_present,
        sequence_present,
        observed_rate_hz: observed,
        mean_dt_s: mean_dt,
        std_dt_s: std(&dts),
        duplicates,
        reordered,
        sequence_gaps: gaps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stationary_synthetic_passes_and_serializes_without_nan() {
        let s: Vec<_> = (0..100)
            .map(|_| ImuSample::new([0.0, 0.0, 1.0], [0.1, -0.1, 0.05]))
            .collect();
        let r = analyze_stationary(
            &s,
            &StationaryConfig {
                nominal_rate_hz: Some(20.0),
                min_samples: 50,
                ..Default::default()
            },
        );
        assert!(!r.physical_thresholds_failed());
        assert_eq!(r.timing_quality.status, VerdictStatus::Unavailable);
        assert_eq!(r.nominal_duration_s, Some(5.0));
        assert_eq!(r.observed_duration_s, None);
        let json = serde_json::to_string(&r).unwrap();
        assert!(!json.contains("NaN"));
        assert!(!json.contains("\"duration_s\""));
        assert!(json.contains("\"nominal_duration_s\""));
    }
    #[test]
    fn moving_gyro_fails_stationary_thresholds() {
        let s: Vec<_> = (0..40)
            .map(|_| ImuSample::new([0.0, 0.0, 1.0], [20.0, 0.0, 0.0]))
            .collect();
        let r = analyze_stationary(
            &s,
            &StationaryConfig {
                nominal_rate_hz: Some(10.0),
                min_samples: 10,
                ..Default::default()
            },
        );
        assert!(r.physical_thresholds_failed());
        assert_eq!(r.stationary_fraction, Some(0.0));
    }

    #[test]
    fn explicit_constructors_preserve_units() {
        let sample = ImuSample::from_g_dps([0.0, 0.0, 1.0], [90.0, 0.0, -90.0]);
        assert_eq!(sample.accel_g, [0.0, 0.0, 1.0]);
        assert_eq!(sample.gyro_dps, [90.0, 0.0, -90.0]);

        let si = ImuSample::from_si([0.0, 0.0, 9.80665], [std::f64::consts::PI, 0.0, 0.0]);
        assert_eq!(si.accel_g, [0.0, 0.0, 1.0]);
        assert!((si.gyro_dps[0] - 180.0).abs() < f64::EPSILON);
    }

    #[test]
    fn physical_thresholds_ignore_non_physical_failures() {
        let s: Vec<_> = (0..5)
            .map(|_| ImuSample::from_g_dps([0.0, 0.0, 1.0], [0.1, -0.1, 0.05]))
            .collect();
        let r = analyze_stationary(
            &s,
            &StationaryConfig {
                nominal_rate_hz: Some(20.0),
                min_samples: 50,
                min_stationary_fraction: 0.0,
                ..Default::default()
            },
        );
        assert_eq!(
            r.verdicts
                .iter()
                .find(|v| v.name == "sample_count")
                .unwrap()
                .status,
            VerdictStatus::Fail
        );
        assert!(!r.physical_thresholds_failed());
    }

    #[test]
    fn timing_quality_status_reflects_available_clean_or_bad_data() {
        let mut one_timestamp = ImuSample::from_g_dps([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
        one_timestamp.timestamp_s = Some(0.0);
        let r = analyze_stationary(&[one_timestamp], &StationaryConfig::default());
        assert_eq!(r.timing_quality.status, VerdictStatus::InsufficientData);

        let mut clean: Vec<_> = (0..3)
            .map(|i| {
                let mut s = ImuSample::from_g_dps([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
                s.timestamp_s = Some(i as f64 * 0.1);
                s.sequence = Some(i);
                s
            })
            .collect();
        let r = analyze_stationary(&clean, &StationaryConfig::default());
        assert_eq!(r.timing_quality.status, VerdictStatus::Pass);
        assert_eq!(r.observed_duration_s, Some(0.2));

        clean[2].timestamp_s = Some(0.05);
        let r = analyze_stationary(&clean, &StationaryConfig::default());
        assert_eq!(r.timing_quality.status, VerdictStatus::Fail);
        assert_eq!(r.timing_quality.reordered, Some(1));

        let mut gap: Vec<_> = (0..3)
            .map(|i| {
                let mut s = ImuSample::from_g_dps([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]);
                s.sequence = Some([0, 1, 3][i]);
                s
            })
            .collect();
        let r = analyze_stationary(&gap, &StationaryConfig::default());
        assert_eq!(r.timing_quality.status, VerdictStatus::Fail);
        assert_eq!(r.timing_quality.sequence_gaps, Some(1));

        gap[2].sequence = Some(1);
        let r = analyze_stationary(&gap, &StationaryConfig::default());
        assert_eq!(r.timing_quality.status, VerdictStatus::Fail);
    }

    #[test]
    fn sixface_synthetic_recovers_offset_and_scale() {
        use super::spec::*;
        let offsets = [0.02, -0.03, 0.04];
        let scales = [1.01, 0.98, 1.05];
        let faces = [
            (SignedAxis::PosX, 0, 1.0),
            (SignedAxis::NegX, 0, -1.0),
            (SignedAxis::PosY, 1, 1.0),
            (SignedAxis::NegY, 1, -1.0),
            (SignedAxis::PosZ, 2, 1.0),
            (SignedAxis::NegZ, 2, -1.0),
        ];
        let samples: Vec<_> = faces
            .into_iter()
            .map(|(face, axis, sign)| {
                let mut accel = Vec::new();
                for _ in 0..4 {
                    let mut row = offsets;
                    row[axis] += sign * scales[axis];
                    accel.push(row);
                }
                FaceAccelSamples {
                    face,
                    accel_g: accel,
                }
            })
            .collect();
        let r = analyze_sixface_accel(
            &samples,
            &SixFaceAccelConfig {
                min_samples_per_face: 3,
                reference_gravity_g: 1.0,
                ..Default::default()
            },
        );
        assert_eq!(r.overall_status, VerdictStatus::Pass);
        for (i, axis) in r.axes.iter().enumerate() {
            assert!((axis.scale_factor.unwrap() - scales[i]).abs() < 1e-12);
            assert!((axis.zero_g_offset_g.unwrap() - offsets[i]).abs() < 1e-12);
        }
        let cal = r.calibration.unwrap();
        for i in 0..3 {
            assert!((cal.offset_g[i] - offsets[i]).abs() < 1e-12);
            assert!((cal.scale[i] - scales[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn sixface_missing_face_is_insufficient_data() {
        use super::spec::*;
        let samples = vec![FaceAccelSamples {
            face: SignedAxis::PosX,
            accel_g: vec![[1.0, 0.0, 0.0]; 3],
        }];
        let r = analyze_sixface_accel(
            &samples,
            &SixFaceAccelConfig {
                min_samples_per_face: 3,
                reference_gravity_g: 1.0,
                ..Default::default()
            },
        );
        assert_eq!(r.overall_status, VerdictStatus::InsufficientData);
        assert!(r.calibration.is_none());
    }

    #[test]
    fn sixface_json_contains_compliance_and_unsupported() {
        use super::spec::*;
        let r = analyze_sixface_accel(
            &[],
            &SixFaceAccelConfig {
                min_samples_per_face: 1,
                reference_gravity_g: 1.0,
                ..Default::default()
            },
        );
        let v = serde_json::to_value(&r).unwrap();
        assert_eq!(v["compliance_claimed"], false);
        let unsupported = v["unsupported_unavailable_metrics"].as_array().unwrap();
        assert!(unsupported.iter().any(|x| x == "bandwidth"));
        assert!(unsupported.iter().any(|x| x == "noise_density"));
    }

    #[test]
    fn sixface_mislabeled_face_data_does_not_pass() {
        use super::spec::*;
        let samples = vec![
            FaceAccelSamples {
                face: SignedAxis::PosX,
                accel_g: vec![[0.0, 1.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::NegX,
                accel_g: vec![[-1.0, 0.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::PosY,
                accel_g: vec![[0.0, 1.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::NegY,
                accel_g: vec![[0.0, -1.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::PosZ,
                accel_g: vec![[0.0, 0.0, 1.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::NegZ,
                accel_g: vec![[0.0, 0.0, -1.0]; 5],
            },
        ];
        let r = analyze_sixface_accel(&samples, &SixFaceAccelConfig::default());
        assert_ne!(r.overall_status, VerdictStatus::Pass);
        assert_eq!(r.faces[0].status, VerdictStatus::Fail);
        assert!(r.calibration.is_none());
    }

    #[test]
    fn sixface_same_orientation_scale_near_zero_fails() {
        use super::spec::*;
        let samples = vec![
            FaceAccelSamples {
                face: SignedAxis::PosX,
                accel_g: vec![[1.0, 0.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::NegX,
                accel_g: vec![[1.0, 0.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::PosY,
                accel_g: vec![[0.0, 1.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::NegY,
                accel_g: vec![[0.0, -1.0, 0.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::PosZ,
                accel_g: vec![[0.0, 0.0, 1.0]; 5],
            },
            FaceAccelSamples {
                face: SignedAxis::NegZ,
                accel_g: vec![[0.0, 0.0, -1.0]; 5],
            },
        ];
        let r = analyze_sixface_accel(&samples, &SixFaceAccelConfig::default());
        assert_ne!(r.overall_status, VerdictStatus::Pass);
        assert_eq!(r.axes[0].status, VerdictStatus::Fail);
    }

    #[test]
    fn sixface_one_sample_per_face_insufficient_by_default() {
        use super::spec::*;
        let samples = vec![
            FaceAccelSamples {
                face: SignedAxis::PosX,
                accel_g: vec![[1.0, 0.0, 0.0]],
            },
            FaceAccelSamples {
                face: SignedAxis::NegX,
                accel_g: vec![[-1.0, 0.0, 0.0]],
            },
            FaceAccelSamples {
                face: SignedAxis::PosY,
                accel_g: vec![[0.0, 1.0, 0.0]],
            },
            FaceAccelSamples {
                face: SignedAxis::NegY,
                accel_g: vec![[0.0, -1.0, 0.0]],
            },
            FaceAccelSamples {
                face: SignedAxis::PosZ,
                accel_g: vec![[0.0, 0.0, 1.0]],
            },
            FaceAccelSamples {
                face: SignedAxis::NegZ,
                accel_g: vec![[0.0, 0.0, -1.0]],
            },
        ];
        let r = analyze_sixface_accel(&samples, &SixFaceAccelConfig::default());
        assert_eq!(r.overall_status, VerdictStatus::InsufficientData);
    }

    #[test]
    fn sixface_invalid_config_and_nonfinite_samples_are_json_safe() {
        use super::spec::*;
        let samples = vec![FaceAccelSamples {
            face: SignedAxis::PosX,
            accel_g: vec![[f64::NAN, f64::INFINITY, 0.0], [1.0, 0.0, 0.0]],
        }];
        let r = analyze_sixface_accel(
            &samples,
            &SixFaceAccelConfig {
                min_samples_per_face: 0,
                reference_gravity_g: f64::NAN,
                magnitude_tolerance_g: f64::NAN,
                scale_factor_min: f64::NAN,
                scale_factor_max: f64::INFINITY,
            },
        );
        assert_ne!(r.overall_status, VerdictStatus::Pass);
        let text = serde_json::to_string(&r).unwrap();
        assert!(!text.contains("NaN"));
        assert!(!text.contains("Infinity"));
    }

    fn white_noise(n: usize) -> Vec<f64> {
        let mut state = 0x1234_5678_9abc_def0u64;
        (0..n)
            .map(|_| {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let a = ((state >> 11) as f64) / ((1u64 << 53) as f64);
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let b = ((state >> 11) as f64) / ((1u64 << 53) as f64);
                (a + b + 0.5) - 1.5
            })
            .collect()
    }

    #[test]
    fn allan_constant_sequence_zero_and_no_white_noise_fit() {
        use super::noise::*;
        let cfg = AllanConfig {
            tau0_s: 0.01,
            timing_source: TimingSource::TrustedTimestamps,
            cluster_sizes: vec![1, 2, 4, 8, 16],
        };
        let report = allan_overlapping(&vec![2.0; 128], "g", &cfg);
        assert!(report.points.iter().all(|p| p.deviation == Some(0.0)));
        let json = serde_json::to_value(&report).unwrap();
        assert_eq!(json["timing_source"], "trusted_timestamps");
        let fit = fit_white_noise_coefficient(&report, 0.2, 4, 0.8, "g/sqrt(Hz)");
        assert_eq!(fit.status, VerdictStatus::Unavailable);
        assert_eq!(
            fit.reason,
            Some("no_contiguous_points_met_white_noise_slope_and_r_squared_requirements")
        );
    }

    #[test]
    fn allan_invalid_inputs_are_json_safe() {
        use super::noise::*;
        let report = allan_overlapping(
            &[1.0, f64::NAN, 2.0],
            "dps",
            &AllanConfig {
                tau0_s: f64::NAN,
                timing_source: TimingSource::NominalRateAssumed,
                cluster_sizes: vec![0, 1, 99],
            },
        );
        assert!(
            report
                .points
                .iter()
                .all(|p| p.status != VerdictStatus::Pass)
        );
        let text = serde_json::to_string(&report).unwrap();
        assert!(!text.contains("NaN"));
        assert!(!text.contains("Infinity"));
        let json = serde_json::to_value(&report).unwrap();
        assert_eq!(json["timing_source"], "nominal_rate_assumed");
    }

    #[test]
    fn allan_white_noise_slope_and_coefficient_are_finite() {
        use super::noise::*;
        let samples = white_noise(4096);
        let report = allan_overlapping(
            &samples,
            "g",
            &AllanConfig {
                tau0_s: 0.01,
                timing_source: TimingSource::NominalRateAssumed,
                cluster_sizes: cluster_sizes_log(samples.len())
                    .into_iter()
                    .filter(|m| *m <= 128)
                    .collect(),
            },
        );
        let fit = fit_white_noise_coefficient(&report, 0.30, 4, 0.65, "g");
        assert_eq!(fit.status, VerdictStatus::Pass);
        assert_eq!(fit.reason, None);
        assert!(fit.white_noise_coefficient.unwrap().is_finite());
        assert!((fit.slope.unwrap() + 0.5).abs() <= 0.30);
    }

    #[test]
    fn psd_white_noise_floor_is_finite() {
        use super::noise::*;
        let samples = white_noise(512);
        let report = estimate_psd_floor(
            &samples,
            "g",
            100.0,
            [5.0, 25.0],
            TimingSource::TrustedTimestamps,
        );
        assert_eq!(report.status, VerdictStatus::Pass);
        let json = serde_json::to_value(&report).unwrap();
        assert_eq!(json["timing_source"], "trusted_timestamps");
        assert_eq!(json["timing_verified"], true);
        assert_eq!(json["publication_grade"], false);
        assert!(report.bins_used > 0);
        assert!(report.floor_psd.unwrap().is_finite());
        assert!(report.noise_density.unwrap().is_finite());
        assert!(report.psd_unit.contains("g^2/Hz"));
    }

    #[test]
    fn psd_invalid_rate_or_band_unavailable() {
        use super::noise::*;
        let samples = white_noise(32);
        let bad_rate = estimate_psd_floor(
            &samples,
            "g",
            0.0,
            [1.0, 2.0],
            TimingSource::TrustedTimestamps,
        );
        assert_eq!(bad_rate.status, VerdictStatus::Unavailable);
        let bad_rate_json = serde_json::to_value(&bad_rate).unwrap();
        assert_eq!(bad_rate_json["timing_source"], "trusted_timestamps");
        assert_eq!(bad_rate_json["timing_verified"], false);
        assert_eq!(bad_rate_json["publication_grade"], false);
        let bad_band = estimate_psd_floor(
            &samples,
            "g",
            10.0,
            [4.0, 6.0],
            TimingSource::NominalRateAssumed,
        );
        assert_eq!(bad_band.status, VerdictStatus::Unavailable);
        let bad_band_json = serde_json::to_value(&bad_band).unwrap();
        assert_eq!(bad_band_json["timing_source"], "nominal_rate_assumed");
        assert_eq!(bad_band_json["timing_verified"], false);
        let empty_band = estimate_psd_floor(
            &samples,
            "g",
            100.0,
            [1.0, 1.1],
            TimingSource::NominalRateAssumed,
        );
        assert_eq!(empty_band.status, VerdictStatus::InsufficientData);
    }

    fn timed_noise_sample(timestamp_s: Option<f64>, sequence: Option<u64>) -> ImuSample {
        ImuSample {
            accel_g: [0.0, 0.0, 1.0],
            gyro_dps: [0.0, 0.0, 0.0],
            timestamp_s,
            sequence,
        }
    }

    fn noise_timing_cfg(rate: f64) -> super::noise::NoiseTimingConfig {
        super::noise::NoiseTimingConfig {
            nominal_rate_hz: Some(rate),
            observed_rate_tolerance_fraction: 0.05,
            jitter_ratio_max: 0.05,
        }
    }

    #[test]
    fn noise_timing_decision_preserves_trust_semantics() {
        use super::noise::*;
        let no_timestamps = vec![timed_noise_sample(None, None); 5];
        let d = decide_noise_timing(&no_timestamps, &noise_timing_cfg(10.0));
        assert_eq!(d.timing_source, TimingSource::NominalRateAssumed);
        assert_eq!(d.reason, "missing_or_partial_timestamps");

        let trusted: Vec<_> = (0..6)
            .map(|i| timed_noise_sample(Some(i as f64 * 0.1), Some(i)))
            .collect();
        let d = decide_noise_timing(&trusted, &noise_timing_cfg(10.0));
        assert_eq!(d.timing_source, TimingSource::TrustedTimestamps);
        assert_eq!(d.reason, "trusted_timestamps");

        let partial_seq = vec![
            timed_noise_sample(Some(0.0), Some(0)),
            timed_noise_sample(Some(0.1), None),
            timed_noise_sample(Some(0.2), Some(2)),
        ];
        let d = decide_noise_timing(&partial_seq, &noise_timing_cfg(10.0));
        assert_eq!(d.timing_source, TimingSource::NominalRateAssumed);
        assert_eq!(d.reason, "partial_sequence_coverage");
        assert_eq!(d.sequence_samples_present, 2);
    }

    #[test]
    fn gyro_bias_calibration_estimates_stationary_bias() {
        let samples: Vec<_> = (0..20)
            .map(|_| ImuSample::from_g_dps([0.0, 0.0, 1.0], [0.1, -0.2, 0.3]))
            .collect();
        let r = estimate_gyro_bias_calibration(&samples, &GyroBiasCalibrationConfig::default());
        assert_eq!(r.status, VerdictStatus::Pass);
        let bias = r.calibration.unwrap().bias_dps;
        assert!((bias[0] - 0.1).abs() < 1e-12);
        assert!((bias[1] + 0.2).abs() < 1e-12);
        assert!((bias[2] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn gyro_bias_calibration_rejects_bad_accel_or_noise() {
        let bad_accel: Vec<_> = (0..20)
            .map(|_| ImuSample::from_g_dps([0.0, 0.0, 1.5], [0.1, -0.2, 0.3]))
            .collect();
        let r = estimate_gyro_bias_calibration(&bad_accel, &GyroBiasCalibrationConfig::default());
        assert_eq!(r.status, VerdictStatus::Fail);
        assert!(r.calibration.is_none());

        let noisy: Vec<_> = (0..20)
            .map(|i| {
                ImuSample::from_g_dps(
                    [0.0, 0.0, 1.0],
                    [if i % 2 == 0 { 10.0 } else { -10.0 }, 0.0, 0.0],
                )
            })
            .collect();
        let r = estimate_gyro_bias_calibration(&noisy, &GyroBiasCalibrationConfig::default());
        assert_eq!(r.status, VerdictStatus::Fail);
        assert!(r.calibration.is_none());
    }

    #[test]
    fn gyro_bias_calibration_insufficient_nonfinite_json_safe() {
        let samples = vec![ImuSample::from_g_dps(
            [f64::NAN, 0.0, 1.0],
            [0.0, f64::INFINITY, 0.0],
        )];
        let r = estimate_gyro_bias_calibration(&samples, &GyroBiasCalibrationConfig::default());
        assert_eq!(r.status, VerdictStatus::InsufficientData);
        assert!(r.calibration.is_none());
        let text = serde_json::to_string(&r).unwrap();
        assert!(!text.contains("NaN"));
        assert!(!text.contains("Infinity"));
    }

    #[test]
    fn analyze_imu_noise_serializes_expected_shape_and_psd_optional() {
        use super::noise::*;
        let samples: Vec<_> = (0..32)
            .map(|i| timed_noise_sample(Some(i as f64 * 0.01), Some(i)))
            .collect();
        let no_psd = analyze_imu_noise(
            &samples,
            &ImuNoiseReportConfig {
                timing: noise_timing_cfg(100.0),
                psd_band_hz: None,
            },
        );
        let v = serde_json::to_value(&no_psd).unwrap();
        assert_eq!(v["noise_report_note"], "informational_only");
        assert_eq!(v["axes"].as_array().unwrap().len(), 6);
        assert!(v["axes"].as_array().unwrap()[0].get("psd_floor").is_none());

        let with_psd = analyze_imu_noise(
            &samples,
            &ImuNoiseReportConfig {
                timing: noise_timing_cfg(100.0),
                psd_band_hz: Some([5.0, 20.0]),
            },
        );
        let v = serde_json::to_value(&with_psd).unwrap();
        assert!(v["axes"].as_array().unwrap()[0].get("psd_floor").is_some());
        assert_eq!(v["axes"][0]["psd_floor"]["timing_verified"], true);
    }
}
