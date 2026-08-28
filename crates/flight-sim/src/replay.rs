//! Recorded IMU / clock streams. The controller is identical to production.

use flight_core::frames::Body;
use flight_core::sensors::{Imu, ImuSample, SensorError, SensorHealth};
use flight_core::time::{Clock, Duration, MonotonicInstant, VirtualClock};
use flight_core::vector::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordedSample {
    pub t_ms: u64,
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    pub sequence: u32,
}

#[derive(Clone, Debug)]
pub struct JsonlReplay {
    samples: Vec<RecordedSample>,
    idx: usize,
    clock: VirtualClock,
}

impl JsonlReplay {
    pub fn from_jsonl(text: &str) -> Result<Self, String> {
        let mut samples = Vec::new();
        for (i, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let s: RecordedSample =
                serde_json::from_str(line).map_err(|e| format!("line {}: {e}", i + 1))?;
            samples.push(s);
        }
        if samples.is_empty() {
            return Err("replay corpus is empty".into());
        }
        Ok(Self {
            samples,
            idx: 0,
            clock: VirtualClock::new(),
        })
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn rewind(&mut self) {
        self.idx = 0;
        self.clock = VirtualClock::new();
    }
}

impl Clock for JsonlReplay {
    fn now(&self) -> MonotonicInstant {
        self.clock.now()
    }
}

impl Imu for JsonlReplay {
    type Frame = Body;

    fn sample(&mut self) -> Result<ImuSample<Body>, SensorError> {
        let rec = self.samples.get(self.idx).ok_or(SensorError::Timeout)?;
        self.idx += 1;
        self.clock.set(MonotonicInstant::from_millis(rec.t_ms));
        Ok(ImuSample {
            timestamp: MonotonicInstant::from_millis(rec.t_ms),
            accel: Vector3::new(rec.accel[0], rec.accel[1], rec.accel[2]),
            gyro: Vector3::new(rec.gyro[0], rec.gyro[1], rec.gyro[2]),
            covariance: None,
            temperature: None,
            status: SensorHealth::Ok,
            sequence: rec.sequence,
        })
    }
}

/// Clock that follows recorded timestamps.
pub fn duration_between(a: &RecordedSample, b: &RecordedSample) -> Duration {
    Duration::from_millis(b.t_ms.saturating_sub(a.t_ms))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replays_jsonl() {
        let text = r#"
{"t_ms":0,"accel":[0,0,-9.81],"gyro":[0,0,0],"sequence":0}
{"t_ms":10,"accel":[0,0,-9.81],"gyro":[0,0,0],"sequence":1}
{"t_ms":20,"accel":[0,0,-9.81],"gyro":[0,0,0],"sequence":2}
"#;
        let mut r = JsonlReplay::from_jsonl(text).unwrap();
        assert_eq!(r.len(), 3);
        let a = r.sample().unwrap();
        assert_eq!(a.sequence, 0);
        let b = r.sample().unwrap();
        assert_eq!(b.sequence, 1);
        assert!(r.sample().is_ok());
        assert!(r.sample().is_err());
    }
}
