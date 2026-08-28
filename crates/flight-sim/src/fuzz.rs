//! Seeded IMU noise wrapper. Same `Imu` trait as the clean sensor.

use flight_core::frames::Frame;
use flight_core::sensors::{Imu, ImuSample, SensorError};

#[derive(Clone, Debug)]
pub struct FuzzedImu<I> {
    inner: I,
    state: u64,
    accel_std: f32,
    gyro_std: f32,
}

impl<I> FuzzedImu<I> {
    pub fn new(inner: I, seed: u64, accel_std: f32, gyro_std: f32) -> Self {
        Self {
            inner,
            state: seed | 1,
            accel_std,
            gyro_std,
        }
    }
}

impl<I: Imu> Imu for FuzzedImu<I>
where
    I::Frame: Frame,
{
    type Frame = I::Frame;

    fn sample(&mut self) -> Result<ImuSample<Self::Frame>, SensorError> {
        let mut s = self.inner.sample()?;
        let ax = self.accel_std * self.unit();
        let ay = self.accel_std * self.unit();
        let az = self.accel_std * self.unit();
        let gx = self.gyro_std * self.unit();
        let gy = self.gyro_std * self.unit();
        let gz = self.gyro_std * self.unit();
        s.accel = s.accel + flight_core::vector::Vector3::new(ax, ay, az);
        s.gyro = s.gyro + flight_core::vector::Vector3::new(gx, gy, gz);
        Ok(s)
    }
}

impl<I> FuzzedImu<I> {
    fn unit(&mut self) -> f32 {
        // xorshift64 + Box-Muller-ish one-dim: uniform [-1,1] is enough for tests.
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        let u = (self.state >> 11) as f32 / (u64::MAX >> 11) as f32;
        u * 2.0 - 1.0
    }
}
