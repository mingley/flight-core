//! Live driver: discover / reference / read do not step; write is attach-gated.

use robot_lab::{Lab, LabCmd};

use crate::error::MhsError;
use crate::limits::DriverLimits;
use crate::surface::{
    preview_write, read_channel, DeviceReference, Discovery, ReadResult, WriteOk, WriteRequest,
};
use crate::ChainOp;

/// MHS-shaped driver over one [`Lab`].
pub struct Driver {
    lab: Lab,
    limits: DriverLimits,
}

impl Driver {
    pub fn open(scenario: &str, seed: u64) -> Result<Self, MhsError> {
        Ok(Self {
            lab: Lab::open(scenario, seed)?,
            limits: DriverLimits::DEFAULT,
        })
    }

    pub fn coastal(seed: u64) -> Self {
        Self {
            lab: Lab::coastal(seed),
            limits: DriverLimits::DEFAULT,
        }
    }

    pub fn from_lab(lab: Lab) -> Self {
        Self {
            lab,
            limits: DriverLimits::DEFAULT,
        }
    }

    pub fn with_limits(mut self, limits: DriverLimits) -> Self {
        self.limits = limits;
        self
    }

    pub fn lab(&self) -> &Lab {
        &self.lab
    }

    pub fn lab_mut(&mut self) -> &mut Lab {
        &mut self.lab
    }

    pub fn into_lab(self) -> Lab {
        self.lab
    }

    pub fn limits(&self) -> &DriverLimits {
        &self.limits
    }

    /// Discovery document. Observe does not step.
    pub fn discover(&self) -> Discovery {
        Discovery::from_observation(&self.lab.observe())
    }

    pub fn reference(&self, device: &str) -> Result<DeviceReference, MhsError> {
        DeviceReference::compile(&self.lab.observe(), device, &self.limits)
    }

    pub fn references(&self) -> Vec<DeviceReference> {
        let obs = self.lab.observe();
        self.discover()
            .devices
            .iter()
            .filter_map(|d| DeviceReference::compile(&obs, &d.id, &self.limits).ok())
            .collect()
    }

    pub fn read(&self, device: &str, channel: &str) -> Result<ReadResult, MhsError> {
        read_channel(&self.lab.observe(), device, channel)
    }

    /// Write through [`Lab::act_through_attach`]. Does not step (P12).
    pub fn write(&mut self, req: &WriteRequest) -> Result<WriteOk, MhsError> {
        let obs = self.lab.observe();
        let cmd = preview_write(&obs, req, &self.limits)?;
        let action = req.to_action(cmd);
        if let Err(e) = self.lab.act_through_attach(action) {
            let mut mapped = MhsError::from(e);
            if let MhsError::UnknownDevice { id, invariant } = &mut mapped {
                if invariant.is_none() {
                    *invariant = crate::error::catalog_omit(obs.scenario, id);
                }
            }
            return Err(mapped);
        }
        Ok(WriteOk {
            ok: true,
            device: req.device.clone(),
            channel: req.channel.clone(),
            message: self.lab.message.clone(),
        })
    }

    /// One verified world step (P12). `n` ticks of `dt`.
    pub fn step(&mut self, dt: f32, n: u32) {
        for _ in 0..n {
            self.lab.step(dt);
        }
    }

    pub fn last_failure(&self, err: &MhsError) -> crate::error::MhsFailure {
        err.as_failure(self.lab.last_reject().cloned())
    }

    pub fn run_chain(&mut self, ops: &[ChainOp], dt_default: f32) -> crate::ChainReport {
        let mut report = crate::ChainReport {
            scenario: self.lab.world().scenario.into(),
            seed: self.lab.world().seed,
            ops: 0,
            steps: 0,
            t: self.lab.world().t,
            all_hold: self.lab.all_hold(),
            broken: self.lab.broken(),
            certificates: Vec::new(),
            rejects: Vec::new(),
            reads: Vec::new(),
            ok: true,
        };
        for op in ops {
            report.ops += 1;
            match op {
                ChainOp::Write {
                    device,
                    channel,
                    vn,
                    ve,
                    vd,
                    yaw_rate,
                } => {
                    let req = WriteRequest {
                        device: device.clone(),
                        channel: channel.clone(),
                        vn: *vn,
                        ve: *ve,
                        vd: *vd,
                        yaw_rate: *yaw_rate,
                    };
                    if let Err(e) = self.write(&req) {
                        report.ok = false;
                        report.rejects.push(self.last_failure(&e));
                        break;
                    }
                }
                ChainOp::Read { device, channel } => match self.read(device, channel) {
                    Ok(r) => report.reads.push(r),
                    Err(e) => {
                        report.ok = false;
                        report.rejects.push(self.last_failure(&e));
                        break;
                    }
                },
                ChainOp::Step { dt, n } => {
                    let dt = if *dt > 0.0 { *dt } else { dt_default };
                    self.step(dt, *n);
                    report.steps += *n;
                }
            }
        }
        report.t = self.lab.world().t;
        report.all_hold = self.lab.all_hold();
        report.broken = self.lab.broken();
        if let Ok(r) = self.read(crate::tags::DEVICE_LAB, "certificates") {
            if let Some(arr) = r.value.as_array() {
                report.certificates = arr
                    .iter()
                    .filter_map(|v| v.as_str().map(str::to_string))
                    .collect();
            }
        }
        report
    }
}

/// Map a successful preview into a lab action (demo HTTP queue).
pub fn queued_action(req: &WriteRequest, cmd: LabCmd) -> robot_lab::AgentAction {
    req.to_action(cmd)
}
