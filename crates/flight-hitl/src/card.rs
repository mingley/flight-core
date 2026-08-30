//! Faithful FCH1 UDP I/O card. Not the in-process plant.
//!
//! The card speaks the [`crate::protocol`] datagrams on a real socket. It
//! never calls `World::try_step`. The rack still steps the verified world after
//! [`WorldRack::drain_io`](crate::WorldRack::drain_io) turns wire commands into
//! [`RackCommand::from_fch1`](crate::RackCommand::from_fch1) (`apply == 0` zeros
//! that slot). Slot map: 0 drone, 1 rover, 2 skiff, 3 surveyor.

use std::net::{SocketAddr, UdpSocket};

use flight_core::vehicle::BackendError;

use crate::protocol::{decode_sample, encode_command, Command, Sample};
use crate::rack::WorldRack;
use crate::{command_from_datagram, RackCommand};

/// One wire event a mock card recorded.
#[derive(Clone, Debug, PartialEq)]
pub enum Fch1WireEvent {
    TxCommand(Command),
    RxSample(Sample),
}

impl Fch1WireEvent {
    pub fn to_json_line(&self) -> String {
        match self {
            Self::TxCommand(c) => format!(
                "{{\"kind\":\"command\",\"slot\":{},\"apply\":{},\"vn\":{},\"ve\":{},\"vd\":{}}}",
                c.slot, c.apply, c.velocity_ned[0], c.velocity_ned[1], c.velocity_ned[2]
            ),
            Self::RxSample(s) => format!(
                "{{\"kind\":\"sample\",\"slot\":{},\"t_ns\":{}}}",
                s.slot, s.t_plant_ns
            ),
        }
    }
}

/// UDP peer that is an I/O card, not a second `WorldSession`.
pub struct Fch1UdpCard {
    sock: UdpSocket,
    peer: Option<SocketAddr>,
    log: Vec<Fch1WireEvent>,
}

impl Fch1UdpCard {
    pub fn bind() -> Result<Self, BackendError> {
        let sock = UdpSocket::bind("127.0.0.1:0").map_err(|_| BackendError::Io)?;
        sock.set_nonblocking(true).map_err(|_| BackendError::Io)?;
        Ok(Self {
            sock,
            peer: None,
            log: Vec::new(),
        })
    }

    pub fn local_addr(&self) -> Result<SocketAddr, BackendError> {
        self.sock.local_addr().map_err(|_| BackendError::Io)
    }

    /// Connect to the rack's bound address so `send` / `recv` stay on that peer.
    pub fn set_peer(&mut self, rack: SocketAddr) -> Result<(), BackendError> {
        self.sock.connect(rack).map_err(|_| BackendError::Io)?;
        self.peer = Some(rack);
        Ok(())
    }

    pub fn send_command(&mut self, c: Command) -> Result<(), BackendError> {
        if self.peer.is_none() {
            return Err(BackendError::Disconnected);
        }
        self.sock
            .send(&encode_command(c))
            .map_err(|_| BackendError::Io)?;
        self.log.push(Fch1WireEvent::TxCommand(c));
        Ok(())
    }

    /// Drain waiting sample datagrams into the log. Returns how many decoded.
    pub fn drain_samples(&mut self) -> usize {
        let mut buf = [0u8; 64];
        let mut n = 0;
        while let Ok(len) = self.sock.recv(&mut buf) {
            if let Some(s) = decode_sample(&buf[..len]) {
                self.log.push(Fch1WireEvent::RxSample(s));
                n += 1;
            }
        }
        n
    }

    pub fn log(&self) -> &[Fch1WireEvent] {
        &self.log
    }

    pub fn to_jsonl(&self) -> String {
        let mut out = String::new();
        for e in &self.log {
            out.push_str(&e.to_json_line());
            out.push('\n');
        }
        out
    }

    pub fn samples_received(&self) -> usize {
        self.log
            .iter()
            .filter(|e| matches!(e, Fch1WireEvent::RxSample(_)))
            .count()
    }
}

/// Recorded inland pass: hold, hull-slot commands do not create bodies,
/// `apply == 0` zeros slot 0 and does not revive hold, then a live climb
/// clears hold. Commands and samples traverse UDP.
pub fn run_fch1_udp_mock() -> Result<Fch1MockReport, String> {
    let mut card = Fch1UdpCard::bind().map_err(|e| format!("card bind: {e}"))?;
    let card_addr = card.local_addr().map_err(|e| format!("card addr: {e}"))?;
    let mut rack = WorldRack::inland(1).map_err(|e| format!("inland: {e}"))?;
    let rack_addr = rack
        .bind_io(&card_addr.to_string())
        .map_err(|e| format!("rack bind_io: {e}"))?;
    card.set_peer(rack_addr)
        .map_err(|e| format!("card peer: {e}"))?;
    rack.hold().map_err(|e| format!("hold: {e}"))?;
    let hold = rack
        .world()
        .body("drone")
        .and_then(|b| b.hold_ned)
        .ok_or_else(|| "hold_ned missing after attach_hold".to_string())?;

    card.send_command(Command {
        slot: 2,
        velocity_ned: [1.0, 0.0, 0.0],
        apply: 1,
    })
    .map_err(|e| format!("skiff cmd: {e}"))?;
    card.send_command(Command {
        slot: 3,
        velocity_ned: [0.0, 1.0, 0.0],
        apply: 1,
    })
    .map_err(|e| format!("surveyor cmd: {e}"))?;
    let f = rack
        .frame_from_io(0.02, 1_000_000)
        .map_err(|e| format!("hull-slot frame: {e}"))?;
    if f.missed() {
        return Err("hull-slot frame missed".into());
    }
    if rack.world().body("skiff").is_some() || rack.world().body("surveyor").is_some() {
        return Err("inland UDP hull slots must not create bodies".into());
    }
    if rack.world().body("drone").and_then(|b| b.hold_ned) != Some(hold) {
        return Err("hull-slot commands must not clear aerial hold".into());
    }

    let idle = Command {
        slot: 0,
        velocity_ned: [0.0, 3.0, -5.0],
        apply: 0,
    };
    let decoded = command_from_datagram(&crate::encode_command(idle))
        .ok_or_else(|| "encode/decode apply=0".to_string())?;
    let zeroed = RackCommand::from_fch1(&[decoded]);
    if zeroed.aerial != [0.0, 0.0, 0.0] {
        return Err(format!(
            "from_fch1 apply=0 must zero aerial, got {:?}",
            zeroed.aerial
        ));
    }
    card.send_command(idle)
        .map_err(|e| format!("apply0 cmd: {e}"))?;
    let f = rack
        .frame_from_io(0.02, 1_000_000)
        .map_err(|e| format!("apply0 frame: {e}"))?;
    if f.missed() {
        return Err("apply0 frame missed".into());
    }
    if rack.world().body("drone").and_then(|b| b.hold_ned) != Some(hold) {
        return Err("apply=0 must not revive or clear hold".into());
    }

    card.send_command(Command {
        slot: 0,
        velocity_ned: [0.0, 0.0, -1.2],
        apply: 1,
    })
    .map_err(|e| format!("climb cmd: {e}"))?;
    let f = rack
        .frame_from_io(0.02, 1_000_000)
        .map_err(|e| format!("climb frame: {e}"))?;
    if f.missed() {
        return Err("climb frame missed".into());
    }
    if rack
        .world()
        .body("drone")
        .and_then(|b| b.hold_ned)
        .is_some()
    {
        return Err("live apply=1 climb must clear hold".into());
    }

    let samples = card.drain_samples();
    if samples == 0 || card.samples_received() == 0 {
        return Err("card received no FCH1 samples".into());
    }
    if !card
        .log()
        .iter()
        .any(|e| matches!(e, Fch1WireEvent::RxSample(s) if s.slot == 0))
    {
        return Err("card never saw drone sample slot 0".into());
    }
    if !card
        .log()
        .iter()
        .any(|e| matches!(e, Fch1WireEvent::RxSample(s) if s.slot == 1))
    {
        return Err("card never saw rover sample slot 1".into());
    }

    Ok(Fch1MockReport {
        frames: rack.frames(),
        samples_rx: card.samples_received(),
        jsonl: card.to_jsonl(),
    })
}

/// Result of [`run_fch1_udp_mock`].
#[derive(Clone, Debug)]
pub struct Fch1MockReport {
    pub frames: u64,
    pub samples_rx: usize,
    pub jsonl: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::encode_command;

    #[test]
    fn inland_udp_mock_is_not_the_plant() {
        let report = run_fch1_udp_mock().expect("udp mock");
        assert!(report.frames >= 3, "frames={}", report.frames);
        assert!(report.samples_rx >= 2, "samples={}", report.samples_rx);
        assert_eq!(
            report.jsonl,
            include_str!("../corpus/fch1_udp_mock.jsonl"),
            "recorded FCH1 UDP mock log must lockstep"
        );
    }

    #[test]
    fn open_water_rover_slot_does_not_create_a_chassis() {
        let mut card = Fch1UdpCard::bind().expect("card");
        let card_addr = card.local_addr().expect("card addr");
        let mut rack = WorldRack::open_water(1).expect("open_water");
        let rack_addr = rack.bind_io(&card_addr.to_string()).expect("bind_io");
        card.set_peer(rack_addr).expect("peer");
        card.send_command(Command {
            slot: 1,
            velocity_ned: [-0.4, 0.0, 0.0],
            apply: 1,
        })
        .expect("rover slot");
        let f = rack.frame_from_io(0.02, 1_000_000).expect("frame");
        assert!(!f.missed());
        assert!(rack.world().body("rover").is_none());
        assert!(rack.world().body("skiff").is_some());
        let n = card.drain_samples();
        assert!(n >= 1);
        assert!(card
            .log()
            .iter()
            .any(|e| matches!(e, Fch1WireEvent::RxSample(s) if s.slot == 2)));
        assert!(!card
            .log()
            .iter()
            .any(|e| matches!(e, Fch1WireEvent::RxSample(s) if s.slot == 1)));
    }

    #[test]
    fn disconnected_card_cannot_send() {
        let mut card = Fch1UdpCard::bind().expect("card");
        let err = card.send_command(Command {
            slot: 0,
            velocity_ned: [0.0, 0.0, -1.0],
            apply: 1,
        });
        assert!(matches!(err, Err(BackendError::Disconnected)));
    }

    #[test]
    fn apply_zero_payload_is_ignored_after_udp_decode() {
        let wire = encode_command(Command {
            slot: 1,
            velocity_ned: [-0.4, 0.2, 0.1],
            apply: 0,
        });
        let c = command_from_datagram(&wire).expect("decode");
        let cmd = RackCommand::from_fch1(&[c]);
        assert_eq!(cmd.ground, [0.0, 0.0, 0.0]);
        assert_eq!(cmd.aerial, [0.0, 0.0, 0.0]);
    }
}
