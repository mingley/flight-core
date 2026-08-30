//! Single-source vehicle contract.
//!
//! Aerial offboard authority is declared once by `define_aerial_authority!`
//! in `safety`. [`vehicle_contract!`] `from_kernel` aliases that table so the
//! typestate capability, runtime monitors, Kani harness name, diagrams, and
//! traceability ids cannot drift from the kernel predicates.

/// Declare a named capability contract.
///
/// `from_kernel` binds a capability to the aerial authority table in `safety`.
/// The event-list form remains for additional capabilities that do not yet
/// have a kernel predicate.
#[macro_export]
macro_rules! vehicle_contract {
    (capability $name:ident { from_kernel }) => {
        #[doc = concat!("Contract capability `", stringify!($name), "`.")]
        pub struct $name;

        impl $name {
            pub const HEARTBEAT_MAX_AGE_MS: u32 = $crate::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS;
            pub const COMMAND_MAX_AGE_MS: u32 = $crate::safety::COMMAND_MAX_AGE_MS;
            pub const TRACE_ID: &'static str = concat!("FC-CAP-", stringify!($name));
            pub const REVOKE_ON: &'static [$crate::safety::Event] =
                $crate::safety::AUTHORITY_REVOKE_EVENTS;
            pub const MERMAID: &'static str = $crate::safety::AERIAL_OFFBOARD_MERMAID;
            pub const SPEC: &'static str = $crate::safety::AERIAL_OFFBOARD_SPEC;
            pub const GRAPHVIZ: &'static str = $crate::safety::AERIAL_OFFBOARD_GRAPHVIZ;

            pub const fn revokes(event: $crate::safety::Event) -> bool {
                $crate::safety::event_revokes_authority(event)
            }

            /// Kernel admission for this capability (heartbeat ∧ command age).
            pub const fn admit(heartbeat_age_ms: u32, command_age_ms: u32) -> bool {
                $crate::safety::admit_offboard_command(heartbeat_age_ms, command_age_ms)
            }

            pub const TRANSITIONS: &'static [$crate::safety::ContractEdge] =
                $crate::safety::AERIAL_OFFBOARD_TRANSITIONS;
            pub const GATE: &'static str = "OffboardControl";
            pub const COMMANDS: &'static [&'static str] = $crate::safety::AERIAL_OFFBOARD_COMMANDS;
            /// Compile-fail UI tests that must exist for this capability gate.
            pub const UI_FORBIDDEN: &'static [&'static str] = &[
                "ready_velocity.rs",
                "armed_velocity.rs",
                "unsafe_mission.rs",
                "disarmed_velocity.rs",
                "disconnected_velocity.rs",
                "failsafe_offboard.rs",
                "recovery_velocity.rs",
                "ready_position.rs",
                "ready_hold.rs",
            ];

            pub const MONITORS: &'static [$crate::contracts::Requirement] = &[
                $crate::contracts::Requirement::NeverActuateWhileDisarmed,
                $crate::contracts::Requirement::ActuatorsImplyArmed,
                $crate::contracts::Requirement::PermitEpochMonotonic,
                $crate::contracts::Requirement::NoNanCommands,
                $crate::contracts::Requirement::OffboardHeartbeatFresh,
                $crate::contracts::Requirement::CommandAgeMs {
                    max_ms: $crate::safety::COMMAND_MAX_AGE_MS,
                },
                $crate::contracts::Requirement::EstimatorTimestampsMonotonic,
                $crate::contracts::Requirement::OffboardAdmitted,
            ];

            pub const KANI_HARNESS: &'static str = "dsl_revokes_match_kernel";

            /// Runtime monitors generated from this capability table.
            pub fn evaluate(
                samples: &[$crate::contracts::TraceSample],
            ) -> Result<(), $crate::contracts::MonitorFail> {
                $crate::contracts::evaluate_trace(samples, Self::MONITORS)
            }
        }
    };
    (
        capability $name:ident {
            heartbeat_age_ms: $hb:expr,
            revokes_on: [$($ev:ident),* $(,)?]
        }
    ) => {
        #[doc = concat!("Contract capability `", stringify!($name), "`.")]
        pub struct $name;

        impl $name {
            pub const HEARTBEAT_MAX_AGE_MS: u32 = $hb;
            pub const TRACE_ID: &'static str = concat!("FC-CAP-", stringify!($name));
            pub const REVOKE_ON: &'static [$crate::safety::Event] = &[
                $($crate::safety::Event::$ev),*
            ];
            pub const MERMAID: &'static str = concat!(
                "stateDiagram-v2\n",
                "    [*] --> Offboard\n",
                "    Offboard --> Failsafe: ",
                stringify!($($ev)|*),
                "\n"
            );
            pub const SPEC: &'static str = concat!(
                "capability ",
                stringify!($name),
                " {\n  requires heartbeat.age < ",
                stringify!($hb),
                ".ms();\n  revokes_on [",
                stringify!($($ev),*),
                "]\n}\n"
            );

            pub const fn revokes(event: $crate::safety::Event) -> bool {
                match event {
                    $($crate::safety::Event::$ev => true,)*
                    _ => false,
                }
            }

            pub const GRAPHVIZ: &'static str = concat!(
                "digraph ",
                stringify!($name),
                " {\n  Offboard -> Failsafe [label=\"",
                stringify!($($ev)|*),
                "\"];\n}\n"
            );

            pub const MONITORS: &'static [$crate::contracts::Requirement] = &[
                $crate::contracts::Requirement::NeverActuateWhileDisarmed,
                $crate::contracts::Requirement::ActuatorsImplyArmed,
            ];

            pub const KANI_HARNESS: &'static str = "dsl_revokes_match_kernel";
        }
    };
}

vehicle_contract! {
    capability AerialOffboard {
        from_kernel
    }
}

/// Generate OffboardControl `*_now` methods from the aerial command table.
///
/// Expand in `vehicle/typestate.rs` (same module as [`Vehicle`] private fields).
/// Public names must match [`crate::safety::AERIAL_OFFBOARD_COMMANDS`].
#[macro_export]
macro_rules! impl_aerial_offboard_now {
    () => {
        /// Public command names generated with the `*_now` methods.
        pub const OFFBOARD_NOW_COMMANDS: &'static [&'static str] =
            &["set_velocity", "set_position", "hold"];

        impl<S: OffboardControl, B: VehicleBackend> Vehicle<S, B> {
            /// AerialOffboard command gate: live permit, then kernel
            /// `HeartbeatFresh` and `MissionCommand`.
            pub fn admit_offboard_now(&mut self) -> Result<(), ErrorKind> {
                self.require_live_permit()?;
                self.apply_event(Event::HeartbeatFresh)?;
                self.apply_event(Event::MissionCommand)?;
                Ok(())
            }

            /// Same grant as [`Self::set_velocity`] without stepping the plant.
            pub fn set_velocity_now(&mut self, velocity: Velocity<Ned>) -> Result<(), ErrorKind> {
                self.admit_offboard_now()?;
                self.inner
                    .backend
                    .set_velocity_ned_now(velocity)
                    .map_err(ErrorKind::Backend)
            }

            /// Same grant as [`Self::set_position`] without stepping the plant.
            pub fn set_position_now(&mut self, position: Position<Ned>) -> Result<(), ErrorKind> {
                self.admit_offboard_now()?;
                self.inner
                    .backend
                    .set_position_ned_now(position)
                    .map_err(ErrorKind::Backend)
            }

            /// Hold at the current NED pose. Same grant as [`Self::set_position_now`].
            pub fn hold_now(&mut self) -> Result<(), ErrorKind> {
                self.admit_offboard_now()?;
                self.inner.backend.hold_now().map_err(ErrorKind::Backend)
            }
        }
    };
}

/// Kani harness generated from the aerial authority table.
///
/// Expand inside `flight-verify`'s `#[cfg(kani)]` proofs module:
/// `flight_core::prove_aerial_authority!();`
#[macro_export]
macro_rules! prove_aerial_authority {
    () => {
        #[kani::proof]
        fn dsl_revokes_match_kernel() {
            use flight_core::contracts::AerialOffboard;
            use flight_core::safety::{
                admit_offboard_command, command_age_ok, estimator_ts_monotonic,
                event_revokes_authority, heartbeat_age_ok, Event, AUTHORITY_REVOKE_EVENTS,
                COMMAND_MAX_AGE_MS, OFFBOARD_HEARTBEAT_MAX_AGE_MS,
            };
            let bits: u8 = kani::any();
            kani::assume(bits <= 23);
            let Some(e) = Event::from_u8(bits) else {
                return;
            };
            let mut in_table = false;
            let mut i = 0;
            while i < AUTHORITY_REVOKE_EVENTS.len() {
                if AUTHORITY_REVOKE_EVENTS[i] == e {
                    in_table = true;
                    break;
                }
                i += 1;
            }
            assert_eq!(event_revokes_authority(e), in_table);
            assert_eq!(AerialOffboard::revokes(e), event_revokes_authority(e));
            let age: u32 = kani::any();
            assert_eq!(heartbeat_age_ok(age), age < OFFBOARD_HEARTBEAT_MAX_AGE_MS);
            assert_eq!(command_age_ok(age), age < COMMAND_MAX_AGE_MS);
            let hb: u32 = kani::any();
            let cmd: u32 = kani::any();
            assert_eq!(
                admit_offboard_command(hb, cmd),
                heartbeat_age_ok(hb) && command_age_ok(cmd)
            );
            assert_eq!(
                AerialOffboard::admit(hb, cmd),
                admit_offboard_command(hb, cmd)
            );
            let prev: u64 = kani::any();
            let next: u64 = kani::any();
            assert_eq!(estimator_ts_monotonic(prev, next), next >= prev);
        }
    };
}

/// Kernel invariant `actuators_enabled → armed` (traceability FC-INV-001).
pub const INV_ACTUATORS_IMPLY_ARMED: &str = "FC-INV-001";
/// Permit epoch must match the live backend (FC-INV-002).
pub const INV_PERMIT_EPOCH: &str = "FC-INV-002";
/// Offboard requires a fresh heartbeat (FC-INV-003).
pub const INV_OFFBOARD_HEARTBEAT: &str = "FC-INV-003";
/// Command age at actuation must be inside the contract bound (FC-INV-004).
pub const INV_COMMAND_AGE: &str = "FC-INV-004";
/// Estimator timestamps are monotonic (FC-INV-005).
pub const INV_ESTIMATOR_TS: &str = "FC-INV-005";

/// Human-readable specification emitted from the same tables as the types.
pub fn human_readable_spec() -> &'static str {
    concat!(
        "flight-core safety contract\n",
        "============================\n",
        "capability AerialOffboard {\n",
        "  requires heartbeat.age < 250.ms();\n",
        "  requires command.age < 100.ms();\n",
        "  revokes_on [TriggerFailsafe, Disarm, Disconnect, HeartbeatStale, EstimatorInvalid, ImuUnhealthy]\n",
        "  commands [set_velocity, set_position, hold]\n",
        "}\n",
        "invariant {\n  actuators.enabled -> armed  [FC-INV-001]\n",
        "  ActuationPermit.epoch == backend.epoch  [FC-INV-002]\n",
        "  OffboardControl => heartbeat_age < 250 ms  [FC-INV-003]\n",
        "  command.age < 100 ms at actuation  [FC-INV-004]\n",
        "  estimator timestamps monotonic  [FC-INV-005]\n",
        "  admit_offboard := heartbeat.age < 250 && command.age < 100\n",
        "}\n"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::Requirement;
    use crate::safety::{
        event_revokes_authority, Event, AUTHORITY_REVOKE_EVENTS, COMMAND_MAX_AGE_MS,
        OFFBOARD_HEARTBEAT_MAX_AGE_MS,
    };

    #[test]
    fn dsl_is_the_kernel_table() {
        assert!(core::ptr::eq(
            AerialOffboard::REVOKE_ON,
            AUTHORITY_REVOKE_EVENTS
        ));
        assert!(core::ptr::eq(
            AerialOffboard::COMMANDS,
            crate::safety::AERIAL_OFFBOARD_COMMANDS
        ));
        assert_eq!(
            AerialOffboard::COMMANDS,
            &["set_velocity", "set_position", "hold"]
        );
        for e in Event::ALL {
            assert_eq!(
                AerialOffboard::revokes(e),
                event_revokes_authority(e),
                "event {e:?} disagrees between capability and kernel"
            );
        }
    }

    #[test]
    fn heartbeat_and_command_bounds_match_kernel() {
        assert_eq!(
            AerialOffboard::HEARTBEAT_MAX_AGE_MS,
            OFFBOARD_HEARTBEAT_MAX_AGE_MS
        );
        assert_eq!(AerialOffboard::COMMAND_MAX_AGE_MS, COMMAND_MAX_AGE_MS);
        assert_eq!(AerialOffboard::SPEC, crate::safety::AERIAL_OFFBOARD_SPEC);
        assert!(AerialOffboard::admit(0, 0));
        assert!(AerialOffboard::admit(249, 99));
        assert!(!AerialOffboard::admit(250, 0));
        assert!(!AerialOffboard::admit(0, 100));
        assert_eq!(
            AerialOffboard::admit(1, 1),
            crate::safety::admit_offboard_command(1, 1)
        );
    }

    #[test]
    fn revoke_table_is_the_named_set() {
        assert!(AerialOffboard::REVOKE_ON.contains(&Event::TriggerFailsafe));
        assert!(AerialOffboard::REVOKE_ON.contains(&Event::Disarm));
        assert!(!AerialOffboard::REVOKE_ON.contains(&Event::MissionCommand));
        assert!(AerialOffboard::TRACE_ID.starts_with("FC-CAP-"));
        assert!(AerialOffboard::MERMAID.contains("Offboard"));
        assert!(AerialOffboard::GRAPHVIZ.contains("digraph"));
        assert_eq!(AerialOffboard::KANI_HARNESS, "dsl_revokes_match_kernel");
        assert!(AerialOffboard::MONITORS.contains(&Requirement::OffboardHeartbeatFresh));
        assert!(
            AerialOffboard::MONITORS.contains(&Requirement::CommandAgeMs {
                max_ms: COMMAND_MAX_AGE_MS
            })
        );
        assert!(AerialOffboard::MONITORS.contains(&Requirement::EstimatorTimestampsMonotonic));
        assert!(AerialOffboard::MONITORS.contains(&Requirement::OffboardAdmitted));
        let generated = include_str!("../../../../docs/generated/aerial-offboard.mmd");
        fn tokens(s: &str) -> Vec<&str> {
            s.split_whitespace().collect()
        }
        assert_eq!(
            tokens(AerialOffboard::MERMAID),
            tokens(generated),
            "docs/generated/aerial-offboard.mmd must match AerialOffboard::MERMAID"
        );
        assert!(human_readable_spec().contains("FC-INV-001"));
        assert!(human_readable_spec().contains("FC-INV-004"));
        assert!(human_readable_spec().contains("command.age < 100"));
        assert!(human_readable_spec().contains("commands [set_velocity"));
        assert!(human_readable_spec().contains("admit_offboard"));
        assert!(AerialOffboard::SPEC.contains("commands [set_velocity"));
        assert_eq!(AerialOffboard::GATE, "OffboardControl");
        assert!(AerialOffboard::COMMANDS.contains(&"set_velocity"));
        let revoke_vias: Vec<&str> = AerialOffboard::TRANSITIONS
            .iter()
            .filter(|e| e.to == "Failsafe")
            .map(|e| e.via)
            .collect();
        for e in AerialOffboard::REVOKE_ON {
            assert!(
                revoke_vias.contains(&e.name()),
                "transition table missing Failsafe via {}",
                e.name()
            );
            assert!(AerialOffboard::MERMAID.contains(e.name()));
        }
        let generated_dot = include_str!("../../../../docs/generated/aerial-offboard.dot");
        assert_eq!(
            tokens(AerialOffboard::GRAPHVIZ),
            tokens(generated_dot),
            "docs/generated/aerial-offboard.dot must match AerialOffboard::GRAPHVIZ"
        );
        let generated_spec = include_str!("../../../../docs/generated/aerial-offboard.spec.txt");
        assert_eq!(
            tokens(AerialOffboard::SPEC),
            tokens(generated_spec),
            "docs/generated/aerial-offboard.spec.txt must match AerialOffboard::SPEC"
        );
        assert_eq!(
            AerialOffboard::TRANSITIONS.len(),
            3 + AerialOffboard::REVOKE_ON.len()
        );
        let transitions_md =
            include_str!("../../../../docs/generated/aerial-offboard.transitions.md");
        for e in AerialOffboard::TRANSITIONS {
            let row = format!("| {} | {} | {} |", e.from, e.via, e.to);
            assert!(
                transitions_md.contains(&row),
                "docs/generated/aerial-offboard.transitions.md missing {row}"
            );
        }
        let faults_md = include_str!("../../../../docs/generated/aerial-offboard.faults.md");
        for e in AerialOffboard::REVOKE_ON {
            assert!(
                faults_md.contains(e.name()),
                "docs/generated/aerial-offboard.faults.md missing {}",
                e.name()
            );
        }
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        for f in AerialOffboard::UI_FORBIDDEN {
            assert!(
                root.join("tests/ui").join(f).is_file(),
                "capability UI {f} must exist"
            );
        }
        let live = crate::contracts::TraceSample {
            armed: true,
            actuators_enabled: true,
            command: Some([0.0, 0.0, -1.0]),
            ..crate::contracts::TraceSample::default()
        };
        assert!(AerialOffboard::evaluate(&[live]).is_ok());
        let stale_hb = crate::contracts::TraceSample {
            heartbeat_age_ms: 250,
            ..live
        };
        assert!(AerialOffboard::evaluate(&[stale_hb]).is_err());
    }
}
