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
            pub const CREUSOT: &'static str = $crate::safety::AERIAL_OFFBOARD_CREUSOT;
            pub const FAULTS: &'static str = $crate::safety::AERIAL_OFFBOARD_FAULTS;

            pub const fn revokes(event: $crate::safety::Event) -> bool {
                $crate::safety::event_revokes_authority(event)
            }

            /// Fault-lab inject: only kernel revoke events are injectable.
            /// `None` means the event is not a generated fault (e.g. `MissionCommand`).
            pub const fn inject(event: $crate::safety::Event) -> Option<$crate::safety::Event> {
                if Self::revokes(event) {
                    Some(event)
                } else {
                    None
                }
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

            pub const fn inject(event: $crate::safety::Event) -> Option<$crate::safety::Event> {
                if Self::revokes(event) {
                    Some(event)
                } else {
                    None
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

impl AerialOffboard {
    /// Leftover GPS-loss monitors (invalid `Estimate` / `EstimatorInvalid`).
    /// World `Scenario::GPS_LOSS.require` must alias this slice so companion
    /// leftover runners cannot drift from the scenario lab.
    pub const GPS_LOSS_REQUIRE: &'static [crate::contracts::Requirement] = &[
        crate::contracts::Requirement::NeverActuateWhileDisarmed,
        crate::contracts::Requirement::ActuatorsImplyArmed,
        crate::contracts::Requirement::NoNanCommands,
        crate::contracts::Requirement::AltitudeBelow { meters: 120.0 },
        crate::contracts::Requirement::PermitEpochMonotonic,
        crate::contracts::Requirement::FailsafeWithinMs(
            crate::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS,
        ),
        crate::contracts::Requirement::EpochBumped,
        crate::contracts::Requirement::CommandAgeMs {
            max_ms: crate::safety::COMMAND_MAX_AGE_MS,
        },
        crate::contracts::Requirement::EstimatorTimestampsMonotonic,
    ];
}

/// Pass the aerial OffboardControl command idents to a callback macro.
///
/// Must stringify to [`crate::safety::AERIAL_OFFBOARD_COMMANDS`]. A new ident
/// without an [`impl_aerial_offboard_command`] arm fails `cargo check`.
#[macro_export]
macro_rules! with_aerial_offboard_commands {
    ($callback:path) => {
        $callback!(set_velocity, set_position, hold);
    };
}

/// One OffboardControl command generated from the aerial table.
///
/// Adding a name to `define_aerial_authority!` `commands` without an arm here
/// fails `cargo check`.
#[macro_export]
macro_rules! impl_aerial_offboard_command {
    (set_velocity) => {
        /// Same grant as [`Self::set_velocity`] without stepping the plant.
        pub fn set_velocity_now(&mut self, velocity: Velocity<Ned>) -> Result<(), ErrorKind> {
            self.admit_offboard_now()?;
            self.inner
                .backend
                .set_velocity_ned_now(velocity)
                .map_err(ErrorKind::Backend)
        }

        /// Same grant as [`Self::set_velocity_now`], then one backend tick.
        pub async fn set_velocity(&mut self, velocity: Velocity<Ned>) -> Result<(), ErrorKind> {
            self.set_velocity_now(velocity)?;
            self.inner
                .backend
                .tick(0.02)
                .await
                .map_err(ErrorKind::Backend)?;
            Ok(())
        }
    };
    (set_position) => {
        /// Same grant as [`Self::set_position`] without stepping the plant.
        pub fn set_position_now(&mut self, position: Position<Ned>) -> Result<(), ErrorKind> {
            self.admit_offboard_now()?;
            self.inner
                .backend
                .set_position_ned_now(position)
                .map_err(ErrorKind::Backend)
        }

        /// Same grant as [`Self::set_position_now`], then one backend tick.
        pub async fn set_position(&mut self, position: Position<Ned>) -> Result<(), ErrorKind> {
            self.set_position_now(position)?;
            self.inner
                .backend
                .tick(0.02)
                .await
                .map_err(ErrorKind::Backend)?;
            Ok(())
        }
    };
    (hold) => {
        /// Hold at the current NED pose. Same grant as [`Self::set_position_now`].
        pub fn hold_now(&mut self) -> Result<(), ErrorKind> {
            self.admit_offboard_now()?;
            self.inner.backend.hold_now().map_err(ErrorKind::Backend)
        }

        /// Same grant as [`Self::hold_now`], then one backend tick.
        pub async fn hold(&mut self) -> Result<(), ErrorKind> {
            self.hold_now()?;
            self.inner
                .backend
                .tick(0.02)
                .await
                .map_err(ErrorKind::Backend)?;
            Ok(())
        }
    };
}

/// Dummy now-call for [`Vehicle::for_each_offboard_now`]. Same idents as
/// [`impl_aerial_offboard_command`].
#[macro_export]
macro_rules! call_aerial_offboard_now {
    ($v:ident, set_velocity) => {
        $v.set_velocity_now(Velocity::<Ned>::ned(0.0, 0.0, -0.2))
    };
    ($v:ident, set_position) => {
        $v.set_position_now(Position::<Ned>::ned(0.0, 0.0, -2.0))
    };
    ($v:ident, hold) => {
        $v.hold_now()
    };
}

/// Generate OffboardControl now-methods, async wrappers, and stamped-command
/// apply from the aerial command table.
///
/// Expand in `vehicle/typestate.rs` (same module as [`Vehicle`] private fields).
/// `()` dispatches [`with_aerial_offboard_commands`]; those idents must
/// stringify to [`crate::safety::AERIAL_OFFBOARD_COMMANDS`].
#[macro_export]
macro_rules! impl_aerial_offboard_now {
    () => {
        $crate::with_aerial_offboard_commands!($crate::impl_aerial_offboard_now);
    };
    ($($cmd:ident),+) => {
        /// Public command names generated with the `*_now` methods.
        pub const OFFBOARD_NOW_COMMANDS: &'static [&'static str] = &[$(stringify!($cmd)),+];

        impl<S: OffboardControl, B: VehicleBackend> Vehicle<S, B> {
            /// AerialOffboard command gate: live permit, then kernel
            /// `HeartbeatFresh` and `MissionCommand`.
            pub fn admit_offboard_now(&mut self) -> Result<(), ErrorKind> {
                self.require_live_permit()?;
                self.apply_event(Event::HeartbeatFresh)?;
                self.apply_event(Event::MissionCommand)?;
                Ok(())
            }

            $($crate::impl_aerial_offboard_command!($cmd);)+

            /// Apply a stamped planner command. Permit must still be live **and**
            /// the command younger than [`crate::safety::COMMAND_MAX_AGE_MS`].
            pub fn apply_velocity_command_now(
                &mut self,
                command: Command<Velocity<Ned>>,
            ) -> Result<(), ErrorKind> {
                let now = self.inner.backend.authority_now();
                if !command.deadline().met(now) || command.check_age(now).is_err() {
                    return Err(ErrorKind::StaleAuthority(AuthorityReject::StaleCommand));
                }
                self.require_command_age(command.age_ms(now))?;
                self.set_velocity_now(command.payload)
            }

            /// Invoke every generated OffboardControl now-method.
            ///
            /// The leftover fault lab uses this so a command added to the
            /// kernel table is either covered here or does not compile.
            pub fn for_each_offboard_now<F>(&mut self, mut f: F)
            where
                F: FnMut(&'static str, Result<(), ErrorKind>),
            {
                $(
                    f(
                        stringify!($cmd),
                        $crate::call_aerial_offboard_now!(self, $cmd),
                    );
                )+
            }

            /// Leftover Offboard after a revoke: every generated command is
            /// `StaleAuthority` and the handle is still typed Offboard.
            /// World `run_revoke_table`, PX4 `run_px4_revoke_table` /
            /// `run_px4_gps_loss`, ArduPilot `run_ardupilot_gps_loss`, HITL
            /// `run_hitl_gps_loss`, ROS 2 `run_ros2_gps_loss`, HITL
            /// `run_hitl_revoke_table`, and ROS 2 `run_ros2_revoke_table` share this.
            pub fn leftover_commands_stale(&mut self) -> Result<(), ErrorKind> {
                let expected = $crate::contracts::AerialOffboard::COMMANDS;
                let mut i = 0usize;
                let mut fail = false;
                self.for_each_offboard_now(|name, result| {
                    if expected.get(i) != Some(&name)
                        || !matches!(result, Err(ErrorKind::StaleAuthority(_)))
                    {
                        fail = true;
                    }
                    i += 1;
                });
                if fail || i != expected.len() || !self.safety().offboard {
                    return Err(ErrorKind::Backend(BackendError::Rejected(
                        "leftover_offboard_still_has_authority",
                    )));
                }
                Ok(())
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
            assert_eq!(
                AerialOffboard::inject(e).is_some(),
                event_revokes_authority(e)
            );
            if let Some(inj) = AerialOffboard::inject(e) {
                assert_eq!(inj, e);
            }
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
            assert_eq!(
                AerialOffboard::inject(e).is_some(),
                AerialOffboard::revokes(e),
                "inject({e:?}) must track revokes"
            );
            if AerialOffboard::revokes(e) {
                assert_eq!(AerialOffboard::inject(e), Some(e));
            } else {
                assert_eq!(AerialOffboard::inject(e), None);
            }
        }
        assert_eq!(AerialOffboard::inject(Event::MissionCommand), None);
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
        assert!(AerialOffboard::GPS_LOSS_REQUIRE.contains(&Requirement::EpochBumped));
        assert!(
            AerialOffboard::GPS_LOSS_REQUIRE.contains(&Requirement::FailsafeWithinMs(
                OFFBOARD_HEARTBEAT_MAX_AGE_MS
            ))
        );
        assert!(
            AerialOffboard::GPS_LOSS_REQUIRE.contains(&Requirement::CommandAgeMs {
                max_ms: COMMAND_MAX_AGE_MS
            })
        );
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
        let spec_tokens = tokens(AerialOffboard::SPEC);
        let human_tokens = tokens(human_readable_spec());
        assert!(
            human_tokens
                .windows(spec_tokens.len())
                .any(|w| w == spec_tokens.as_slice()),
            "human_readable_spec must embed AerialOffboard::SPEC"
        );
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
        let generated_creusot =
            include_str!("../../../../docs/generated/aerial-offboard.creusot.txt");
        assert_eq!(
            tokens(AerialOffboard::CREUSOT),
            tokens(generated_creusot),
            "docs/generated/aerial-offboard.creusot.txt must match AerialOffboard::CREUSOT"
        );
        for e in AerialOffboard::REVOKE_ON {
            assert!(
                AerialOffboard::CREUSOT.contains(e.name()),
                "Creusot listing missing {}",
                e.name()
            );
        }
        let generated_faults =
            include_str!("../../../../docs/generated/aerial-offboard.faults.txt");
        assert_eq!(
            tokens(AerialOffboard::FAULTS),
            tokens(generated_faults),
            "docs/generated/aerial-offboard.faults.txt must match AerialOffboard::FAULTS"
        );
        assert!(AerialOffboard::FAULTS.starts_with("inject "));
        assert!(AerialOffboard::FAULTS.contains("refuse "));
        for c in AerialOffboard::COMMANDS {
            assert!(
                AerialOffboard::FAULTS.contains(c),
                "FAULTS listing missing command {c}"
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
        for c in AerialOffboard::COMMANDS {
            assert!(
                faults_md.contains(c),
                "docs/generated/aerial-offboard.faults.md missing leftover refuse {c}"
            );
        }
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
